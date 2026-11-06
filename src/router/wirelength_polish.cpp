#include "router/negotiated_router.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace eda {
// The net's shortest path on its own TSV slot with every route, reserved TSV
// demand and negotiation history hidden from the search; ledgers and routes
// are not touched.
NetRoute NegotiatedRouter::free_path(size_t index) {
    const double saved_history=history_scale_;
    struct Restore {std::function<void()> f;~Restore(){f();}} restore{[&] {
        ignore_route_demand_=false;v7_refinement_active_=false;history_scale_=saved_history;
    }};
    ignore_route_demand_=true;history_scale_=0;
    const bool maze=options_.v7_rrr_astar_expansions>0 && options_.v7_rrr_astar_seconds>0;
    v7_refinement_active_=maze;
    if(maze) {
        v7_refinement_expansion_start_=counters_.expansions;
        v7_refinement_deadline_=std::min(deadline_,std::chrono::steady_clock::now()+
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(options_.v7_rrr_astar_seconds)));
    }
    ++counters_.wirelength_polish_free_searches;
    return build_route(index,routes_.at(index).slot);
}
// Nets that fill the cells of this net's shortest path. A single-net move can
// only use room that is already free, so a net whose short path other nets
// fill keeps its detour however often it is rerouted alone. The path comes
// from the normal search on the same TSV slot with every route and reserved
// TSV demand hidden, so only fixed PIN/OBS can fill a cell. A cell of that
// path is blocked when PIN/OBS leave room but other routes fill it; a cell
// that PIN/OBS fill alone cannot be freed and is skipped. Owners are taken
// greedily, the net freeing the most blocked cells first and then the smaller
// net, until every blocked cell has one or the group is full. Empty when the
// free path is not at least one GCell shorter than the current route. Only
// demand and geometry decide; nothing depends on the design's name or place.
std::vector<size_t> NegotiatedRouter::detour_blockers(size_t index) {
    const auto& current=routes_.at(index);
    NetRoute open_path;
    try {open_path=free_path(index);}
    catch(const std::exception& error) {
        ++rejection_reasons_[std::string("polish free path failed: ")+error.what()];
        return {};
    }
    const auto& m=options_.official_metrics;
    const double pitch=std::max(m.gcell_width_dbu,m.gcell_height_dbu);
    if(!(open_path.measured.wirelength<=current.measured.wirelength-pitch)) return {};
    std::unordered_map<int,int> own;
    for(auto [node,amount]:current.measured.demand) own[node]+=amount;
    std::set<int> blocked;
    for(auto [node,amount]:open_path.measured.demand) {
        if(amount<=0) continue;
        const double cap=graph_.official_capacity(node);
        const double fixed=graph_.official_obstacle_demand(node);
        if(cap<=0 || fixed>=cap) continue;
        const auto found=own.find(node);
        const double others=double(search_cell_demand(node))-(found==own.end()?0:found->second);
        if(fixed+others+amount>cap) blocked.insert(node);
    }
    if(blocked.empty()) return {};
    std::map<size_t,std::vector<int>> covers;
    for(int cell:blocked) for(int owner:cell_owners(cell))
        if(size_t(owner)!=index) covers[size_t(owner)].push_back(cell);
    std::vector<size_t> result;
    while(result.size()<options_.wirelength_polish_group && !blocked.empty()) {
        size_t best=std::numeric_limits<size_t>::max(),best_count=0,best_size=0;
        for(const auto& [net,cells]:covers) {
            size_t count=0;for(int c:cells) count+=blocked.count(c);
            const size_t size=routes_[net].measured.demand.size();
            if(count>best_count || (count && count==best_count && size<best_size)) {best=net;best_count=count;best_size=size;}
        }
        if(!best_count) break;
        result.push_back(best);
        for(int c:covers[best]) blocked.erase(c);
        covers.erase(best);
    }
    return result;
}
// Final wirelength pass. Initial routing of a large design only tries L/Z
// templates, and later RRR gives the maze search only to nets in overflowing
// cells, so most nets never see it. Here every net is rerouted alone with the
// maze search, the longest excess over its own reference first, and a move is
// kept only when the exact complete score improves (same rrr_round journal,
// local proof and periodic full checkpoint as the overflow cleanup). Fixed-slot
// RRR rejects any change of TSV cell, so T cannot change.
//
// Every move keeps its local proof, but the complete audit is spaced by its own
// measured cost so it takes at most about a tenth of this pass. With the shared
// RRR schedule (every 32 accepted moves or 5 s) a 17 s audit on a large design
// left little time for the search itself. The final audit is unchanged.
//
// With wirelength_polish_blockers, a net whose single move is refused next
// tries the group move above: lift it with the nets filling its shortest path,
// route it first, then the others, under the same joint_rrr_round transaction
// and exact complete-score acceptance as the overflow cleanup's group move.
void NegotiatedRouter::run_wirelength_polish() {
    if(!options_.official_model || options_.wirelength_polish_rounds<=0 || timed_out()) return;
    using Clock=std::chrono::steady_clock;
    const auto started=Clock::now();
    const auto saved_present=present_;const auto saved_history=history_scale_;
    const bool saved_incremental=options_.v7_rrr,saved_topology=v5_topology_trial_;
    const size_t saved_maze_nets=options_.v7_rrr_astar_net_limit,saved_scope=v7_scope_limit_;
    const double saved_period=options_.v9_checkpoint_seconds;const size_t saved_groups=options_.v9_checkpoint_groups;
    const bool saved_amortization=options_.checkpoint_amortization;
    struct Restore {std::function<void()> f;~Restore(){f();}} restore{[&] {
        present_=saved_present;history_scale_=saved_history;options_.v7_rrr=saved_incremental;
        v5_topology_trial_=saved_topology;options_.v7_rrr_astar_net_limit=saved_maze_nets;v7_scope_limit_=saved_scope;
        options_.v9_checkpoint_seconds=saved_period;options_.v9_checkpoint_groups=saved_groups;
        options_.checkpoint_amortization=saved_amortization;
        v7_refinement_active_=false;force_rrr_maze_=false;
        counters_.wirelength_polish_seconds+=std::chrono::duration<double>(Clock::now()-started).count();
    }};
    // Plain present cost and no history, as in the overflow cleanup: acceptance
    // is the exact score, so the search should price only real C losses.
    present_=1;history_scale_=0;options_.v7_rrr=true;v5_topology_trial_=true;
    options_.v7_rrr_astar_net_limit=std::max<size_t>(1,options_.v7_rrr_astar_net_limit);
    if(options_.wirelength_polish_blockers)
        options_.v7_rrr_astar_net_limit=std::max(options_.v7_rrr_astar_net_limit,options_.wirelength_polish_group+1);
    force_rrr_maze_=true;
    const bool checkpointed=options_.v9_incremental_rrr;
    // Wait nine audit durations of work after each audit (never less than the
    // shared period), and let accepted moves pile up to a large cap.
    const auto space_audits=[&](double audit_seconds) {
        options_.checkpoint_amortization=false;
        options_.v9_checkpoint_groups=std::max<size_t>(saved_groups,4096);
        options_.v9_checkpoint_seconds=std::max(saved_period,9*audit_seconds);
    };
    if(checkpointed) {
        const auto audit_started=Clock::now();
        begin_rrr_checkpoint();
        space_audits(std::chrono::duration<double>(Clock::now()-audit_started).count());
    }
    // Full audits run on their own spaced schedule after every transaction.
    const auto audit=[&]()->bool {
        if(!checkpointed) return true;
        const auto audits=counters_.v9_full_checkpoints;const auto audit_started=Clock::now();
        if(!v9_rrr_checkpoint_active_ || !finish_rrr_checkpoint()) return false;
        if(counters_.v9_full_checkpoints!=audits) space_audits(std::chrono::duration<double>(Clock::now()-audit_started).count());
        return true;
    };
    bool failed=false;
    try {
        for(int round=0;round<options_.wirelength_polish_rounds && !timed_out() && !failed;++round) {
            ++counters_.wirelength_polish_rounds;
            std::vector<std::pair<double,size_t>> queue;queue.reserve(routes_.size());
            for(size_t i=0;i<routes_.size();++i) {
                const auto& m=routes_[i].measured;
                const double excess=m.wirelength-m.reference;
                if(excess>0) queue.push_back({-excess,i});
            }
            std::sort(queue.begin(),queue.end());
            size_t accepted=0,attempted=0,joint=0,joint_accepted=0;double gain=0;
            for(auto [key,i]:queue) {
                (void)key;if(timed_out()) break;
                const double old_length=routes_[i].measured.wirelength;
                v7_scope_limit_=1;++attempted;++counters_.wirelength_polish_attempted;
                const bool single=rrr_round({i});
                if(single) {
                    ++accepted;++counters_.wirelength_polish_accepted;
                    gain+=old_length-routes_[i].measured.wirelength;
                }
                if(!audit()) {failed=true;break;}
                if(single || !options_.wirelength_polish_blockers || timed_out()) continue;
                const auto blockers=detour_blockers(i);
                if(blockers.empty()) continue;
                std::vector<size_t> scope{i};scope.insert(scope.end(),blockers.begin(),blockers.end());
                double before=0;for(size_t k:scope) before+=routes_[k].measured.wirelength;
                v7_scope_limit_=scope.size();++joint;
                ++counters_.wirelength_polish_joint;counters_.wirelength_polish_joint_nets+=scope.size();
                if(joint_rrr_round(scope)) {
                    double after=0;for(size_t k:scope) after+=routes_[k].measured.wirelength;
                    ++accepted;++joint_accepted;++counters_.wirelength_polish_joint_accepted;
                    gain+=before-after;counters_.wirelength_polish_joint_gain_dbu+=before-after;
                }
                if(!audit()) {failed=true;break;}
            }
            counters_.wirelength_polish_gain_dbu+=gain;
            std::cout<<"[WIRELENGTH_POLISH] round="<<round+1<<" queued="<<queue.size()<<" attempted="<<attempted
                     <<" accepted="<<accepted<<" joint="<<joint<<" joint_accepted="<<joint_accepted
                     <<" gain_dbu="<<gain<<" normalized="<<best_metrics_.guide.normalized
                     <<" quality="<<best_metrics_.guide.quality<<'\n'<<std::flush;
            if(!accepted) break;
        }
    } catch(const std::exception& error) {
        if(checkpointed && v9_rrr_checkpoint_active_) restore_rrr_checkpoint(error.what());
        else throw;
    }
    if(checkpointed) {
        if(v9_rrr_checkpoint_active_) finish_rrr_checkpoint(true);
        v9_rrr_checkpoint_active_=false;
    } else {
        complete_validation_cached_=false;validate_complete();best_metrics_=measure();verify_official_ledger(best_metrics_);
    }
}
}  // namespace eda
