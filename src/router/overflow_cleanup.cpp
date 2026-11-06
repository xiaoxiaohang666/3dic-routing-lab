#include "router/negotiated_router.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <map>
#include <set>

namespace eda {
// Nets that fill the full cells around a net's overflowing cells. A net whose
// terminal is boxed in by fixed occupancy can only leave once one of these
// moves, yet none of them is in an overflowing cell, so the overflow-driven
// queue never selects them. Only cells that routes (not fixed PIN/OBS alone)
// fill are considered; smaller nets come first because they are cheaper to move.
std::vector<size_t> NegotiatedRouter::overflow_blockers(size_t index) const {
    const auto& route=routes_.at(index);
    std::set<int> own(route.nodes.begin(),route.nodes.end());
    if(index<own_pin_nodes_.size()) own.insert(own_pin_nodes_[index].begin(),own_pin_nodes_[index].end());
    for(auto [node,amount]:route.measured.demand) if(amount>0) own.insert(node);
    std::set<int> around;
    for(auto [node,amount]:route.measured.demand) {
        if(amount<=0) continue;
        const double cap=graph_.official_capacity(node);
        if(cap<=0 || graph_.official_obstacle_demand(node)+demand_[size_t(node)]<=cap) continue;
        // The overflowing cell and the net's own cells right next to it.
        std::vector<int> anchors{node};
        for(int next:graph_.neighbors(node)) if(own.count(next)) anchors.push_back(next);
        for(int anchor:anchors) for(int next:graph_.neighbors(anchor)) {
            if(next==node || own.count(next)) continue;
            const double c=graph_.official_capacity(next);
            const double fixed=graph_.official_obstacle_demand(next);
            if(c<=0 || fixed>=c || fixed+demand_[size_t(next)]<c) continue;
            around.insert(next);
        }
    }
    std::map<size_t,size_t> owners;
    for(int cell:around) for(int owner:cell_owners(cell)) {
        if(size_t(owner)==index) continue;
        if(options_.overflow_cleanup_same_die_blockers && def_.nets()[size_t(owner)].is_cross_die) continue;
        owners.emplace(size_t(owner),routes_.at(size_t(owner)).measured.demand.size());
    }
    std::vector<std::pair<size_t,size_t>> ranked;for(auto [net,size]:owners) ranked.push_back({size,net});
    std::sort(ranked.begin(),ranked.end());
    std::vector<size_t> result;
    for(auto [size,net]:ranked) {
        (void)size;if(result.size()>=options_.overflow_cleanup_group) break;
        result.push_back(net);
    }
    return result;
}
// Lift every net of the scope first, then route them in scope order, so the
// first net sees the room its neighbours used to fill. Same journal, local
// proof and exact complete-score acceptance as rrr_round.
bool NegotiatedRouter::joint_rrr_round(const std::vector<size_t>& scope) {
    ++counters_.rrr_attempted;
    const bool incremental=options_.v7_rrr && options_.official_model;
    const auto before=incremental?recovery_metrics(best_metrics_):measure();
    std::vector<std::pair<size_t,NetRoute>> saved;saved.reserve(scope.size());
    if(v9_rrr_checkpoint_active_) journal_rrr_scope(scope);
    std::vector<char> lifted(scope.size(),0),placed(scope.size(),0);
    auto rollback=[&] {
        v7_refinement_active_=false;
        for(size_t k=scope.size();k-->0;) {
            const size_t i=scope[k];
            if(placed[k]) {account(i,routes_[i],-1);placed[k]=0;}
            if(lifted[k]) {routes_[i]=std::move(saved[k].second);account(i,routes_[i],1);lifted[k]=0;}
        }
        saved.clear();best_metrics_=before;++counters_.rollbacks;
        if(incremental) ++counters_.v7_checkpoint_restores;
    };
    try {
        for(size_t k=0;k<scope.size();++k) {
            const size_t i=scope[k];
            saved.emplace_back(i,routes_.at(i));
            account(i,saved.back().second,-1);lifted[k]=1;routes_[i]=NetRoute{};
        }
        size_t maze_nets=0;
        for(size_t k=0;k<scope.size();++k) {
            if(timed_out()) {rollback();return false;}
            const size_t i=scope[k];
            const bool maze=incremental && maze_nets<options_.v7_rrr_astar_net_limit &&
                options_.v7_rrr_astar_expansions>0 && options_.v7_rrr_astar_seconds>0;
            v7_refinement_active_=maze;
            if(maze) {
                ++maze_nets;++counters_.v7_astar_nets;
                v7_refinement_expansion_start_=counters_.expansions;
                v7_refinement_deadline_=std::min(deadline_,std::chrono::steady_clock::now()+
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(options_.v7_rrr_astar_seconds)));
            }
            NetRoute trial;
            try {trial=build_route(i,saved[k].second.slot);}
            catch(const std::exception& error) {
                // Nothing of this net is placed yet; undo the lifted scope locally.
                v7_refinement_active_=false;
                ++rejection_reasons_[std::string("joint RRR route failed: ")+error.what()];
                rollback();return false;
            }
            v7_refinement_active_=false;
            if(incremental && trial.measured.tsv_cells!=saved[k].second.measured.tsv_cells)
                throw std::logic_error("joint RRR changed thermal TSV cells");
            routes_[i]=std::move(trial);account(i,routes_[i],1);placed[k]=1;
        }
        auto after=incremental?recovery_metrics(before):measure();
        if(incremental) for(const auto& [i,old]:saved) {
            after.physical_length+=routes_[i].length-old.length;
            after.vias=after.vias-old.vias+routes_[i].vias;
        }
        if(better(after,before) && !timed_out() && after.official.zero_capacity_cells<=before.official.zero_capacity_cells) {
            if(v9_rrr_checkpoint_active_) {
                validate_rrr_group(saved);
                best_metrics_=after;++counters_.rrr_accepted;++v9_rrr_pending_groups_;
                return true;
            }
            validate_complete();after=measure();
            if(incremental) verify_official_ledger(after);
            if(better(after,before)) {best_metrics_=after;++counters_.rrr_accepted;return true;}
        }
        ++rejection_reasons_["joint RRR did not improve the complete comparison key"];
        rollback();return false;
    } catch(const std::exception& error) {
        v7_refinement_active_=false;
        if(v9_rrr_checkpoint_active_) {restore_rrr_checkpoint(error.what());return false;}
        // Without a journal, undo by hand; a route that failed to build is not placed.
        ++rejection_reasons_[std::string("joint RRR failed: ")+error.what()];
        rollback();return false;
    }
}
void NegotiatedRouter::run_overflow_cleanup() {
    if(!options_.official_model || options_.overflow_cleanup_rounds<=0 || timed_out()) return;
    using Clock=std::chrono::steady_clock;
    const auto started=Clock::now();
    // Search settings follow the replay that showed single nets can leave:
    // plain present cost and no history, since acceptance is the exact score.
    const auto saved_present=present_;const auto saved_history=history_scale_;
    const bool saved_incremental=options_.v7_rrr,saved_topology=v5_topology_trial_;
    const size_t saved_maze_nets=options_.v7_rrr_astar_net_limit,saved_scope=v7_scope_limit_;
    struct Restore {std::function<void()> f;~Restore(){f();}} restore{[&] {
        present_=saved_present;history_scale_=saved_history;options_.v7_rrr=saved_incremental;
        v5_topology_trial_=saved_topology;options_.v7_rrr_astar_net_limit=saved_maze_nets;v7_scope_limit_=saved_scope;
        v7_refinement_active_=false;
        counters_.overflow_cleanup_seconds+=std::chrono::duration<double>(Clock::now()-started).count();
    }};
    present_=1;history_scale_=0;options_.v7_rrr=true;v5_topology_trial_=true;
    options_.v7_rrr_astar_net_limit=std::max<size_t>(1,options_.overflow_cleanup_group+1);
    const bool checkpointed=options_.v9_incremental_rrr;
    if(checkpointed) begin_rrr_checkpoint();
    bool failed=false;
    try {
        for(int round=0;round<options_.overflow_cleanup_rounds && !timed_out() && !failed;++round) {
            ++counters_.overflow_cleanup_rounds;
            std::vector<std::pair<double,size_t>> queue;
            for(size_t i=0;i<routes_.size();++i) {
                if((i&1023U)==0 && timed_out()) break;
                const double p=official_movable_priority(i);
                if(p>0) queue.push_back({-p,i});
            }
            std::sort(queue.begin(),queue.end());
            size_t single=0,joint=0;
            for(auto [p,i]:queue) {
                (void)p;if(timed_out()) break;
                if(official_movable_priority(i)<=0) continue;  // an earlier move already cleared it
                v7_scope_limit_=1;++counters_.overflow_cleanup_single;
                bool accepted=rrr_round({i});
                if(accepted) {++single;++counters_.overflow_cleanup_single_accepted;}
                if(checkpointed && (!v9_rrr_checkpoint_active_ || !finish_rrr_checkpoint())) {failed=true;break;}
                if(accepted || !options_.overflow_cleanup_blockers || timed_out()) continue;
                const auto blockers=overflow_blockers(i);
                if(blockers.empty()) continue;
                std::vector<size_t> scope{i};scope.insert(scope.end(),blockers.begin(),blockers.end());
                v7_scope_limit_=scope.size();++counters_.overflow_cleanup_joint;counters_.overflow_cleanup_joint_nets+=scope.size();
                if(joint_rrr_round(scope)) {++joint;++counters_.overflow_cleanup_joint_accepted;}
                if(checkpointed && (!v9_rrr_checkpoint_active_ || !finish_rrr_checkpoint())) {failed=true;break;}
            }
            std::cout<<"[OVERFLOW_CLEANUP] round="<<round+1<<" queued="<<queue.size()<<" single_accepted="<<single
                     <<" joint_accepted="<<joint<<" overflow="<<best_metrics_.guide.overflow<<" quality="<<best_metrics_.guide.quality<<'\n';
            if(!single && !joint) break;
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
