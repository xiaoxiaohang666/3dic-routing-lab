#include "router/negotiated_router.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

namespace eda {
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
            size_t accepted=0,attempted=0;double gain=0;
            for(auto [key,i]:queue) {
                (void)key;if(timed_out()) break;
                const double old_length=routes_[i].measured.wirelength;
                v7_scope_limit_=1;++attempted;++counters_.wirelength_polish_attempted;
                if(rrr_round({i})) {
                    ++accepted;++counters_.wirelength_polish_accepted;
                    gain+=old_length-routes_[i].measured.wirelength;
                }
                if(checkpointed) {
                    const auto audits=counters_.v9_full_checkpoints;const auto audit_started=Clock::now();
                    if(!v9_rrr_checkpoint_active_ || !finish_rrr_checkpoint()) {failed=true;break;}
                    if(counters_.v9_full_checkpoints!=audits) space_audits(std::chrono::duration<double>(Clock::now()-audit_started).count());
                }
            }
            counters_.wirelength_polish_gain_dbu+=gain;
            std::cout<<"[WIRELENGTH_POLISH] round="<<round+1<<" queued="<<queue.size()<<" attempted="<<attempted
                     <<" accepted="<<accepted<<" gain_dbu="<<gain<<" normalized="<<best_metrics_.guide.normalized
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
