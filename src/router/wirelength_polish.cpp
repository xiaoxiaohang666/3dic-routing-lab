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
void NegotiatedRouter::run_wirelength_polish() {
    if(!options_.official_model || options_.wirelength_polish_rounds<=0 || timed_out()) return;
    using Clock=std::chrono::steady_clock;
    const auto started=Clock::now();
    const auto saved_present=present_;const auto saved_history=history_scale_;
    const bool saved_incremental=options_.v7_rrr,saved_topology=v5_topology_trial_;
    const size_t saved_maze_nets=options_.v7_rrr_astar_net_limit,saved_scope=v7_scope_limit_;
    struct Restore {std::function<void()> f;~Restore(){f();}} restore{[&] {
        present_=saved_present;history_scale_=saved_history;options_.v7_rrr=saved_incremental;
        v5_topology_trial_=saved_topology;options_.v7_rrr_astar_net_limit=saved_maze_nets;v7_scope_limit_=saved_scope;
        v7_refinement_active_=false;force_rrr_maze_=false;
        counters_.wirelength_polish_seconds+=std::chrono::duration<double>(Clock::now()-started).count();
    }};
    // Plain present cost and no history, as in the overflow cleanup: acceptance
    // is the exact score, so the search should price only real C losses.
    present_=1;history_scale_=0;options_.v7_rrr=true;v5_topology_trial_=true;
    options_.v7_rrr_astar_net_limit=std::max<size_t>(1,options_.v7_rrr_astar_net_limit);
    force_rrr_maze_=true;
    const bool checkpointed=options_.v9_incremental_rrr;
    if(checkpointed) begin_rrr_checkpoint();
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
                if(checkpointed && (!v9_rrr_checkpoint_active_ || !finish_rrr_checkpoint())) {failed=true;break;}
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
