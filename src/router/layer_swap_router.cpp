#include "router/negotiated_router.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <set>

namespace eda {
// Layer swap (opt-in, --layer-swap 1). A same-die net whose route climbs many
// layers usually does so because the low layers on its short path are full.
// Every climb costs vias and intermediate-layer cells. The stage takes one
// climbing net, finds the few nets that fill the cells of its cheapest
// congestion-free path, rips them up together, lays the climbing net on that
// path first and then reroutes the others with the normal congestion costs.
// TSV positions never change, so T is invariant. The move is kept only if the
// complete W/C score improves without adding total wirelength; otherwise every
// net is restored exactly. Nothing depends on case names or coordinates.
namespace {
int demand_at(const std::vector<std::pair<int,int>>& demand,int node) {
    const auto p=std::lower_bound(demand.begin(),demand.end(),node,[](const auto& e,int id){return e.first<id;});
    return p!=demand.end() && p->first==node?p->second:0;
}
}

std::vector<size_t> NegotiatedRouter::layer_swap_targets() const {
    const double via_cell=options_.official_metrics.via_cost_ratio*
        std::max(options_.official_metrics.gcell_width_dbu,options_.official_metrics.gcell_height_dbu);
    const bool rotate=swap_tried_.size()==routes_.size();
    std::vector<std::pair<double,size_t>> ranked;
    for(size_t i=0;i<routes_.size();++i) {
        const auto& r=routes_[i];const auto& net=def_.nets()[i];
        if(r.slot>=0 || net.is_cross_die || r.explicit_geometry || net.pins.size()<2 || (rotate && swap_tried_[i])) continue;
        // A bend per extra pin plus pin access is ordinary; anything well
        // beyond that is a climb through layers the planar path did not need.
        const double ordinary=2*via_cell*double(net.pins.size()-1);
        const double climb=r.measured.via_wirelength-ordinary;
        if(climb>=4*via_cell) ranked.push_back({climb,i});
    }
    std::stable_sort(ranked.begin(),ranked.end(),[&](const auto& a,const auto& b) {
        return a.first!=b.first?a.first>b.first:def_.nets()[a.second].name<def_.nets()[b.second].name;
    });
    std::vector<size_t> result;result.reserve(ranked.size());
    for(const auto& entry:ranked) result.push_back(entry.second);
    return result;
}

bool NegotiatedRouter::layer_swap_transaction(size_t target,std::vector<std::pair<size_t,NetRoute>>& journal,size_t& pending) {
    ++swap_stats_.attempted;
    const auto before=recovery_metrics(best_metrics_);
    struct Entry {size_t index;NetRoute old;bool installed;};
    std::vector<Entry> ripped;
    bool done=false;
    auto restore=[&] {
        if(done) return;
        done=true;
        for(auto it=ripped.rbegin();it!=ripped.rend();++it) if(it->installed) {account(it->index,routes_[it->index],-1);it->installed=false;}
        for(auto it=ripped.rbegin();it!=ripped.rend();++it) {routes_[it->index]=it->old;account(it->index,it->old,1);}
        best_metrics_=before;
    };
    const double saved_present=present_;
    struct RestorePresent {double& value;double saved;~RestorePresent(){value=saved;}} keep_present{present_,saved_present};
    try {
        ripped.push_back({target,routes_.at(target),false});
        account(target,ripped.back().old,-1);routes_[target]=NetRoute{};
        // Cheapest path for the target as if congestion were free.
        present_=0;
        NetRoute free=build_candidate(target,ripped.front().old.slot,0);
        present_=saved_present;
        if(free.measured.wirelength>=ripped.front().old.measured.wirelength) {++swap_stats_.no_gain;restore();return false;}
        // Cells on that path that the target would overflow, and by how much.
        std::map<int,int> needed;
        for(auto [node,amount]:free.measured.demand) {
            if(amount<=0) continue;
            const double cap=graph_.official_capacity(node);
            const double fixed=graph_.official_obstacle_demand(node);
            const double used=fixed+demand_[size_t(node)];
            if(used+amount<=cap) continue;
            if(cap<=0 || fixed+amount>cap) {++swap_stats_.no_blocker;restore();return false;}
            needed[node]=int(std::ceil(used+amount-cap));
        }
        if(needed.empty()) {++swap_stats_.no_blocker;restore();return false;}
        // Each owner's removable share of the needed cells (protected pin and
        // TSV resources stay with their owner whatever the reroute does).
        std::set<size_t> candidates;
        for(const auto& entry:needed) for(int owner:cell_owners(entry.first)) {
            const size_t o=size_t(owner);
            if(o!=target && !routes_[o].explicit_geometry) candidates.insert(o);
        }
        std::map<size_t,std::vector<std::pair<int,int>>> removable;
        for(size_t o:candidates) {
            const auto protected_cells=official_protected_demand(o,routes_[o]);
            for(const auto& entry:needed) {
                const int amount=demand_at(routes_[o].measured.demand,entry.first)-demand_at(protected_cells,entry.first);
                if(amount>0) removable[o].push_back({entry.first,amount});
            }
        }
        // Owners clearing more of the needed cells first, then longer routes,
        // which have more room to find another way round.
        std::vector<size_t> owners;owners.reserve(removable.size());
        for(const auto& entry:removable) owners.push_back(entry.first);
        std::stable_sort(owners.begin(),owners.end(),[&](size_t a,size_t b) {
            if(removable[a].size()!=removable[b].size()) return removable[a].size()>removable[b].size();
            const double la=routes_[a].measured.wirelength,lb=routes_[b].measured.wirelength;
            return la!=lb?la>lb:a<b;
        });
        std::vector<size_t> blockers;
        auto remaining=needed;
        for(size_t o:owners) {
            if(blockers.size()>=options_.layer_swap_blockers) break;
            bool useful=false;
            for(auto [node,amount]:removable[o]) {(void)amount;if(remaining[node]>0) useful=true;}
            if(!useful) continue;
            blockers.push_back(o);
            for(auto [node,amount]:removable[o]) remaining[node]-=amount;
        }
        for(const auto& entry:remaining) if(entry.second>0) {++swap_stats_.no_blocker;restore();return false;}
        // Rip every blocker before anything is rerouted.
        for(size_t b:blockers) {
            ripped.push_back({b,routes_[b],false});
            account(b,ripped.back().old,-1);routes_[b]=NetRoute{};
        }
        auto install=[&](Entry& e,NetRoute trial) {
            std::string error;
            if(trial.slot!=e.old.slot || !policy_.validate_mapping(def_.nets()[e.index],trial.guides,trial.slot,error))
                throw std::runtime_error("layer swap changed TSV mapping: "+error);
            if(!validate_route(e.index,trial.guides,error)) throw std::runtime_error("layer swap invalid net: "+error);
            trial.measured=measure_official_net_guides(def_.nets()[e.index],trial.guides,graph_,lef_,
                options_.official_metrics,reference_lengths_[e.index]);
            if(trial.measured.tsv_cells!=e.old.measured.tsv_cells || trial.measured.tsv_points.size()!=e.old.measured.tsv_points.size())
                throw std::runtime_error("layer swap changed selected TSV");
            for(size_t k=0;k<trial.measured.tsv_points.size();++k)
                if(trial.measured.tsv_points[k].distance_to(e.old.measured.tsv_points[k])!=0)
                    throw std::runtime_error("layer swap changed selected TSV point");
            for(int n:trial.nodes) if(!graph_.legal_node(n)) throw std::runtime_error("layer swap illegal node");
            for(auto edge:trial.edges) {auto [a,b]=edge_nodes(edge);
                if(!trial.nodes.count(a)||!trial.nodes.count(b)||!graph_.legal_edge(a,b))
                    throw std::runtime_error("layer swap illegal edge");
            }
            trial.recovery_subtree=trial.recovery_layer=false;
            routes_[e.index]=std::move(trial);account(e.index,routes_[e.index],1);e.installed=true;
        };
        install(ripped.front(),std::move(free));
        std::vector<size_t> order(ripped.size()-1);
        for(size_t k=0;k<order.size();++k) order[k]=k+1;
        std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b) {
            return ripped[a].old.measured.wirelength>ripped[b].old.measured.wirelength;
        });
        for(size_t k:order) {
            if(timed_out()) {restore();return false;}
            install(ripped[k],build_route(ripped[k].index,ripped[k].old.slot));
        }
        auto after=recovery_metrics(before);
        // Total wirelength may not grow: the batch checkpoint compares against
        // the last verified state and would otherwise undo the whole batch.
        if((options_.unified_budget&&timed_out()) || after.official.zero_capacity_cells>before.official.zero_capacity_cells ||
           after.guide.wirelength>before.guide.wirelength || after.guide.quality<=before.guide.quality+1e-12) {
            ++swap_stats_.rejected;restore();return false;
        }
        for(const auto& e:ripped) {
            after.physical_length+=routes_[e.index].length-e.old.length;
            after.vias=after.vias-e.old.vias+routes_[e.index].vias;
        }
        swap_stats_.delta_length+=after.guide.wirelength-before.guide.wirelength;
        best_metrics_=after;++swap_stats_.accepted;++pending;swap_stats_.blockers_moved+=blockers.size();
        for(auto& e:ripped) journal.emplace_back(e.index,std::move(e.old));
        done=true;
        return true;
    } catch(const std::exception& e) {
        ++swap_stats_.failures;++rejection_reasons_[std::string("layer swap: ")+e.what()];
        present_=saved_present;restore();return false;
    }
}

void NegotiatedRouter::recover_layer_swaps() {
    if(!options_.official_model) throw std::runtime_error("layer swap requires official model");
    if(options_.layer_swap_blockers<1 || options_.layer_swap_blockers>16) throw std::runtime_error("invalid layer swap blocker limit");
    const auto started=std::chrono::steady_clock::now();
    const double previous_history=history_scale_;const bool previous_cell=recovery_cell_cost_;
    struct RestorePhase {std::function<void()> action;~RestorePhase(){action();}} restore{[&] {
        history_scale_=previous_history;recovery_cell_cost_=previous_cell;
    }};
    history_scale_=0;recovery_cell_cost_=true;
    validate_complete();best_metrics_=measure();
    CompleteMetrics checkpoint=best_metrics_;std::vector<std::pair<size_t,NetRoute>> journal;size_t pending=0;
    // The checkpoint is shared with budgeted recovery; keep its counters
    // separate so each stage reports only its own work.
    auto verify=[&] {
        const auto saved=recovery_stats_;
        const bool ok=recovery_checkpoint(journal,checkpoint);
        const size_t rollbacks=recovery_stats_.checkpoint_rollbacks-saved.checkpoint_rollbacks;
        recovery_stats_=saved;
        if(ok) ++swap_stats_.checkpoints;
        else {swap_stats_.checkpoint_rollbacks+=std::max<size_t>(1,rollbacks);swap_stats_.accepted-=std::min(swap_stats_.accepted,pending);}
        pending=0;return ok;
    };
    if(swap_tried_.size()!=routes_.size()) swap_tried_.assign(routes_.size(),0);
    auto targets=layer_swap_targets();
    if(targets.empty()) {
        std::fill(swap_tried_.begin(),swap_tried_.end(),uint8_t{0});++swap_stats_.sweeps;
        targets=layer_swap_targets();
    }
    bool valid=true;
    for(size_t index:targets) {
        if(timed_out()) break;
        swap_tried_[index]=1;
        if(layer_swap_transaction(index,journal,pending) && journal.size()>=64 && !(valid=verify())) break;
    }
    if(valid && !journal.empty()) verify();
    swap_stats_.seconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    std::cout<<"[LAYER_SWAP] targets="<<targets.size()<<" attempted="<<swap_stats_.attempted<<" accepted="<<swap_stats_.accepted
             <<" blockers_moved="<<swap_stats_.blockers_moved<<" no_blocker="<<swap_stats_.no_blocker
             <<" no_gain="<<swap_stats_.no_gain<<" rejected="<<swap_stats_.rejected<<" failures="<<swap_stats_.failures
             <<" delta_L="<<swap_stats_.delta_length<<" S="<<100*best_metrics_.guide.quality<<std::endl;
}
} // namespace eda
