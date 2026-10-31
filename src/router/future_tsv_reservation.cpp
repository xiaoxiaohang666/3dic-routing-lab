#include "router/negotiated_router.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace eda {
int NegotiatedRouter::search_cell_demand(int node) const {
    const auto found=future_tsv_demand_.find(node);
    const int64_t total=int64_t(demand_.at(size_t(node)))+
        (found==future_tsv_demand_.end()?0:found->second);
    if(total<0 || total>std::numeric_limits<int>::max()) throw std::logic_error("search reservation demand overflow");
    return int(total);
}
void NegotiatedRouter::update_tsv_reservation(size_t index,int sign) {
    if(future_tsv_by_net_.empty()) return;
    if(index>=future_tsv_by_net_.size() || (sign!=1 && sign!=-1))
        throw std::logic_error("invalid future TSV handoff");
    const auto& contribution=future_tsv_by_net_[index];
    if(contribution.empty()) return; // Both real PIN roots may be exempt.
    if(bool(future_tsv_active_[index])!=(sign<0)) throw std::logic_error("duplicate future TSV handoff");
    long double next_squared=future_tsv_squared_delta_;
    for(auto [node,amount]:contribution) {
        const auto found=future_tsv_demand_.find(node);
        const int old=found==future_tsv_demand_.end()?0:found->second;
        const int64_t next=int64_t(old)+sign*int64_t(amount);
        const int64_t before=int64_t(demand_[node])+old,after=int64_t(demand_[node])+next;
        if(next<0 || before>std::numeric_limits<int>::max() || after>std::numeric_limits<int>::max())
            throw std::logic_error("inconsistent future TSV demand");
        next_squared+=official_resource_cell_contribution(graph_,node,int(after)).squared-
                      official_resource_cell_contribution(graph_,node,int(before)).squared;
    }
    // At most two roots. Prepare all allocations before changing either amount.
    std::array<std::pair<int,bool>,2> inserted{};size_t prepared=0;
    if(contribution.size()>inserted.size()) throw std::logic_error("invalid TSV root count");
    try {
        for(auto [node,amount]:contribution) {
            (void)amount;
            const bool added=future_tsv_demand_.try_emplace(node,0).second;
            inserted[prepared++]={node,added};
        }
    } catch(...) {
        for(size_t i=0;i<prepared;++i) if(inserted[i].second) future_tsv_demand_.erase(inserted[i].first);
        throw;
    }
    for(auto [node,amount]:contribution) {
        auto found=future_tsv_demand_.find(node);found->second+=sign*amount;
        if(!found->second) future_tsv_demand_.erase(found);
    }
    future_tsv_active_[index]=sign>0;
    if(sign>0) {++counters_.future_tsv_remaining;++counters_.future_tsv_restores;}
    else {--counters_.future_tsv_remaining;++counters_.future_tsv_releases;}
    future_tsv_squared_delta_=future_tsv_demand_.empty()?0:next_squared;
    pricing_generation_=std::numeric_limits<uint64_t>::max();
}
void NegotiatedRouter::prepare_tsv_reservations(const std::vector<int>& assignment) {
    if(!options_.future_tsv_reservation) return;
    if(assignment.size()!=def_.nets().size() || !future_tsv_demand_.empty())
        throw std::logic_error("invalid future TSV assignment");
    future_tsv_by_net_.resize(assignment.size());future_tsv_active_.assign(assignment.size(),0);
    for(size_t i=0;i<assignment.size();++i) if(assignment[i]>=0) {
        future_tsv_by_net_[i]=mandatory_tsv_contribution(graph_,policy_.slots().at(size_t(assignment[i])).point,
            options_.official_metrics.tsv_congestion_ratio,true,own_pin_nodes_.at(i),options_.official_metrics.a150_rules);
        if(!future_tsv_by_net_[i].empty()) {update_tsv_reservation(i,1);++counters_.future_tsv_reserved_nets;}
    }
    counters_.future_tsv_reserved_cells=future_tsv_demand_.size();
    counters_.future_tsv_restores=0;
    for(auto [node,amount]:future_tsv_demand_)
        if(int64_t(graph_.official_obstacle_demand(node))+demand_[node]+amount>graph_.official_capacity(node))
            ++counters_.future_tsv_conflict_cells;
}
void NegotiatedRouter::route_initial_net(size_t index,int slot) {
    update_tsv_reservation(index,-1);
    try {
        auto trial=build_route(index,slot);
        account(index,trial,1);
        routes_[index]=std::move(trial);
    } catch(...) {
        // Promises do not enter real demand/owners. A failed search or rejected
        // account restores this net's promise before propagating the failure.
        update_tsv_reservation(index,1);
        throw;
    }
}
} // namespace eda