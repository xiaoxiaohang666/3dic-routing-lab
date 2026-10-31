#pragma once

#include "router/routing_graph.hpp"
#include "objectives/scoring_profile.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace eda {
// One net contributes at most one ordinary Guide demand to a cell. At a TSV
// root that demand shares its reservation; demands of distinct nets are summed.
using NetCellContribution = std::vector<std::pair<int,int>>;

inline bool net_cell_pin_exempt(const std::vector<int>& own_pin_nodes,int node) {
    return std::binary_search(own_pin_nodes.begin(),own_pin_nodes.end(),node);
}

inline NetCellContribution mandatory_tsv_contribution(const RoutingGraph& graph,Point point,
    int tsv_ratio,bool track_demand,const std::vector<int>& own_pin_nodes={},bool a150_rules=false) {
    if(tsv_ratio<0) throw std::invalid_argument("negative TSV resource ratio");
    if(a150_rules && (!track_demand || tsv_ratio!=1))
        throw std::invalid_argument("A150 requires direct physical-track TSV demand with ratio one");
    if(!std::isfinite(point.x)||!std::isfinite(point.y)||!graph.die_area().contains(point))
        throw std::invalid_argument("TSV resource point outside DIEAREA");
    if(track_demand && (!graph.score_20261020() || !graph.documented_rules()))
        throw std::invalid_argument("TSV track demand requires documented October 20 resources");
    const int col=graph.grid().get_col(point.x),row=graph.grid().get_row(point.y);
    if(col<0||row<0||col>=graph.grid().cols||row>=graph.grid().rows)
        throw std::invalid_argument("TSV resource point outside GCell grid");
    NetCellContribution result;
    for(int layer:{10,11}) {
        const int node=graph.node(col,row,layer);
        if(!a150_rules && net_cell_pin_exempt(own_pin_nodes,node)) continue;
        const int64_t tracks=track_demand?int64_t(graph.tsv_landing_tracks(point,layer))*tsv_ratio:0;
        const int64_t reserved=track_demand?(a150_rules?tracks:std::max<int64_t>(1,tracks)):
            int64_t(1)+tsv_ratio;
        if(reserved>std::numeric_limits<int>::max())
            throw std::overflow_error("TSV landing demand exceeds integer range");
        result.emplace_back(node,int(reserved));
    }
    return result;
}

// Ordinary entries are positive. A150 mandatory entries may contain zero
// markers and replace ordinary coverage before sparse positive output is made.
// Legacy mode max-merges positive entries. Never use this to combine owners.
inline NetCellContribution merge_net_cell_contribution(const NetCellContribution& ordinary,
    const NetCellContribution& mandatory,const std::vector<int>& own_pin_nodes={},bool a150_rules=false) {
    const auto by_node=[](const auto& a,const auto& b){return a.first<b.first;};
    if(a150_rules) {
        // Keep zero TSV markers until ordinary coverage has been replaced.
        // Canonical measurements take a linear path without per-cell maps.
        if(std::is_sorted(ordinary.begin(),ordinary.end(),by_node) &&
           std::is_sorted(mandatory.begin(),mandatory.end(),by_node)) {
            NetCellContribution result;result.reserve(ordinary.size()+mandatory.size());
            size_t a=0,b=0;
            while(a<ordinary.size() || b<mandatory.size()) {
                const int node=a==ordinary.size()?mandatory[b].first:
                    (b==mandatory.size()?ordinary[a].first:std::min(ordinary[a].first,mandatory[b].first));
                int ordinary_value=0,reserved_value=0;bool has_reservation=false;
                while(a<ordinary.size() && ordinary[a].first==node) {
                    const int value=ordinary[a++].second;
                    if(node<0 || value<=0) throw std::invalid_argument("invalid ordinary cell contribution");
                    ordinary_value=std::max(ordinary_value,value);
                }
                while(b<mandatory.size() && mandatory[b].first==node) {
                    const int value=mandatory[b++].second;
                    if(node<0 || value<0) throw std::invalid_argument("invalid A150 TSV cell contribution");
                    has_reservation=true;reserved_value=std::max(reserved_value,value);
                }
                const int value=has_reservation?reserved_value:
                    (net_cell_pin_exempt(own_pin_nodes,node)?0:ordinary_value);
                if(value) result.emplace_back(node,value);
            }
            return result;
        }
        std::map<int,int> merged;
        for(const auto [node,value]:ordinary) {
            if(node<0 || value<=0) throw std::invalid_argument("invalid ordinary cell contribution");
            if(!net_cell_pin_exempt(own_pin_nodes,node)) merged[node]=std::max(merged[node],value);
        }
        std::map<int,int> reserved;
        for(const auto [node,value]:mandatory) {
            if(node<0 || value<0) throw std::invalid_argument("invalid A150 TSV cell contribution");
            reserved[node]=std::max(reserved[node],value);
        }
        for(const auto [node,value]:reserved) {
            if(value) merged[node]=value;
            else merged.erase(node);
        }
        return {merged.begin(),merged.end()};
    }
    if(!std::is_sorted(ordinary.begin(),ordinary.end(),by_node)||
       !std::is_sorted(mandatory.begin(),mandatory.end(),by_node)) {
        std::map<int,int> merged;
        for(const auto* contribution:{&ordinary,&mandatory}) for(auto [node,demand]:*contribution) {
            if(node<0||demand<=0) throw std::invalid_argument("invalid per-net cell contribution");
            if(!net_cell_pin_exempt(own_pin_nodes,node)) {
                auto& value=merged[node];value=std::max(value,demand);
            }
        }
        return {merged.begin(),merged.end()};
    }
    // Canonical measurements and two-cell reservations take this allocation-
    // bounded linear path; repeated route evaluation must not create one map
    // allocation for every cell of a long/high-fanout net.
    NetCellContribution result;result.reserve(ordinary.size()+mandatory.size());
    size_t a=0,b=0;
    while(a<ordinary.size()||b<mandatory.size()) {
        const int node=a==ordinary.size()?mandatory[b].first:
            (b==mandatory.size()?ordinary[a].first:std::min(ordinary[a].first,mandatory[b].first));
        int value=0;
        auto consume=[&](const NetCellContribution& part,size_t& position) {
            while(position<part.size()&&part[position].first==node) {
                const int demand=part[position++].second;
                if(node<0||demand<=0)throw std::invalid_argument("invalid per-net cell contribution");
                value=std::max(value,demand);
            }
        };
        consume(ordinary,a);consume(mandatory,b);
        if(!net_cell_pin_exempt(own_pin_nodes,node)) result.emplace_back(node,value);
    }
    return result;
}

struct OfficialResourceCellContribution {
    uint64_t overflow{};
    long double squared{};
    bool zero_capacity{};
};
// The same cell contract is used by full measurement, incremental acceptance,
// reservation quotes and group comparisons. Fixed objects enter exactly once.
inline OfficialResourceCellContribution official_resource_cell_contribution(
    const RoutingGraph& graph,int node,int guide_demand) {
    if(node<0||size_t(node)>=graph.size()||guide_demand<0)
        throw std::invalid_argument("invalid official cell/demand");
    const int64_t total=int64_t(graph.official_obstacle_demand(node))+guide_demand;
    if(total>std::numeric_limits<int>::max())
        throw std::overflow_error("official aggregate demand overflow");
    if(!total) return {};
    const double capacity=graph.official_capacity(node);
    if(!std::isfinite(capacity)||capacity<0||capacity>std::numeric_limits<int>::max())
        throw std::logic_error("invalid official raw capacity");
    const int excess=std::max(0,int(total)-int(capacity));
    if(capacity<=0) return {uint64_t(excess),10000.0L,true};
    return {uint64_t(excess),congestion_cell_penalty(total,capacity,graph.congestion_overflow_scale()),false};
}

struct NetCellReplacementDelta {
    // Sorted signed changes, already summed across all participating owners.
    NetCellContribution demand_delta;
    long double squared_delta{};
    int64_t overflow_delta{};
    int64_t zero_capacity_delta{};
};

// Compare a complete group transaction without changing the caller's ledger.
// Remove ALL old owners before adding new owners. A cell's nonlinear Q change
// is calculated once after cross-net summation, rather than summing independent
// moves against the same stale aggregate. Ownership is committed by the router.
inline NetCellReplacementDelta compare_net_cell_replacement(const RoutingGraph& graph,
    const std::vector<int>& aggregate,const std::vector<NetCellContribution>& old_nets,
    const std::vector<NetCellContribution>& new_nets) {
    if(aggregate.size()!=graph.size()) throw std::invalid_argument("replacement demand dimensions mismatch");
    std::map<int,std::pair<int64_t,int64_t>> changes;
    auto collect=[&](const std::vector<NetCellContribution>& nets,bool add) {
        for(const auto& net:nets) {
            int previous=-1;
            for(auto [node,demand]:net) {
                if(node<=previous||node<0||size_t(node)>=aggregate.size()||demand<=0)
                    throw std::invalid_argument("replacement needs sorted unique per-net contributions");
                previous=node;
                auto& value=changes[node];
                (add?value.second:value.first)+=demand;
                if(value.first>std::numeric_limits<int>::max()||value.second>std::numeric_limits<int>::max())
                    throw std::overflow_error("replacement group demand overflow");
            }
        }
    };
    collect(old_nets,false);collect(new_nets,true);
    NetCellReplacementDelta result;
    for(const auto& [node,values]:changes) {
        const int64_t before=aggregate[node],residual=before-values.first;
        const int64_t after=residual+values.second;
        if(before<0||residual<0||after>std::numeric_limits<int>::max())
            throw std::logic_error("replacement group is inconsistent with aggregate ledger");
        const int delta=int(after-before);
        if(!delta) continue;
        result.demand_delta.emplace_back(node,delta);
        const auto old=official_resource_cell_contribution(graph,node,int(before));
        const auto next=official_resource_cell_contribution(graph,node,int(after));
        result.squared_delta+=next.squared-old.squared;
        result.overflow_delta+=int64_t(next.overflow)-int64_t(old.overflow);
        result.zero_capacity_delta+=int64_t(next.zero_capacity)-int64_t(old.zero_capacity);
    }
    return result;
}
} // namespace eda
