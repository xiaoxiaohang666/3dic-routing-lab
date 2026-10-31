#include "router/negotiated_router.hpp"
#include <algorithm>
#include <map>
#include <stdexcept>

namespace eda {
namespace {
bool same_tree_move_access(const PinAccess& a,const PinAccess& b) {
    if(a.node!=b.node || a.vias!=b.vias || a.length!=b.length || a.guides.size()!=b.guides.size()) return false;
    for(size_t k=0;k<a.guides.size();++k) {
        const auto& x=a.guides[k];const auto& y=b.guides[k];
        if(x.layer!=y.layer || x.x1!=y.x1 || x.y1!=y.y1 || x.x2!=y.x2 || x.y2!=y.y2) return false;
    }
    return true;
}
}

// The caller has removed the old net from the live resource ledger. This
// helper only builds a value: it never accounts a route or changes TSV owners.
std::optional<NetRoute> NegotiatedRouter::build_tree_tsv_move(size_t index,int slot,const NetRoute& old) {
    if(timed_out() || !options_.official_model || !graph_.documented_rules() ||
       !options_.official_metrics.a150_rules || !options_.official_metrics.tsv_track_demand ||
       options_.tsv_policy.resource_mode!=TsvResourceMode::GLOBAL_GRID ||
       old.explicit_geometry || old.slot<0 || slot<0 || old.slot==slot ||
       index>=def_.nets().size() || size_t(old.slot)>=policy_.slots().size() ||
       size_t(slot)>=policy_.slots().size() || old.net_index!=index ||
       old.nodes.empty() || old.nodes.size()>8192 || old.edges.size()>8192 ||
       options_.tsv_tree_move_seconds<=0 || !options_.tsv_tree_move_expansions) return std::nullopt;
    const auto& net=def_.nets()[index];
    if(!net.is_cross_die || !policy_.compatible(net,slot)) return std::nullopt;
    const auto& previous=policy_.slots()[size_t(old.slot)];
    const auto& target=policy_.slots()[size_t(slot)];
    // Complete all-layer geometry is necessary here even if another profile
    // allows interface-only sites: it proves the canonical old TSV accesses
    // cannot also be a real PIN access. No representative-point inference.
    for(const auto* landing:{&previous,&target})
        if(!graph_.tsv_landing_legal(graph_.col(landing->lower_node),graph_.row(landing->lower_node),
                                   TsvObstacleScope::ALL_LAYERS)) return std::nullopt;
    // Free-slot proposals are checked again against owners of the two landing
    // cells. Avoid a whole-design net scan for each bounded local candidate.
    for(int node:{graph_.node(graph_.col(target.lower_node),graph_.row(target.lower_node),9),target.lower_node,target.upper_node})
        for(int other:cell_owners(node))
            if(size_t(other)!=index && routes_[size_t(other)].slot==slot) return std::nullopt;

    // A zero-track TSV may have no landing-cell owners in the billing ledger.
    // Its occupancy is still physical; retain a rare conservative full check.
    if(graph_.tsv_landing_tracks(target.point,10)==0 && graph_.tsv_landing_tracks(target.point,11)==0)
        for(size_t other=0;other<routes_.size();++other)
            if(other!=index && routes_[other].slot==slot) return std::nullopt;

    using Clock=std::chrono::steady_clock;
    const auto saved_deadline=deadline_,saved_refinement_deadline=v7_refinement_deadline_;
    const auto saved_limit=options_.search_limit,saved_refinement_limit=options_.v7_rrr_astar_expansions;
    const auto saved_expansion_start=v7_refinement_expansion_start_;
    const bool saved_trial=v5_topology_trial_,saved_refinement=v7_refinement_active_;
    struct Restore {std::function<void()> action;~Restore(){action();}} restore{[&] {
        deadline_=saved_deadline;options_.search_limit=saved_limit;
        options_.v7_rrr_astar_expansions=saved_refinement_limit;
        v7_refinement_deadline_=saved_refinement_deadline;v7_refinement_expansion_start_=saved_expansion_start;
        v5_topology_trial_=saved_trial;v7_refinement_active_=saved_refinement;
    }};
    deadline_=std::min(saved_deadline,Clock::now()+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.tsv_tree_move_seconds)));
    v5_topology_trial_=true;v7_refinement_active_=true;v7_refinement_deadline_=deadline_;
    v7_refinement_expansion_start_=counters_.expansions;
    options_.v7_rrr_astar_expansions=std::min(saved_refinement_limit,options_.tsv_tree_move_expansions);
    const auto expansion_start=counters_.expansions;
    const size_t expansion_limit=std::min(saved_limit,options_.tsv_tree_move_expansions);
    if(!expansion_limit) return std::nullopt;
    try {
        std::array<PinAccess,2> old_access,new_access;
        std::array<size_t,2> old_position{old.accesses.size(),old.accesses.size()};
        for(int side=0;side<2;++side) {
            const int original=side?11:10,low=side?11:1,high=side?20:9;
            const auto before=accesses(previous.point,original,low,high);
            const auto after=accesses(target.point,original,low,high);
            if(before.size()!=1 || after.size()!=1) return std::nullopt;
            old_access[size_t(side)]=before.front();new_access[size_t(side)]=after.front();
            size_t matches=0;
            for(size_t k=0;k<old.accesses.size();++k)
                if(same_tree_move_access(old.accesses[k],before.front())) {old_position[size_t(side)]=k;++matches;}
            // Ambiguous/unrecorded synthetic accesses are not reconstructed by
            // guessing. Let the existing whole-net candidate handle this net.
            if(matches!=1 || !old.nodes.count(before.front().node)) return std::nullopt;
        }
        if(old_position[0]==old_position[1]) return std::nullopt;
        NetRoute trial=old;trial.slot=slot;trial.guides.clear();trial.accesses.clear();
        trial.recovery_subtree=false;trial.recovery_layer=false;trial.measured=NetGuideMetrics{};
        std::set<int> protected_nodes;
        std::array<size_t,2> real_access_count{};
        for(size_t k=0;k<old.accesses.size();++k) if(k!=old_position[0] && k!=old_position[1]) {
            const auto& access=old.accesses[k];
            if(!old.nodes.count(access.node)) return std::nullopt;
            trial.accesses.push_back(access);protected_nodes.insert(access.node);
            ++real_access_count[size_t(graph_.layer(access.node)>10)];
        }
        if(!real_access_count[0] || !real_access_count[1]) return std::nullopt;
        std::map<int,std::set<int>> adjacency;
        for(int node:old.nodes) adjacency[node];
        size_t work=0;
        for(auto edge:old.edges) {
            if((++work&127U)==0 && timed_out()) return std::nullopt;
            const auto [a,b]=edge_nodes(edge);
            if(!old.nodes.count(a) || !old.nodes.count(b) || !graph_.legal_edge(a,b) ||
               (graph_.layer(a)>10)!=(graph_.layer(b)>10)) return std::nullopt;
            adjacency[a].insert(b);adjacency[b].insert(a);
        }
        // Branches and all existing ordinary via columns belong to the saved
        // tree. Only a planar TSV-only leaf stem is removed in this first
        // bounded operator; a via-bearing stem is conservatively retained.
        for(const auto& [node,neighbors]:adjacency) {
            if(neighbors.size()>=3) protected_nodes.insert(node);
            for(int next:neighbors) if(graph_.layer(next)!=graph_.layer(node)) {
                protected_nodes.insert(node);protected_nodes.insert(next);
            }
        }
        for(int side=0;side<2;++side) {
            int node=old_access[size_t(side)].node;
            while(!protected_nodes.count(node)) {
                if(timed_out()) return std::nullopt;
                auto found=adjacency.find(node);
                if(found==adjacency.end() || found->second.size()!=1) break;
                const int next=*found->second.begin();
                trial.edges.erase(route_edge(node,next));trial.nodes.erase(node);
                adjacency[next].erase(node);adjacency.erase(found);node=next;
            }
        }
        for(int side=0;side<2;++side) {
            if(timed_out()) return std::nullopt;
            const int low=side?11:1,high=side?20:9;
            std::set<int> targets;
            for(int node:trial.nodes) if(graph_.layer(node)>=low && graph_.layer(node)<=high) targets.insert(node);
            if(targets.empty()) return std::nullopt;
            // A single attachment is valid only if the retained graph on this
            // die is connected. Explicit/disconnected access geometry falls
            // back rather than silently leaving a component behind.
            std::set<int> reached{*targets.begin()};std::vector<int> pending{*targets.begin()};
            while(!pending.empty()) {
                if((++work&127U)==0 && timed_out()) return std::nullopt;
                const int node=pending.back();pending.pop_back();
                for(int next:adjacency.at(node)) if(targets.count(next) && reached.insert(next).second) pending.push_back(next);
            }
            if(reached!=targets) return std::nullopt;
            const size_t used=counters_.expansions-expansion_start;
            if(used>=expansion_limit) return std::nullopt;
            options_.search_limit=expansion_limit-used;
            const std::vector<PinAccess> sources{new_access[size_t(side)]};
            const auto path=search(sources,targets,trial,low,high,options_.dijkstra);
            if(path.status!=PathResult::Status::FOUND || path.source!=0 || path.nodes.empty() ||
               path.nodes.front()!=sources.front().node || !targets.count(path.nodes.back()) ||
               timed_out() || counters_.expansions-expansion_start>expansion_limit) return std::nullopt;
            trial.accesses.push_back(sources.front());
            // Stop at the first retained node: no second attachment or cycle
            // can be introduced by a template that traverses existing branches.
            bool attached=false;
            for(size_t k=0;k<path.nodes.size();++k) {
                const int node=path.nodes[k];trial.nodes.insert(node);
                if(k) {
                    if(!graph_.legal_edge(path.nodes[k-1],node)) return std::nullopt;
                    trial.edges.insert(route_edge(path.nodes[k-1],node));
                }
                if(targets.count(node)) {attached=true;break;}
            }
            if(!attached) return std::nullopt;
        }
        if(timed_out()) return std::nullopt;
        encode(index,trial);
        std::string error;
        if(trial.measured.tsv_cells.size()!=1 || trial.measured.tsv_points.size()!=1 ||
           trial.measured.tsv_points.front().distance_to(target.point)!=0 ||
           !policy_.validate_mapping(net,trial.guides,slot,error) || !validate_route(index,trial.guides,error) ||
           timed_out()) return std::nullopt;
        return trial;
    } catch(const std::runtime_error&) {
        // Search/encoding failures reject this optional value. Invariant
        // failures (logic_error and its subclasses) deliberately propagate.
        return std::nullopt;
    }
}
} // namespace eda
