#include "router/negotiated_router.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>

namespace eda {
std::vector<NetRoute> NegotiatedRouter::low_layer_candidates(size_t index,const NetRoute& old) {
    std::vector<NetRoute> result;
    if(!options_.low_layer_mask || timed_out()) return result;
    if(old.explicit_geometry) {++counters_.low_layer_explicit_skips;return result;}
    if(old.edges.empty() || old.edges.size()>4096) return result;
    std::map<int,std::vector<int>> adjacency;
    for(auto edge:old.edges) {
        const auto [a,b]=edge_nodes(edge);
        adjacency[a].push_back(b);adjacency[b].push_back(a);
    }
    std::set<int> protected_nodes;
    for(const auto& access:old.accesses) protected_nodes.insert(access.node);
    if(old.slot>=0) {
        const auto& slot=policy_.slots().at(size_t(old.slot));
        protected_nodes.insert(slot.lower_node);protected_nodes.insert(slot.upper_node);
        protected_nodes.insert(graph_.node(graph_.col(slot.lower_node),graph_.row(slot.lower_node),9));
    }
    struct Chain {std::vector<int> nodes;double priority{};int side{};};
    std::vector<Chain> chains;
    for(int side=0;side<2;++side) {
        if(!(options_.low_layer_mask&(1<<side))) continue;
        // The graph's two dies are layers 1..10 and 11..20. No case-name rules.
        const int layer=side?11:1;
        const auto planar=[&](int a,int b){return graph_.layer(a)==layer && graph_.layer(b)==layer;};
        std::set<int> anchors;
        for(const auto& [node,neighbors]:adjacency) if(graph_.layer(node)==layer) {
            bool anchor=protected_nodes.count(node)||neighbors.size()!=2;
            for(int next:neighbors) if(!planar(node,next)) anchor=true;
            if(anchor) {anchors.insert(node);++counters_.low_layer_protected_nodes;}
        }
        std::set<EdgeKey> consumed;
        for(int start:anchors) for(int next:adjacency.at(start)) {
            if(timed_out()) return result;
            if(!planar(start,next)||consumed.count(route_edge(start,next))) continue;
            Chain chain;chain.side=side;chain.nodes.push_back(start);
            int previous=start,current=next;
            bool valid=true;
            while(true) {
                if(!consumed.insert(route_edge(previous,current)).second) {valid=false;break;}
                ++counters_.low_layer_planar_edges;chain.nodes.push_back(current);
                if(anchors.count(current)) break;
                const auto& adjacent=adjacency.at(current);
                if(adjacent.size()!=2) {valid=false;break;}
                const int following=adjacent[0]==previous?adjacent[1]:adjacent[0];
                if(!planar(current,following)) {valid=false;break;}
                previous=current;current=following;
            }
            if(!valid) continue;
            if(chain.nodes.size()<3) {++counters_.low_layer_short_chains;continue;}
            // Only interior cells can disappear: endpoints remain for genuine
            // PIN/access/branch connectivity, including every adjacent-layer via.
            for(size_t k=1;k+1<chain.nodes.size();++k) {
                const int node=chain.nodes[k];
                const auto it=std::lower_bound(old.measured.demand.begin(),old.measured.demand.end(),node,
                    [](const auto& entry,int n){return entry.first<n;});
                if(it!=old.measured.demand.end()&&it->first==node)
                    chain.priority+=official_reroute_priority(node,it->second);
            }
            ++counters_.low_layer_chains;
            if(chain.priority>0) chains.push_back(std::move(chain));
        }
    }
    std::stable_sort(chains.begin(),chains.end(),[](const auto& a,const auto& b) {
        return a.priority!=b.priority?a.priority>b.priority:a.nodes<b.nodes;
    });
    if(chains.size()>4) chains.resize(4);
    for(const auto& chain:chains) {
        if(timed_out()) break;
        const int low=chain.side?11:1,high=chain.side?20:(old.slot>=0?9:10);
        const auto direction=lef_.get_layer_direction(get_layer_name(low));
        bool has_target=false;
        for(int layer=low+1;layer<=high && !timed_out();++layer) {
            if(lef_.get_layer_direction(get_layer_name(layer))!=direction) continue;
            has_target=true;
            auto trial=old;trial.guides.clear();trial.recovery_subtree=false;trial.recovery_layer=false;
            for(size_t k=1;k<chain.nodes.size();++k)
                trial.edges.erase(route_edge(chain.nodes[k-1],chain.nodes[k]));
            for(size_t k=1;k+1<chain.nodes.size();++k) trial.nodes.erase(chain.nodes[k]);
            bool legal=true;
            const auto add=[&](int a,int b) {
                if(!graph_.legal_edge(a,b)) {legal=false;return;}
                trial.nodes.insert(a);trial.nodes.insert(b);trial.edges.insert(route_edge(a,b));
            };
            int previous=-1;
            for(int node:chain.nodes) {
                const int lifted=graph_.node(graph_.col(node),graph_.row(node),layer);
                if(previous>=0) add(previous,lifted);
                previous=lifted;
            }
            for(int endpoint:{chain.nodes.front(),chain.nodes.back()})
                for(int l=low;l<layer;++l)
                    add(graph_.node(graph_.col(endpoint),graph_.row(endpoint),l),
                        graph_.node(graph_.col(endpoint),graph_.row(endpoint),l+1));
            if(!legal) {++counters_.low_layer_illegal;continue;}
            try {
                encode(index,trial);
                std::string error;
                if(!validate_route(index,trial.guides,error) ||
                   !policy_.validate_mapping(def_.nets()[index],trial.guides,trial.slot,error) ||
                   trial.measured.tsv_cells!=old.measured.tsv_cells ||
                   trial.measured.tsv_points.size()!=old.measured.tsv_points.size()) {
                    ++counters_.low_layer_illegal;continue;
                }
                bool same=true;
                for(size_t k=0;k<old.measured.tsv_points.size();++k)
                    same=same&&old.measured.tsv_points[k].distance_to(trial.measured.tsv_points[k])==0;
                if(!same) {++counters_.low_layer_illegal;continue;}
                ++counters_.low_layer_candidates;++counters_.low_layer_candidates_by_die[size_t(chain.side)];
                result.push_back(std::move(trial));
            } catch(const std::exception& error) {
                ++counters_.low_layer_illegal;
                ++rejection_reasons_[std::string("low-layer encode: ")+error.what()];
            }
        }
        if(!has_target) ++counters_.low_layer_no_target;
    }
    return result;
}

bool NegotiatedRouter::low_layer_transaction(size_t index) {
    if(!v9_rrr_checkpoint_active_) throw std::logic_error("low-layer transaction needs complete checkpoint");
    ++counters_.low_layer_attempted;
    const auto old=routes_.at(index);const auto before=recovery_metrics(best_metrics_);
    // Enumeration reads the complete live ledger to rank actual congested cells.
    auto choices=low_layer_candidates(index,old);
    if(choices.empty()||timed_out()) return false;
    journal_rrr_scope({index});
    // Any failure after account begins is handled by the outer full-journal
    // rebuild, not an inverse delta against potentially partially written state.
    account(index,old,-1);routes_[index]=NetRoute{};
    const auto base=official_candidate_base();
    const auto old_key=official_candidate_key(old,base);
    auto best=choices.end();auto key=old_key;
    for(auto it=choices.begin();it!=choices.end();++it) {
        const auto candidate_key=official_candidate_key(*it,base);
        if(candidate_key<key) {best=it;key=candidate_key;}
    }
    if(best==choices.end()||timed_out()) {
        routes_[index]=old;account(index,old,1);best_metrics_=before;
        ++counters_.low_layer_score_rejected;return false;
    }
    routes_[index]=std::move(*best);account(index,routes_[index],1);
    auto after=recovery_metrics(before);
    if(after.official.zero_capacity_cells>before.official.zero_capacity_cells ||
       after.guide.quality<=before.guide.quality+1e-12 || timed_out()) {
        account(index,routes_[index],-1);routes_[index]=old;account(index,old,1);best_metrics_=before;
        ++counters_.low_layer_score_rejected;return false;
    }
    validate_rrr_group({{index,old}});
    if(timed_out()) {
        account(index,routes_[index],-1);routes_[index]=old;account(index,old,1);best_metrics_=before;
        ++counters_.low_layer_score_rejected;return false;
    }
    after.physical_length+=routes_[index].length-old.length;
    after.vias=after.vias-old.vias+routes_[index].vias;
    for(int side=0;side<2;++side) {
        const int low=side?11:1;
        bool changed=false;
        for(int node:old.nodes) if(graph_.layer(node)==low&&!routes_[index].nodes.count(node)) {changed=true;break;}
        if(changed) ++counters_.low_layer_accepted_by_die[size_t(side)];
    }
    ++counters_.low_layer_accepted;++v9_rrr_pending_groups_;
    counters_.low_layer_delta_q+=after.official.squared_normalized_overflow-before.official.squared_normalized_overflow;
    counters_.low_layer_delta_length+=after.guide.wirelength-before.guide.wirelength;
    counters_.low_layer_delta_score+=100*(after.guide.quality-before.guide.quality);
    best_metrics_=after;return true;
}

void NegotiatedRouter::run_low_layer_repair() {
    if(!options_.low_layer_mask || options_.low_layer_seconds<=0 || !options_.low_layer_max_attempts || timed_out()) return;
    if(!options_.official_model || !graph_.documented_rules() || !options_.official_metrics.score_20261020)
        throw std::invalid_argument("low-layer repair requires the documented physical resource model");
    using Clock=std::chrono::steady_clock;
    const auto started=Clock::now(),outer_deadline=deadline_;
    deadline_=std::min(outer_deadline,started+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.low_layer_seconds)));
    struct Restore {std::function<void()> action;~Restore(){action();}} restore{[&] {
        deadline_=outer_deadline;counters_.low_layer_seconds+=std::chrono::duration<double>(Clock::now()-started).count();
    }};
    struct Accepted {size_t count;std::array<size_t,2> sides;double q,length,score;};
    const auto snapshot=[&]{return Accepted{counters_.low_layer_accepted,counters_.low_layer_accepted_by_die,
        counters_.low_layer_delta_q,counters_.low_layer_delta_length,counters_.low_layer_delta_score};};
    auto verified=snapshot();
    const auto restore_counts=[&] {
        counters_.low_layer_accepted=verified.count;counters_.low_layer_accepted_by_die=verified.sides;
        counters_.low_layer_delta_q=verified.q;counters_.low_layer_delta_length=verified.length;
        counters_.low_layer_delta_score=verified.score;
    };
    begin_rrr_checkpoint();
    try {
        struct Entry {size_t net;double priority;};std::vector<Entry> queue;
        for(size_t index=0;index<routes_.size()&&!timed_out();++index) {
            const auto& route=routes_[index];
            if(route.explicit_geometry) {++counters_.low_layer_explicit_skips;continue;}
            double priority=0;
            for(auto [node,amount]:route.measured.demand) {
                const int layer=graph_.layer(node);
                if((layer==1&&(options_.low_layer_mask&1))||(layer==11&&(options_.low_layer_mask&2)))
                    priority+=official_reroute_priority(node,amount);
            }
            if(priority>0) queue.push_back({index,priority/std::max<size_t>(1,route.edges.size())});
        }
        std::stable_sort(queue.begin(),queue.end(),[](const auto& a,const auto& b){
            return a.priority!=b.priority?a.priority>b.priority:a.net<b.net;
        });
        counters_.low_layer_queued+=queue.size();
        size_t attempts=0;
        for(const auto& entry:queue) {
            if(timed_out()||attempts++>=options_.low_layer_max_attempts) break;
            low_layer_transaction(entry.net);
            const auto checkpoints=counters_.v9_full_checkpoints;
            if(!finish_rrr_checkpoint()) {restore_counts();break;}
            if(counters_.v9_full_checkpoints!=checkpoints) verified=snapshot();
        }
        if(!finish_rrr_checkpoint(true)) restore_counts();
        v9_rrr_checkpoint_active_=false;
    } catch(const std::exception& error) {
        ++counters_.low_layer_failures;
        if(v9_rrr_checkpoint_active_) {restore_rrr_checkpoint(error.what());restore_counts();}
        else throw;
    }
    std::cout<<"[low-layer] queued="<<counters_.low_layer_queued<<" attempted="<<counters_.low_layer_attempted
        <<" candidates="<<counters_.low_layer_candidates<<" accepted="<<counters_.low_layer_accepted
        <<" deltaQ="<<counters_.low_layer_delta_q<<" deltaL="<<counters_.low_layer_delta_length
        <<" deltaS="<<counters_.low_layer_delta_score<<'\n';
}
} // namespace eda