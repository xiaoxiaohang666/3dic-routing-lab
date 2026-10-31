#include "router/negotiated_router.hpp"
#include <map>

namespace eda {
std::vector<NetRoute> NegotiatedRouter::via_stack_candidates(size_t index,const NetRoute& old) {
    std::vector<NetRoute> result;
    if(!options_.via_stack_radius || !options_.official_model || old.explicit_geometry || timed_out()) return result;
    using Clock=std::chrono::steady_clock;
    const auto start=Clock::now();
    struct Timer {double& seconds;Clock::time_point start;
        ~Timer(){seconds+=std::chrono::duration<double>(Clock::now()-start).count();}
    } timer{counters_.via_stack_seconds,start};
    // Bound per-net proposal work as well as the enclosing shared deadline.
    const auto cutoff=std::min(deadline_,start+std::chrono::milliseconds(20));
    const auto expired=[&]{return Clock::now()>=cutoff;};
    std::map<int,std::vector<int>> adjacent;
    for(auto edge:old.edges) {
        if(expired()) return result;
        const auto [a,b]=edge_nodes(edge);adjacent[a].push_back(b);adjacent[b].push_back(a);
    }
    std::set<int> protected_nodes;
    for(const auto& access:old.accesses) protected_nodes.insert(access.node);
    for(auto [node,amount]:official_protected_demand(index,old)) { (void)amount;protected_nodes.insert(node); }
    for(int node:own_pin_nodes_.at(index)) protected_nodes.insert(node);
    const auto vertical=[&](int a,int b) {
        return graph_.col(a)==graph_.col(b) && graph_.row(a)==graph_.row(b) &&
            std::abs(graph_.layer(a)-graph_.layer(b))==1 &&
            (graph_.layer(a)-1)/10==(graph_.layer(b)-1)/10;
    };
    const auto interior=[&](int n) {
        const auto it=adjacent.find(n);
        return !protected_nodes.count(n) && it!=adjacent.end() && it->second.size()==2 &&
            vertical(n,it->second[0]) && vertical(n,it->second[1]);
    };
    struct Chain {std::vector<int> nodes;double relief{};};
    std::vector<Chain> chains;
    for(const auto& [a,neighbors]:adjacent) {
        if(expired()) return result;
        if(interior(a)) continue;
        for(int b:neighbors) {
            if(!vertical(a,b)||graph_.layer(a)>=graph_.layer(b)) continue;
            Chain chain{{a,b},0};int previous=a,current=b;
            while(interior(current)) {
                const auto& next=adjacent.at(current);
                const int following=next[0]==previous?next[1]:next[0];
                chain.nodes.push_back(following);previous=current;current=following;
            }
            if(chain.nodes.size()<3) continue;
            // Also admit compatible subladders of an opposite-direction
            // maximal ladder. Cut endpoints keep their external via edges.
            for(size_t begin=0;begin+2<chain.nodes.size();++begin)
                for(size_t end=begin+2;end<chain.nodes.size();++end) {
                    const int low=graph_.layer(chain.nodes[begin]),high=graph_.layer(chain.nodes[end]);
                    if(lef_.get_layer_direction(get_layer_name(low))!=lef_.get_layer_direction(get_layer_name(high))) continue;
                    Chain part{{chain.nodes.begin()+begin,chain.nodes.begin()+end+1},0};
                    for(size_t k=1;k+1<part.nodes.size();++k) {
                        const int n=part.nodes[k];
                        const auto it=std::lower_bound(old.measured.demand.begin(),old.measured.demand.end(),n,
                            [](const auto& p,int id){return p.first<id;});
                        if(it!=old.measured.demand.end()&&it->first==n) part.relief+=official_reroute_priority(n,it->second);
                    }
                    ++counters_.via_stack_chains;
                    if(part.relief>0) chains.push_back(std::move(part));
                }
        }
    }
    std::sort(chains.begin(),chains.end(),[](const Chain& a,const Chain& b) {
        return a.relief!=b.relief?a.relief>b.relief:a.nodes<b.nodes;
    });
    if(chains.size()>4) chains.resize(4);
    for(const auto& chain:chains) {
        const int first=chain.nodes.front(),last=chain.nodes.back();
        const int low=graph_.layer(first),high=graph_.layer(last),c=graph_.col(first),r=graph_.row(first);
        const bool horizontal=lef_.get_layer_direction(get_layer_name(low))==Direction::HORIZONTAL;
        for(int distance=1;distance<=options_.via_stack_radius;++distance) for(int sign:{-1,1}) {
            if(expired()) return result;
            const int dc=horizontal?sign*distance:0,dr=horizontal?0:sign*distance;
            if(graph_.node(c+dc,r+dr,low)<0) continue;
            auto trial=old;trial.guides.clear();trial.recovery_subtree=false;trial.recovery_layer=false;
            for(size_t k=1;k<chain.nodes.size();++k) trial.edges.erase(route_edge(chain.nodes[k-1],chain.nodes[k]));
            for(size_t k=1;k+1<chain.nodes.size();++k) trial.nodes.erase(chain.nodes[k]);
            bool legal=true;
            const auto add=[&](int a,int b) {
                if(!graph_.legal_edge(a,b)) {legal=false;return;}
                trial.nodes.insert(a);trial.nodes.insert(b);trial.edges.insert(route_edge(a,b));
            };
            for(int layer:{low,high}) for(int step=0;step<distance;++step)
                add(graph_.node(c+(horizontal?sign*step:0),r+(horizontal?0:sign*step),layer),
                    graph_.node(c+(horizontal?sign*(step+1):0),r+(horizontal?0:sign*(step+1)),layer));
            for(int layer=low;layer<high;++layer)
                add(graph_.node(c+dc,r+dr,layer),graph_.node(c+dc,r+dr,layer+1));
            if(!legal) {++counters_.via_stack_illegal;continue;}
            try {
                encode(index,trial);std::string error;
                if(!validate_route(index,trial.guides,error)||
                   !policy_.validate_mapping(def_.nets()[index],trial.guides,trial.slot,error)||
                   trial.measured.tsv_cells!=old.measured.tsv_cells||trial.measured.tsv_points.size()!=old.measured.tsv_points.size()) {
                    ++counters_.via_stack_illegal;continue;
                }
                bool same=true;
                for(size_t k=0;k<old.measured.tsv_points.size();++k)
                    same=same&&old.measured.tsv_points[k].distance_to(trial.measured.tsv_points[k])==0;
                if(!same) {++counters_.via_stack_illegal;continue;}
                if(expired()) return result;
                ++counters_.via_stack_candidates;result.push_back(std::move(trial));
            } catch(const std::exception&) {++counters_.via_stack_illegal;}
        }
    }
    return result;
}
} // namespace eda
