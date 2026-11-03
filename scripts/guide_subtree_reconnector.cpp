#include "guide_subtree_reconnector.hpp"
#include "validation/gcell_encoding.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <algorithm>
#include <map>
#include <queue>
#include <array>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
namespace eda { namespace {
using Nodes=std::unordered_set<int>;
Nodes coverage(const std::vector<GuideRect>& guides,const RoutingGraph& graph) {
    Nodes result;const auto& g=graph.grid();
    for(const auto& s:guides) {
        const int cb=std::max(0,int(std::upper_bound(g.x_lines.begin(),g.x_lines.end(),s.x1)-g.x_lines.begin())-1);
        const int ce=std::min(g.cols,int(std::lower_bound(g.x_lines.begin(),g.x_lines.end(),s.x2)-g.x_lines.begin()));
        const int rb=std::max(0,int(std::upper_bound(g.y_lines.begin(),g.y_lines.end(),s.y1)-g.y_lines.begin())-1);
        const int re=std::min(g.rows,int(std::lower_bound(g.y_lines.begin(),g.y_lines.end(),s.y2)-g.y_lines.begin()));
        for(int r=rb;r<re;++r) for(int c=cb;c<ce;++c) result.insert(graph.node(c,r,get_layer_index(s.layer)));
    }
    return result;
}
std::vector<int> neighbors(int n,const Nodes& nodes,const RoutingGraph& graph,const LefParser& lef) {
    std::vector<int> out;const int c=graph.col(n),r=graph.row(n),l=graph.layer(n);
    const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
    for(int x:{graph.node(c+(h?1:0),r+(h?0:1),l),graph.node(c-(h?1:0),r-(h?0:1),l),
               graph.node(c,r,l-1),graph.node(c,r,l+1)}) if(x>=0 && nodes.count(x)) out.push_back(x);
    return out;
}
std::vector<GuideRect> subtract_cell(const std::vector<GuideRect>& input,int node,
                                    const RoutingGraph& graph,const LefParser& lef) {
    const auto box=graph.cell(node);const int layer=graph.layer(node);std::vector<GuideRect> out;
    for(const auto& r:input) {
        if(get_layer_index(r.layer)!=layer || std::max(r.x1,box.x1)>=std::min(r.x2,box.x2) ||
           std::max(r.y1,box.y1)>=std::min(r.y2,box.y2)) {out.push_back(r);continue;}
        // Only remove physical coverage. Preserve every unrelated rectangle;
        // never normalize/reorder rectangles to reduce connection charges.
        const bool h=lef.get_layer_direction(r.layer)==Direction::HORIZONTAL;
        if(h) {
            if(r.x1<box.x1) out.emplace_back(r.x1,r.y1,box.x1,r.y2,r.layer);
            if(box.x2<r.x2) out.emplace_back(box.x2,r.y1,r.x2,r.y2,r.layer);
        } else {
            if(r.y1<box.y1) out.emplace_back(r.x1,r.y1,r.x2,box.y1,r.layer);
            if(box.y2<r.y2) out.emplace_back(r.x1,box.y2,r.x2,r.y2,r.layer);
        }
    }
    return out;
}

std::vector<GuideRect> merge(std::vector<GuideRect> rects,const LefParser& lef) {
    const auto key=[&](const GuideRect& r) {const bool h=lef.get_layer_direction(r.layer)==Direction::HORIZONTAL;
        return std::tuple(get_layer_index(r.layer),h?r.y1:r.x1,h?r.y2:r.x2,h?r.x1:r.y1,h?r.x2:r.y2);};
    std::sort(rects.begin(),rects.end(),[&](const auto& a,const auto& b){return key(a)<key(b);});
    std::vector<GuideRect> out;
    for(const auto& r:rects) {
        const bool h=lef.get_layer_direction(r.layer)==Direction::HORIZONTAL;bool joined=false;
        if(!out.empty()) {auto& b=out.back();if(b.layer==r.layer&&(h?(b.y1==r.y1&&b.y2==r.y2&&r.x1<=b.x2):(b.x1==r.x1&&b.x2==r.x2&&r.y1<=b.y2))) {
            if(h)b.x2=std::max(b.x2,r.x2);else b.y2=std::max(b.y2,r.y2);joined=true;}}
        if(!joined)out.push_back(r);
    }return out;
}
bool same_tsv(const NetGuideMetrics& a,const NetGuideMetrics& b) {
    if(a.tsv_cells!=b.tsv_cells||a.tsv_connections!=b.tsv_connections||a.tsv_points.size()!=b.tsv_points.size())return false;
    for(size_t i=0;i<a.tsv_points.size();++i)if(a.tsv_points[i].distance_to(b.tsv_points[i])!=0)return false;
    return true;
}
using Clock=std::chrono::steady_clock;
struct Box {int cb,ce,rb,re,lo,hi;bool contains(int n,const RoutingGraph& g) const {
    return n>=0&&g.col(n)>=cb&&g.col(n)<=ce&&g.row(n)>=rb&&g.row(n)<=re&&g.layer(n)>=lo&&g.layer(n)<=hi;
}};
struct SearchResult {std::vector<int> path;size_t expansions{};bool exhausted{};};
struct Profile {const char* name;double congestion,via;};
SearchResult search(const Nodes& sources,const Nodes& goals,const Nodes& target,const Nodes& removed,
    const Box& box,const RoutingGraph& graph,const LefParser& lef,const std::vector<int>& demand,
    const std::map<int,int>& old_demand,const Profile& profile,double price,const GuideSubtreeOptions& o,
    Clock::time_point cutoff) {
    SearchResult result;if(sources.empty()||goals.empty())return result;
    const double pitch=std::max(graph.grid().x_lines[1]-graph.grid().x_lines[0],graph.grid().y_lines[1]-graph.grid().y_lines[0]);
    struct Label {double g;uint64_t parent;};struct Item {double f,g;uint64_t state;};
    auto cmp=[](const Item& a,const Item& b){return std::tie(a.f,a.g,a.state)>std::tie(b.f,b.g,b.state);};
    std::priority_queue<Item,std::vector<Item>,decltype(cmp)> heap(cmp);std::unordered_map<uint64_t,Label> labels;
    constexpr uint64_t none=UINT64_MAX;
    const auto heuristic=[&](int n){double best=std::numeric_limits<double>::infinity();const auto p=graph.point(n);
        for(int goal:goals){const auto t=graph.point(goal);best=std::min(best,(std::abs(p.x-t.x)+std::abs(p.y-t.y))/pitch+3*std::abs(graph.layer(n)-graph.layer(goal)));}return best;};
    std::vector<int> ordered(sources.begin(),sources.end());std::sort(ordered.begin(),ordered.end());
    for(int n:ordered){const uint64_t s=uint64_t(n)*3+2;labels.emplace(s,Label{0,none});heap.push({heuristic(n),0,s});}
    while(!heap.empty()) {
        if(Clock::now()>=cutoff||result.expansions>=o.max_expansions){result.exhausted=true;return result;}
        const auto item=heap.top();heap.pop();auto it=labels.find(item.state);if(it==labels.end()||item.g!=it->second.g)continue;
        const int n=int(item.state/3),incoming=int(item.state%3);++result.expansions;
        if(goals.count(n)) {uint64_t s=item.state;while(s!=none){result.path.push_back(int(s/3));s=labels.at(s).parent;}std::reverse(result.path.begin(),result.path.end());return result;}
        const int c=graph.col(n),r=graph.row(n),l=graph.layer(n);const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
        const std::array<int,4> nexts={graph.node(c+(h?1:0),r+(h?0:1),l),graph.node(c-(h?1:0),r-(h?0:1),l),graph.node(c,r,l-1),graph.node(c,r,l+1)};
        for(int next:nexts) {
            if(!box.contains(next,graph)||!graph.legal_edge(n,next))continue;
            if(!target.count(next)&&(graph.tsv_obstacle_layers(graph.col(next),graph.row(next),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(graph.layer(next)-1))))continue;
            const bool via=graph.layer(next)!=l;const int kind=via?1:0;
            const double length=via?3:graph.point(n).distance_to(graph.point(next))/pitch;
            double q=0;
            if(!target.count(next)) {int base=demand.at(next);if(removed.count(next)){auto old=old_demand.find(next);if(old!=old_demand.end())base-=old->second;}
                q=double(official_resource_cell_contribution(graph,next,base+1).squared-official_resource_cell_contribution(graph,next,base).squared);}
            const double cost=length+(via?profile.via:0)+profile.congestion*price*std::max(0.,q)+(incoming!=2&&incoming!=kind?.05:0.);
            const double g=item.g+cost;const uint64_t s=uint64_t(next)*3+kind;auto at=labels.find(s);
            if(at==labels.end()||g<at->second.g-1e-12){labels[s]={g,item.state};heap.push({g+heuristic(next),g,s});}
        }
    }return result;
}
bool dominates(const GuideSubtreeCandidate& a,const GuideSubtreeCandidate& b) {
    return a.delta_L<=b.delta_L+1e-8&&a.delta_Q<=b.delta_Q+1e-12&&a.delta_vias<=b.delta_vias;
}
}
GuideSubtreeResult reconnect_guide_subtree(const Net& net,const std::vector<GuideRect>& input,
    const RoutingGraph& graph,const LefParser& lef,const OfficialMetricOptions& mo,const GuideSubtreeOptions& o,
    const std::vector<int>& demand,const std::function<double(int)>& priority) {
    GuideSubtreeResult result;result.guides=input;
    const auto start=Clock::now();if(start>=o.deadline){result.timed_out=true;return result;}
    if(o.radius<1||!o.max_trials||o.reference<=0)return result;
    const auto cutoff=std::min(o.deadline,start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(o.net_seconds)));
    const auto original=measure_official_net_guides(net,input,graph,lef,mo,0);
    const auto normalized=measure_official_net_guides(net,merge(input,lef),graph,lef,mo,0);
    if(original.wirelength!=normalized.wirelength||original.ordinary_connections!=normalized.ordinary_connections||!same_tsv(original,normalized)){++result.canonical_rejections;return result;}
    const auto nodes=coverage(input,graph);Nodes locked;for(int n:graph.own_pin_nodes(net))if(nodes.count(n))locked.insert(n);
    for(int cell:original.tsv_cells)for(int layer:{10,11})locked.insert(graph.node(cell%graph.grid().cols,cell/graph.grid().cols,layer));
    std::map<int,int> old_demand;for(auto [n,d]:original.demand)old_demand[n]=d;
    const auto pin=[&](int n){return (graph.tsv_obstacle_layers(graph.col(n),graph.row(n),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(graph.layer(n)-1)))!=0;};
    std::vector<std::pair<double,int>> seeds;for(int n:nodes)if(priority(n)>0&&!locked.count(n)&&!pin(n))seeds.emplace_back(priority(n),n);
    std::sort(seeds.begin(),seeds.end(),[](auto a,auto b){return a.first!=b.first?a.first>b.first:a.second<b.second;});
    std::set<std::vector<int>> seen_patches,seen_targets;
    const auto utility=[&](double L,double Q){return 45*std::min(1.,o.reference/L)+30/std::sqrt(1+std::max(0.,Q));};
    const double before=utility(o.total_L,o.total_Q),pitch=std::max(mo.gcell_width_dbu,mo.gcell_height_dbu);
    const double price=std::clamp((15/std::pow(1+o.total_Q,1.5))/(45*o.reference/(o.total_L*o.total_L)*pitch),1.,1e6);
    const std::array<Profile,4> profiles={{{"low_congestion",8,0},{"low_length",.02,0},{"low_via",.2,8},{"balanced",1,0}}};
    for(auto [rank,seed]:seeds) for(int radius=1;radius<=o.radius;++radius) {
        (void)rank;if(Clock::now()>=cutoff||result.windows>=o.max_windows||result.trials>=o.max_trials)goto finish;
        const int c=graph.col(seed),r=graph.row(seed),die=(graph.layer(seed)-1)/10;
        const auto in_patch=[&](int n){return (graph.layer(n)-1)/10==die&&std::abs(graph.col(n)-c)<=radius&&std::abs(graph.row(n)-r)<=radius&&!locked.count(n)&&!pin(n);};
        Nodes removed;removed.insert(seed);std::vector<int> todo{seed};
        while(!todo.empty()&&removed.size()<=o.max_patch_nodes){int n=todo.back();todo.pop_back();for(int next:neighbors(n,nodes,graph,lef))if(in_patch(next)&&removed.insert(next).second)todo.push_back(next);}
        if(removed.size()>o.max_patch_nodes){++result.extraction_rejections;continue;}
        std::vector<int> patch(removed.begin(),removed.end());std::sort(patch.begin(),patch.end());if(!seen_patches.insert(patch).second)continue;
        Nodes retained=nodes;for(int n:removed)retained.erase(n);
        if(retained.empty()){++result.extraction_rejections;continue;}
        std::vector<Nodes> components;Nodes unseen=retained;
        while(!unseen.empty()) {int first=*std::min_element(unseen.begin(),unseen.end());unseen.erase(first);Nodes part{first};todo={first};
            while(!todo.empty()){int n=todo.back();todo.pop_back();for(int next:neighbors(n,retained,graph,lef))if(unseen.erase(next)){part.insert(next);todo.push_back(next);}}components.push_back(std::move(part));}
        if(components.size()<2||components.size()>o.max_components){++result.extraction_rejections;continue;}
        ++result.windows;const int margin=radius+o.search_margin;
        Box box{std::max(0,c-margin),std::min(graph.grid().cols-1,c+margin),std::max(0,r-margin),std::min(graph.grid().rows-1,r+margin),die*10+1,die*10+10};
        // Re-rooting the greedy component tree produces alternative Steiner
        // junctions without changing any retained endpoint subtree.
        for(size_t root=0;root<std::min<size_t>(2,components.size());++root)for(const auto& profile:profiles) {
            if(Clock::now()>=cutoff||result.trials>=o.max_trials)goto finish;
            ++result.trials;
            Nodes target=retained,connected=components[root];std::vector<bool> joined(components.size());joined[root]=true;bool complete=true;
            for(size_t joins=1;joins<components.size();++joins) {
                Nodes sources,goals;for(int n:connected)if(box.contains(n,graph))sources.insert(n);
                for(size_t i=0;i<components.size();++i)if(!joined[i])for(int n:components[i])if(box.contains(n,graph))goals.insert(n);
                auto found=search(sources,goals,target,removed,box,graph,lef,demand,old_demand,profile,price,o,cutoff);result.astar_expansions+=found.expansions;result.search_bound_hits+=found.exhausted;
                if(found.path.empty()){complete=false;break;}
                for(int n:found.path){target.insert(n);connected.insert(n);}
                bool added=false;for(size_t i=0;i<components.size();++i)if(!joined[i]&&components[i].count(found.path.back())){joined[i]=true;connected.insert(components[i].begin(),components[i].end());added=true;break;}
                if(!added){complete=false;break;}
            }
            if(!complete)continue;
            std::vector<int> key(target.begin(),target.end());std::sort(key.begin(),key.end());if(!seen_targets.insert(key).second)continue;
            GuideSubtreeCandidate candidate;candidate.components=components.size();candidate.profile=profile.name;
            for(int n:nodes)if(!target.count(n))candidate.removed.push_back(n);
            for(int n:target)if(!nodes.count(n))candidate.added.push_back(n);
            if(candidate.removed.empty())continue;
            bool illegal=false;for(int n:locked)if(!target.count(n))illegal=true;
            for(int n:candidate.removed)if(pin(n))illegal=true;
            for(int n:candidate.added)if(pin(n))illegal=true;
            if(illegal){++result.pin_rejections;continue;}
            candidate.guides=input;for(int n:candidate.removed)candidate.guides=subtract_cell(candidate.guides,n,graph,lef);
            for(int n:candidate.added){const auto b=graph.cell(n);candidate.guides.emplace_back(b.x1,b.y1,b.x2,b.y2,get_layer_name(graph.layer(n)));}
            candidate.guides=merge(std::move(candidate.guides),lef);std::string error;
            if(coverage(candidate.guides,graph)!=target||!validate_gcell_encoding(net,graph.die_area(),graph.grid(),lef,candidate.guides,error)||!validate_net_guides(net,graph.die_area(),lef,candidate.guides,error,mo.gcell_width_dbu,mo.gcell_height_dbu)){++result.invalid;continue;}
            const auto next=measure_official_net_guides(net,candidate.guides,graph,lef,mo,0);if(!same_tsv(original,next)){++result.invalid;continue;}
            std::map<int,int> delta;for(auto [n,d]:original.demand)delta[n]-=d;for(auto [n,d]:next.demand)delta[n]+=d;
            long double dq=0;for(auto [n,d]:delta)if(d){if(pin(n))illegal=true;dq+=official_resource_cell_contribution(graph,n,demand.at(n)+d).squared-official_resource_cell_contribution(graph,n,demand.at(n)).squared;}
            candidate.delta_L=next.wirelength-original.wirelength;candidate.delta_Q=double(dq);candidate.delta_vias=static_cast<long long>(next.ordinary_connections)-static_cast<long long>(original.ordinary_connections);
            candidate.gain=utility(o.total_L+candidate.delta_L,o.total_Q+candidate.delta_Q)-before;
            if(illegal||o.total_L+candidate.delta_L>o.max_total_L){++result.invalid;continue;}
            if(components.size()>2)++result.multi_terminal_candidates;
            bool dominated=false;for(const auto& other:result.pareto)if(dominates(other,candidate)){dominated=true;break;}
            if(!dominated){std::erase_if(result.pareto,[&](const auto& x){return dominates(candidate,x);});result.pareto.push_back(std::move(candidate));}
        }
    }
finish:
    result.timed_out=Clock::now()>=o.deadline;result.work_exhausted=Clock::now()>=cutoff&&!result.timed_out;
    // Preserve the four requested objective extrema before bounding storage.
    if(result.pareto.size()>o.pareto_limit&&o.pareto_limit>=4) {
        std::set<size_t> keep;for(int objective=0;objective<4;++objective){size_t best=0;for(size_t i=1;i<result.pareto.size();++i){const auto& a=result.pareto[i];const auto& b=result.pareto[best];bool better=objective==0?a.delta_Q<b.delta_Q:objective==1?a.delta_L<b.delta_L:objective==2?a.delta_vias<b.delta_vias:a.gain>b.gain;if(better)best=i;}keep.insert(best);}
        std::vector<size_t> order(result.pareto.size());std::iota(order.begin(),order.end(),0);std::sort(order.begin(),order.end(),[&](size_t a,size_t b){return result.pareto[a].gain>result.pareto[b].gain;});for(size_t i:order)if(keep.size()<o.pareto_limit)keep.insert(i);
        std::vector<GuideSubtreeCandidate> limited;for(size_t i:keep)limited.push_back(std::move(result.pareto[i]));result.pareto=std::move(limited);
    }
    if(!o.reject_all&&!result.timed_out)for(const auto& candidate:result.pareto)if(candidate.delta_Q<=1e-12&&candidate.gain>std::max(1e-9,result.gain)) {
        result.guides=candidate.guides;result.removed=candidate.removed;result.added=candidate.added;result.delta_L=candidate.delta_L;result.delta_Q=candidate.delta_Q;result.gain=candidate.gain;
    }
    return result;
}
}
