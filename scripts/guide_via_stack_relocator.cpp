#include "guide_via_stack_relocator.hpp"
#include "validation/gcell_encoding.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <algorithm>
#include <set>
#include <map>
#include <unordered_set>
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
}
GuideViaStackResult relocate_guide_via_stack(const Net& net,const std::vector<GuideRect>& input,
    const RoutingGraph& graph,const LefParser& lef,const OfficialMetricOptions& mo,
    const GuideViaStackOptions& options,const std::vector<int>& demand,const std::function<double(int)>& priority) {
    using Clock=std::chrono::steady_clock;GuideViaStackResult result;result.guides=input;
    const auto expired=[&]{return Clock::now()>=options.deadline;};
    if(expired()){result.timed_out=true;return result;}
    if(options.radius<1||!options.max_trials||options.reference<=0)return result;
    const auto original=measure_official_net_guides(net,input,graph,lef,mo,0);
    const auto normalized=measure_official_net_guides(net,merge(input,lef),graph,lef,mo,0);
    if(original.wirelength!=normalized.wirelength||original.ordinary_connections!=normalized.ordinary_connections||!same_tsv(original,normalized)) {
        ++result.canonical_rejections;return result;
    }
    const auto nodes=coverage(input,graph);Nodes locked;
    for(int n:graph.own_pin_nodes(net))if(nodes.count(n))locked.insert(n);
    for(int cell:original.tsv_cells)for(int layer:{10,11})locked.insert(graph.node(cell%graph.grid().cols,cell/graph.grid().cols,layer));
    const auto vertical=[&](int a,int b){return graph.col(a)==graph.col(b)&&graph.row(a)==graph.row(b)&&std::abs(graph.layer(a)-graph.layer(b))==1&&(graph.layer(a)-1)/10==(graph.layer(b)-1)/10;};
    const auto interior=[&](int n){const auto a=neighbors(n,nodes,graph,lef);return !locked.count(n)&&a.size()==2&&vertical(n,a[0])&&vertical(n,a[1]);};
    struct Window {double rank;std::vector<int> path;};std::vector<Window> windows;
    std::vector<int> ordered(nodes.begin(),nodes.end());std::sort(ordered.begin(),ordered.end());
    for(int first:ordered) {
        if(expired()){result.timed_out=true;return result;}
        if(interior(first))continue;
        for(int adjacent:neighbors(first,nodes,graph,lef)) {
            if(!vertical(first,adjacent)||graph.layer(first)>=graph.layer(adjacent))continue;
            std::vector<int> chain{first,adjacent};int prev=first,cur=adjacent;
            while(interior(cur)) {auto a=neighbors(cur,nodes,graph,lef);const int next=a[0]==prev?a[1]:a[0];chain.push_back(next);prev=cur;cur=next;}
            // Full compatible ladders rank first by relief; compatible subladders
            // also cover maximal chains whose endpoint directions differ.
            for(size_t begin=0;begin+2<chain.size();++begin)for(size_t end=begin+2;end<chain.size();++end) {
                if(lef.get_layer_direction(get_layer_name(graph.layer(chain[begin])))!=lef.get_layer_direction(get_layer_name(graph.layer(chain[end]))))continue;
                double rank=0;for(size_t k=begin+1;k<end;++k)rank+=priority(chain[k]);
                if(rank>0)windows.push_back({rank,{chain.begin()+begin,chain.begin()+end+1}});
            }
        }
    }
    std::sort(windows.begin(),windows.end(),[](const auto& a,const auto& b){return a.rank!=b.rank?a.rank>b.rank:a.path<b.path;});
    result.windows=windows.size();if(windows.size()>options.max_windows)windows.resize(options.max_windows);
    const auto utility=[&](double length,double q){return 45*std::min(1.,options.reference/length)+30/std::sqrt(1+std::max(0.,q));};
    const double before=utility(options.total_L,options.total_Q);
    for(const auto& window:windows)for(int distance=1;distance<=options.radius;++distance)for(int sign:{-1,1}) {
        if(expired()){result.timed_out=true;return result;}if(result.trials>=options.max_trials)return result;++result.trials;
        const auto& path=window.path;const int first=path.front(),last=path.back(),lo=graph.layer(first),hi=graph.layer(last),c=graph.col(first),r=graph.row(first);
        const bool h=lef.get_layer_direction(get_layer_name(lo))==Direction::HORIZONTAL;
        const int nc=c+(h?sign*distance:0),nr=r+(h?0:sign*distance);
        if(graph.node(nc,nr,lo)<0)continue;
        Nodes target=nodes;for(size_t k=1;k+1<path.size();++k)target.erase(path[k]);
        for(int layer=lo;layer<=hi;++layer)target.insert(graph.node(nc,nr,layer));
        for(int layer:{lo,hi})for(int step=0;step<=distance;++step)target.insert(graph.node(c+(h?sign*step:0),r+(h?0:sign*step),layer));
        std::vector<int> removed,added;
        for(int n:nodes)if(!target.count(n))removed.push_back(n);
        for(int n:target)if(!nodes.count(n))added.push_back(n);
        if(removed.empty()||added.empty())continue;
        bool illegal=false;for(int n:locked)if(!target.count(n))illegal=true;
        for(const auto* changes:{&removed,&added})for(int n:*changes)
            if(graph.tsv_obstacle_layers(graph.col(n),graph.row(n),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(graph.layer(n)-1)))illegal=true;
        if(illegal){++result.pin_rejections;continue;}
        auto trial=input;for(int n:removed)trial=subtract_cell(trial,n,graph,lef);
        for(int n:added){const auto b=graph.cell(n);trial.emplace_back(b.x1,b.y1,b.x2,b.y2,get_layer_name(graph.layer(n)));}
        trial=merge(std::move(trial),lef);std::string error;
        if(coverage(trial,graph)!=target||!validate_gcell_encoding(net,graph.die_area(),graph.grid(),lef,trial,error)||!validate_net_guides(net,graph.die_area(),lef,trial,error,mo.gcell_width_dbu,mo.gcell_height_dbu)) {++result.invalid;continue;}
        const auto next=measure_official_net_guides(net,trial,graph,lef,mo,0);
        if(!same_tsv(original,next)||(!options.allow_extra_vias&&next.ordinary_connections>original.ordinary_connections)){++result.invalid;continue;}
        std::map<int,int> delta;for(auto [n,d]:original.demand)delta[n]-=d;for(auto [n,d]:next.demand)delta[n]+=d;
        long double dq=0;for(auto [n,d]:delta)if(d) {
            if(graph.tsv_obstacle_layers(graph.col(n),graph.row(n),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(graph.layer(n)-1)))illegal=true;
            dq+=official_resource_cell_contribution(graph,n,demand.at(n)+d).squared-official_resource_cell_contribution(graph,n,demand.at(n)).squared;
        }
        const double dl=next.wirelength-original.wirelength,total=options.total_L+dl,gain=utility(total,options.total_Q+double(dq))-before;
        if(illegal||total>options.max_total_L||dq>=-1e-14L||gain<=std::max(1e-9,result.gain)||options.reject_all){++result.dominated;continue;}
        if(expired()){result.timed_out=true;return result;}
        result.guides=std::move(trial);result.removed=std::move(removed);result.added=std::move(added);result.delta_L=dl;result.delta_Q=double(dq);result.gain=gain;
    }
    return result;
}
}
