#include "router/shared_hub_optimizer.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "validation/gcell_encoding.hpp"
#include <array>
#include <iomanip>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace eda {
namespace {
using Clock=std::chrono::steady_clock;
using Cells=std::unordered_set<int>;
struct Tail {
    Cells retained;
    std::array<int,2> anchor{{-1,-1}};
    std::array<std::vector<int>,2> removed;
    bool valid{false};
    std::string reason;
};
std::vector<int> neighbors(int node,const RoutingGraph& graph,const LefParser& lef,int low,int high) {
    const int c=graph.col(node),r=graph.row(node),l=graph.layer(node);
    std::vector<int> out;
    const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
    if(h) {if(c>0) out.push_back(graph.node(c-1,r,l));if(c+1<graph.grid().cols) out.push_back(graph.node(c+1,r,l));}
    else {if(r>0) out.push_back(graph.node(c,r-1,l));if(r+1<graph.grid().rows) out.push_back(graph.node(c,r+1,l));}
    if(l>low) out.push_back(graph.node(c,r,l-1));
    if(l<high) out.push_back(graph.node(c,r,l+1));
    return out;
}
double cell_length(int node,const RoutingGraph& graph,const LefParser& lef,const OfficialMetricOptions& m) {
    return lef.get_layer_direction(get_layer_name(graph.layer(node)))==Direction::HORIZONTAL?m.gcell_width_dbu:m.gcell_height_dbu;
}
Cells expand(const std::vector<GuideRect>& guides,const RoutingGraph& graph) {
    Cells result;const auto& grid=graph.grid();
    for(const auto& g:guides) {
        const int l=get_layer_index(g.layer),c1=grid.get_col(g.x1),r1=grid.get_row(g.y1);
        // Integral GCell-aligned rectangles have positive area; evaluate the
        // upper bound strictly inside it, independently of PIN boundary rules.
        const int c2=grid.get_col(std::nextafter(g.x2,g.x1)),r2=grid.get_row(std::nextafter(g.y2,g.y1));
        if(c1<0||r1<0||c2>=grid.cols||r2>=grid.rows||l<1||l>20)
            throw std::runtime_error("shared hub Guide lies outside grid");
        for(int r=r1;r<=r2;++r) for(int c=c1;c<=c2;++c) result.insert(graph.node(c,r,l));
    }
    return result;
}
std::vector<GuideRect> encode(const Cells& cells,const RoutingGraph& graph,const LefParser& lef) {
    std::map<std::pair<int,int>,std::vector<int>> runs;
    for(int n:cells) {
        const int l=graph.layer(n);const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
        runs[{l,h?graph.row(n):graph.col(n)}].push_back(h?graph.col(n):graph.row(n));
    }
    std::vector<GuideRect> result;
    for(auto& [key,values]:runs) {
        std::sort(values.begin(),values.end());
        const auto [l,axis]=key;const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
        for(size_t i=0;i<values.size();) {
            const int first=values[i];int last=first;
            while(++i<values.size()&&values[i]==last+1) last=values[i];
            const auto a=graph.cell(graph.node(h?first:axis,h?axis:first,l));
            const auto b=graph.cell(graph.node(h?last:axis,h?axis:last,l));
            const auto& die=graph.die_area();
            result.emplace_back(std::max(a.x1,die.x1),std::max(a.y1,die.y1),
                std::min(b.x2,die.x2),std::min(b.y2,die.y2),get_layer_name(l));
        }
    }
    return result;
}
Tail detach(const Net& net,const std::vector<GuideRect>& guides,int cell,
            const RoutingGraph& graph,const LefParser& lef,size_t limit) {
    Tail out;out.retained=expand(guides,graph);
    const auto pins=graph.own_pin_nodes(net);const int c=cell%graph.grid().cols,r=cell/graph.grid().cols;
    for(int side=0;side<2;++side) {
        int n=graph.node(c,r,side?11:10);const int low=side?11:1,high=side?20:10;
        while(true) {
            if(!out.retained.count(n)) {out.reason="missing-interface-root";return out;}
            std::vector<int> adjacent;
            for(int v:neighbors(n,graph,lef,low,high)) if(out.retained.count(v)) adjacent.push_back(v);
            const bool protected_pin=std::binary_search(pins.begin(),pins.end(),n);
            if(adjacent.size()!=1||protected_pin) {
                if(out.removed[side].empty()) {out.reason="interface-root-is-branch-or-pin";return out;}
                out.anchor[side]=n;break;
            }
            if(out.removed[side].size()>=limit) {out.reason="attachment-tail-budget";return out;}
            out.removed[side].push_back(n);out.retained.erase(n);n=adjacent.front();
        }
    }
    out.valid=true;return out;
}
bool attach(Cells& cells,int root,int anchor,int side,const std::vector<int>& own_pins,
    const RoutingGraph& graph,const LefParser& lef,const OfficialMetricOptions& metric,
    size_t cap,Clock::time_point deadline,size_t& expansions) {
    const int low=side?11:1,high=side?20:10,interface=side?11:10;
    const int pad=3;
    const int c1=std::max(0,std::min(graph.col(root),graph.col(anchor))-pad);
    const int c2=std::min(graph.grid().cols-1,std::max(graph.col(root),graph.col(anchor))+pad);
    const int r1=std::max(0,std::min(graph.row(root),graph.row(anchor))-pad);
    const int r2=std::min(graph.grid().rows-1,std::max(graph.row(root),graph.row(anchor))+pad);
    struct State {double cost;int parent;};
    std::unordered_map<int,State> seen;
    using Item=std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    const double pitch=std::max(metric.gcell_width_dbu,metric.gcell_height_dbu);
    seen.emplace(root,State{cell_length(root,graph,lef,metric),-1});queue.emplace(seen.at(root).cost,root);
    size_t count=0;
    while(!queue.empty()&&count<cap) {
        const auto [cost,n]=queue.top();queue.pop();
        if(cost!=seen.at(n).cost) continue;
        if((count++%128)==0&&Clock::now()>=deadline) return false;
        ++expansions;
        if(cells.count(n)) {
            for(int p=n;p!=-1;p=seen.at(p).parent) cells.insert(p);
            return true;
        }
        for(int v:neighbors(n,graph,lef,low,high)) {
            if(graph.col(v)<c1||graph.col(v)>c2||graph.row(v)<r1||graph.row(v)>r2) continue;
            // Do not introduce another M10/M11 contact, including one in the
            // untouched tree. Full exact measurement checks the final result.
            if(graph.layer(v)==interface&&v!=root&&
               cells.count(graph.node(graph.col(v),graph.row(v),side?10:11))) continue;
            if(!cells.count(v)&&std::binary_search(own_pins.begin(),own_pins.end(),v)) continue;
            if(!cells.count(v)&&graph.official_capacity(v)<=0) continue;
            const double extra=(cells.count(v)?0:cell_length(v,graph,lef,metric))+
                (graph.layer(n)!=graph.layer(v)?metric.via_cost_ratio*pitch:0);
            const double next=cost+extra;
            auto f=seen.find(v);
            if(f==seen.end()||next<f->second.cost) {
                seen[v]={next,n};queue.emplace(next,v);
            }
        }
    }
    return false;
}
bool near(double a,double b) {return std::abs(a-b)<=1e-8*std::max({1.,std::abs(a),std::abs(b)});}
SharedHubQuality quality(double L,double Q,double rho,double ref,size_t via,size_t zero,const OfficialMetricOptions& m) {
    SharedHubQuality q;q.L=L;q.Q=Q;q.rho=rho;q.ordinary_vias=via;q.zero_capacity=zero;
    q.W=official_wirelength_score(L,ref,m);q.C=100/std::sqrt(1+Q);q.T=50*(1+std::tanh(m.thermal_gain*rho));
    q.S=.45*q.W+.30*q.C+.25*q.T;return q;
}
void write_quality(std::ostream& out,const SharedHubQuality& q) {
    out<<"{\"L\":"<<q.L<<",\"Q\":"<<q.Q<<",\"rho\":"<<q.rho<<",\"W\":"<<q.W<<",\"C\":"<<q.C<<",\"T\":"<<q.T
       <<",\"S\":"<<q.S<<",\"ordinary_vias\":"<<q.ordinary_vias<<",\"zero_capacity\":"<<q.zero_capacity<<'}';
}
} // namespace

bool shared_hub_cell_geometry(const std::vector<GuideRect>& guides,int cell,const RoutingGraph& graph) {
    const auto& grid=graph.grid();
    if(cell<0||size_t(cell)>=size_t(grid.cols)*grid.rows) return false;
    const auto box=graph.cell(graph.node(cell%grid.cols,cell/grid.cols,10));const auto& die=graph.die_area();
    const Rect landing{std::ceil(std::max(box.x1,die.x1)),std::ceil(std::max(box.y1,die.y1)),
                       std::floor(std::min(box.x2,die.x2)),std::floor(std::min(box.y2,die.y2))};
    if(landing.width()<=0||landing.height()<=0) return false;
    constexpr double eps=1e-6;size_t positive=0;
    for(const auto& a:guides) if(get_layer_index(a.layer)==10)
        for(const auto& b:guides) if(get_layer_index(b.layer)==11) {
            const double x1=std::max(a.x1,b.x1),y1=std::max(a.y1,b.y1);
            const double x2=std::min(a.x2,b.x2),y2=std::min(a.y2,b.y2);
            if(x1>x2+eps||y1>y2+eps) continue;
            // Match the existing conservative electrical mapping contract:
            // even an edge/corner contact must lie within the selected TSV.
            if(x1<landing.x1-eps||y1<landing.y1-eps||x2>landing.x2+eps||y2>landing.y2+eps) return false;
            if(x1<x2&&y1<y2) {
                ++positive;
                if(std::abs(x1-landing.x1)>eps||std::abs(y1-landing.y1)>eps||
                   std::abs(x2-landing.x2)>eps||std::abs(y2-landing.y2)>eps) return false;
            }
        }
    return positive==1;
}

SharedHubReport optimize_shared_hubs(const DefParser& def,const LefParser& lef,const RoutingGraph& graph,
    const ThermalParser& thermal,const OfficialMetricOptions& metric,const std::vector<double>& references,
    GuideMap& guides,const SharedHubOptions& options) {
    const auto started=Clock::now();SharedHubReport report;
    if(!metric.a150_rules||!metric.tsv_track_demand||!metric.all_adjacent_overlaps||
       !metric.score_20261020||!graph.pin_zero_track_floor()||graph.congestion_overflow_scale()!=1)
        throw std::invalid_argument("shared hub experiment requires unchanged production A150 site contract");
    if(references.size()!=def.nets().size()||guides.size()!=def.nets().size()||options.radius<1||options.radius>32||
       options.max_candidates<1||options.max_candidates>128||options.search_nodes<1||options.max_tail_nodes<1||
       !std::isfinite(options.seconds)||options.seconds<0)
        throw std::invalid_argument("invalid shared hub dimensions or search budget");
    std::vector<NetGuideMetrics> measured;measured.reserve(def.nets().size());
    std::vector<int> demand(graph.size());std::map<int,std::vector<size_t>> groups;
    const auto& grid=graph.grid();const size_t cells=size_t(grid.cols)*grid.rows;
    std::vector<double> heat(cells);std::vector<int> counts(cells);
    for(int r=0;r<grid.rows;++r) for(int c=0;c<grid.cols;++c)
        heat[size_t(r)*grid.cols+c]=std::max(0.,thermal.get_temperature(c,r)-thermal.ambient_temperature());
    size_t vias=0;
    for(size_t i=0;i<def.nets().size();++i) {
        const auto& net=def.nets()[i];const auto f=guides.find(net.name);std::string error;
        if(f==guides.end()||!validate_gcell_encoding(net,def.die_area(),grid,lef,f->second,error)||
           !validate_net_guides(net,def.die_area(),lef,f->second,error,def.gcell_pitch_x(),def.gcell_pitch_y()))
            throw std::runtime_error("invalid shared hub input net "+net.name+": "+error);
        measured.push_back(measure_official_net_guides(net,f->second,graph,lef,metric,references[i]));
        const auto& m=measured.back();vias+=m.ordinary_connections;
        for(auto [node,amount]:m.demand) demand.at(size_t(node))+=amount;
        if(net.is_cross_die) {
            if(m.tsv_cells.size()!=1||m.tsv_connections!=1) throw std::runtime_error("shared hub input needs one actual TSV per cross-die net");
            const int cell=m.tsv_cells.front();
            if(!shared_hub_cell_geometry(f->second,cell,graph))
                throw std::runtime_error("shared hub input has extra interface contacts or a non-cell TSV overlap");
            if(!graph.tsv_landing_legal(cell%grid.cols,cell/grid.cols,TsvObstacleScope::ALL_LAYERS))
                throw std::runtime_error("shared hub input violates all-layer TSV ban");
            groups[cell].push_back(i);++counts.at(size_t(cell));
        }
    }
    std::vector<const NetGuideMetrics*> pointers;for(const auto& m:measured) pointers.push_back(&m);
    const auto initial=aggregate_official_metrics(pointers,demand,graph,thermal,metric,false);
    const double reference=initial.guide.reference;
    report.before=quality(initial.guide.wirelength,initial.squared_normalized_overflow,initial.guide.correlation,
        reference,vias,initial.zero_capacity_cells,metric);
    report.after=report.before;
    // Whole-group moves to empty cells preserve sum(count^2); only the heat
    // product changes. Compute the exact count-based Pearson denominator once.
    long double hs=0,h2=0,cs=0,c2=0;
    for(size_t i=0;i<cells;++i) {hs+=heat[i];h2+=(long double)heat[i]*heat[i];cs+=counts[i];c2+=(long double)counts[i]*counts[i];}
    const long double denominator=std::sqrt(std::max(0.L,h2-hs*hs/cells)*std::max(0.L,c2-cs*cs/cells));
    for(const auto& [cell,members]:groups) if(members.size()>1) {
        SharedHubGroup group;group.cell=cell;group.members=members;group.eligible=true;
        for(size_t i:members) {
            const auto& net=def.nets()[i];const auto& raw=guides.at(net.name);
            const auto tail=detach(net,raw,cell,graph,lef,options.max_tail_nodes);
            if(!tail.valid) {group.eligible=false;group.reason=tail.reason;break;}
            // Reject representation-sensitive baselines. An apparent gain from
            // merging old rectangles is outside a hub-position experiment.
            const auto canonical=encode(expand(raw,graph),graph,lef);
            const auto canonical_metric=measure_official_net_guides(net,canonical,graph,lef,metric,references[i]);
            if(!near(canonical_metric.wirelength,measured[i].wirelength)||
               canonical_metric.ordinary_connections!=measured[i].ordinary_connections||canonical_metric.demand!=measured[i].demand) {
                group.eligible=false;group.reason="representation-sensitive-baseline";break;
            }
            // Actual removable cells and ordinary-via interfaces, rather than a
            // PIN median/HPWL, rank a bounded number of expensive groups.
            for(const auto& side:tail.removed) for(int n:side) {
                group.detachable_L+=cell_length(n,graph,lef,metric);++group.detached_nodes;
            }
            const auto original_cells=expand(raw,graph);
            for(int n:original_cells) if(graph.layer(n)!=10&&graph.layer(n)<20) {
                const int up=graph.node(graph.col(n),graph.row(n),graph.layer(n)+1);
                if(original_cells.count(up)&&(!tail.retained.count(n)||!tail.retained.count(up)))
                    group.detachable_L+=metric.via_cost_ratio*std::max(metric.gcell_width_dbu,metric.gcell_height_dbu);
            }
        }
        report.groups.push_back(std::move(group));
    }
    std::stable_sort(report.groups.begin(),report.groups.end(),[](const auto& a,const auto& b) {
        if(a.eligible!=b.eligible) return a.eligible>b.eligible;
        if(a.detachable_L!=b.detachable_L) return a.detachable_L>b.detachable_L;
        return a.cell<b.cell;
    });
    report.setup_seconds=std::chrono::duration<double>(Clock::now()-started).count();
    const auto search_start=Clock::now(),deadline=search_start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.seconds));
    for(const auto& group:report.groups) {
        if(!group.eligible||report.evaluated_groups>=options.max_groups||report.moves>=options.max_moves) break;
        if(Clock::now()>=deadline) {report.timed_out=true;break;}
        ++report.evaluated_groups;
        const int old=group.cell,oc=old%grid.cols,orr=old/grid.cols;
        std::vector<Tail> tails;std::vector<NetCellContribution> previous;
        std::vector<int> ac,ar;
        for(size_t i:group.members) {
            tails.push_back(detach(def.nets()[i],guides.at(def.nets()[i].name),old,graph,lef,options.max_tail_nodes));
            if(!tails.back().valid) throw std::logic_error("frozen shared group attachment changed unexpectedly");
            previous.push_back(measured[i].demand);
            for(int n:tails.back().anchor) {ac.push_back(graph.col(n));ar.push_back(graph.row(n));}
        }
        std::sort(ac.begin(),ac.end());std::sort(ar.begin(),ar.end());
        std::set<int> candidates;
        const auto add=[&](int c,int r) {
            if(c<0||r<0||c>=grid.cols||r>=grid.rows||std::abs(c-oc)>options.radius||std::abs(r-orr)>options.radius) return;
            const int cell=r*grid.cols+c;
            if(cell!=old&&!counts[size_t(cell)]&&graph.tsv_landing_legal(c,r,TsvObstacleScope::ALL_LAYERS)) candidates.insert(cell);
        };
        // Nearby cells plus projections toward actual retained-tree anchors.
        for(int dc=-1;dc<=1;++dc) for(int dr=-1;dr<=1;++dr) add(oc+dc,orr+dr);
        const int mc=std::clamp(ac[ac.size()/2],oc-options.radius,oc+options.radius);
        const int mr=std::clamp(ar[ar.size()/2],orr-options.radius,orr+options.radius);
        add(mc,mr);add(mc,orr);add(oc,mr);
        for(int n:{0,int(ac.size()/4),int(ac.size()*3/4),int(ac.size()-1)})
            add(std::clamp(ac[size_t(n)],oc-options.radius,oc+options.radius),std::clamp(ar[size_t(n)],orr-options.radius,orr+options.radius));
        std::vector<int> shortlist(candidates.begin(),candidates.end());
        std::stable_sort(shortlist.begin(),shortlist.end(),[&](int a,int b) {
            const auto distance=[&](int cell) {long long d=0;for(const auto& t:tails) for(int n:t.anchor)
                d+=std::abs(cell%grid.cols-graph.col(n))+std::abs(cell/grid.cols-graph.row(n));return d;};
            const auto da=distance(a),db=distance(b);return da!=db?da<db:a<b;
        });
        if(shortlist.size()>options.max_candidates) shortlist.resize(options.max_candidates);
        // Reconnect the SAME tails at the current hub first. A moved hub must
        // beat this control as well as the original Guide, so merely rerouting
        // a detoured attachment is not credited to hub position.
        shortlist.insert(shortlist.begin(),old);
        SharedHubQuality same_hub=report.after;bool same_hub_valid=false;
        size_t best_trial=std::numeric_limits<size_t>::max();
        std::vector<std::vector<GuideRect>> best_guides;std::vector<NetGuideMetrics> best_metrics;
        NetCellReplacementDelta best_delta;double best_score=report.after.S;
        for(int cell:shortlist) {
            if(Clock::now()>=deadline) {report.timed_out=true;break;}
            SharedHubTrial trial;trial.from=old;trial.to=cell;trial.members=group.members.size();trial.before=report.after;trial.after=report.after;
            std::vector<std::vector<GuideRect>> new_guides;std::vector<NetGuideMetrics> new_metrics;
            std::vector<NetCellContribution> next;double delta_L=0;int64_t delta_vias=0;bool valid=true;
            for(size_t k=0;k<group.members.size();++k) {
                if(Clock::now()>=deadline) {valid=false;report.timed_out=true;trial.reason="deadline";break;}
                const size_t i=group.members[k];const auto& net=def.nets()[i];auto working=tails[k].retained;
                const auto own=graph.own_pin_nodes(net);
                for(int side=0;side<2;++side) {
                    const int root=graph.node(cell%grid.cols,cell/grid.cols,side?11:10);
                    if(!attach(working,root,tails[k].anchor[side],side,own,graph,lef,metric,options.search_nodes,deadline,trial.expansions)) {
                        valid=false;trial.reason="bounded-reattachment-failed";break;
                    }
                }
                if(!valid) break;
                auto encoded=encode(working,graph,lef);std::string error;
                if(!shared_hub_cell_geometry(encoded,cell,graph)) {valid=false;trial.reason="extra-interface-electrical-contact";break;}
                if(!validate_gcell_encoding(net,def.die_area(),grid,lef,encoded,error)||
                   !validate_net_guides(net,def.die_area(),lef,encoded,error,def.gcell_pitch_x(),def.gcell_pitch_y())) {
                    valid=false;trial.reason="connectivity-or-pin-check";break;
                }
                auto m=measure_official_net_guides(net,encoded,graph,lef,metric,references[i]);
                if(m.tsv_connections!=1||m.tsv_cells!=std::vector<int>{cell}) {valid=false;trial.reason="additional-interface-contact";break;}
                const auto original=expand(guides.at(net.name),graph);
                for(int pin:own) if(original.count(pin)!=working.count(pin)) {valid=false;trial.reason="changed-pin-access-cell";break;}
                if(!valid) break;
                delta_L+=m.wirelength-measured[i].wirelength;
                delta_vias+=int64_t(m.ordinary_connections)-int64_t(measured[i].ordinary_connections);
                next.push_back(m.demand);new_guides.push_back(std::move(encoded));new_metrics.push_back(std::move(m));
            }
            report.search_expansions+=trial.expansions;
            if(valid) {
                auto delta=compare_net_cell_replacement(graph,demand,previous,next);
                const double rho=denominator>0?double((long double)report.after.rho+
                    (long double)group.members.size()*(heat[size_t(cell)]-heat[size_t(old)])/denominator):0;
                trial.after=quality(report.after.L+delta_L,std::max(0.,report.after.Q+double(delta.squared_delta)),rho,reference,
                    size_t(int64_t(report.after.ordinary_vias)+delta_vias),size_t(int64_t(report.after.zero_capacity)+delta.zero_capacity_delta),metric);
                if(cell==old) {same_hub=trial.after;same_hub_valid=true;trial.reason="current-hub-reattachment-control";}
                else if(delta_L>=-1e-6) trial.reason="no-wirelength-gain";
                else if(!same_hub_valid||trial.after.L>=same_hub.L-1e-6||trial.after.S<=same_hub.S+1e-9)
                    trial.reason="no-position-gain-over-current-hub-control";
                else if(trial.after.zero_capacity>trial.before.zero_capacity) trial.reason="new-zero-capacity-demand";
                else if(trial.after.S<=best_score+1e-9) trial.reason="full-score-not-better";
                else {
                    trial.reason="feasible-improvement";best_score=trial.after.S;best_trial=report.trials.size();
                    best_guides=std::move(new_guides);best_metrics=std::move(new_metrics);best_delta=std::move(delta);
                }
            } else ++report.candidate_failures;
            report.trials.push_back(std::move(trial));
            if(cell==old&&!valid) break;
        }
        if(best_trial!=std::numeric_limits<size_t>::max()&&!report.timed_out&&Clock::now()<deadline) {
            // All allocations, nonlinear resource pricing, connectivity/PIN and
            // exact overlap/via checks precede this indivisible group commit.
            if(options.before_commit) options.before_commit();
            if(Clock::now()>=deadline) {report.timed_out=true;break;}
            for(size_t k=0;k<group.members.size();++k) {
                const size_t i=group.members[k];guides.at(def.nets()[i].name).swap(best_guides[k]);
                measured[i]=std::move(best_metrics[k]);
            }
            for(auto [node,delta]:best_delta.demand_delta) demand[size_t(node)]+=delta;
            auto& trial=report.trials[best_trial];trial.accepted=true;trial.reason="accepted";
            counts[size_t(old)]-=int(group.members.size());counts[size_t(trial.to)]+=int(group.members.size());
            report.after=trial.after;++report.moves;
        }
    }
    report.search_seconds=std::chrono::duration<double>(Clock::now()-search_start).count();
    const auto exact=aggregate_official_metrics(pointers,demand,graph,thermal,metric,false);
    if(!near(exact.guide.wirelength,report.after.L)||!near(exact.squared_normalized_overflow,report.after.Q)||
       !near(exact.guide.correlation,report.after.rho)||!near(100*exact.guide.quality,report.after.S))
        throw std::logic_error("shared hub incremental/full aggregate mismatch");
    return report;
}

void write_shared_hub_report(std::ostream& out,const SharedHubReport& report,const DefParser& def) {
    out<<std::setprecision(17)<<"{\"official_equivalence\":false,\"group_members_frozen\":true,\"before\":";write_quality(out,report.before);
    out<<",\"after\":";write_quality(out,report.after);
    out<<",\"evaluated_groups\":"<<report.evaluated_groups<<",\"moves\":"<<report.moves<<",\"candidate_failures\":"<<report.candidate_failures
       <<",\"search_expansions\":"<<report.search_expansions<<",\"timed_out\":"<<(report.timed_out?"true":"false")
       <<",\"setup_seconds\":"<<report.setup_seconds<<",\"search_seconds\":"<<report.search_seconds<<",\"groups\":[";
    bool first=true;
    for(const auto& g:report.groups) {
        if(!first) out<<',';first=false;
        out<<"{\"cell\":"<<g.cell<<",\"eligible\":"<<(g.eligible?"true":"false")<<",\"reason\":"<<std::quoted(g.reason)
           <<",\"detachable_L\":"<<g.detachable_L<<",\"detached_nodes\":"<<g.detached_nodes<<",\"members\":[";
        bool member_first=true;for(size_t i:g.members) {if(!member_first) out<<',';member_first=false;out<<std::quoted(def.nets()[i].name);}
        out<<"]}";
    }
    out<<"],\"trials\":[";first=true;
    for(const auto& t:report.trials) {
        if(!first) out<<',';first=false;
        out<<"{\"from\":"<<t.from<<",\"to\":"<<t.to<<",\"members\":"<<t.members<<",\"expansions\":"<<t.expansions
           <<",\"accepted\":"<<(t.accepted?"true":"false")<<",\"reason\":"<<std::quoted(t.reason)<<",\"before\":";
        write_quality(out,t.before);out<<",\"after\":";write_quality(out,t.after);out<<'}';
    }
    out<<"]}\n";
}
} // namespace eda
