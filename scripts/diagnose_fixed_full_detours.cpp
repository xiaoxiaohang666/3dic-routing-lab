// Read-only: how much of a saved Guide's length is detour around GCells whose
// fixed PIN/OBS tracks alone already reach capacity, and what fills those cells.
//
// For an evenly spaced sample of nets per class (same/cross die, pin count), take
// one net out and reroute it with the large-budget maze search (cross-die nets keep
// their current TSV cell) under three costs:
//   maze_large                - all other routes present (the router's own view);
//   maze_large_fixed_walls    - other routes and TSVs hidden, so only cells that
//                               fixed PIN/OBS tracks fill still cost anything;
//   maze_large_no_congestion  - no congestion cost at all.
// The gap between the last two is the detour forced by fixed-full cells. Every
// fixed-full cell that the no-congestion route enters is then broken down by the
// fixed shapes that fill it (macro OBS, PIN of a routed net, macro PIN that no DEF
// net uses, top-level IO PIN, and the one-track fallback of a PIN clip that holds no
// track) using the same track rules as the observed-20261104 fixed model, and it is
// looked up in official overflow lists. A listed cell had official demand above
// capacity with a route in it; the model matched those lists cell by cell, so a
// listed fixed-full cell is one the official scorer also counts as full.
//
// Nothing is committed and no Guide is written. All numbers are one-net-at-a-time
// offline estimates, not achievable totals, and not official results.
#include "objectives/net_cell_contribution.hpp"
#include "objectives/official_metrics.hpp"
#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
using namespace eda;
namespace fs=std::filesystem;
namespace {
std::string net_class(const Net& net) {
    const size_t p=net.pins.size();
    const char* bucket=p<=2?"2":p<=4?"3-4":p<=8?"5-8":p<=16?"9-16":p<=32?"17-32":"33+";
    return std::string(net.is_cross_die?"cross:":"same:")+bucket;
}
struct Mode {std::string name;size_t expansions;double seconds;double present;bool ignore_routes;};

// Same cell coverage as compat::Resources: positive-area shapes use the cells whose
// interiors they intersect; the last cell is closed.
std::pair<int,int> covered_cells(double low,double high,const std::vector<double>& lines) {
    if(high<low) std::swap(low,high);
    if(high<lines.front()||low>lines.back()) return {0,0};
    const int count=int(lines.size())-1;
    if(low==high) {
        const int cell=std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,count-1);
        return {cell,cell+1};
    }
    if(high<=lines.front()||low>=lines.back()) return {0,0};
    const int begin=std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,count);
    const int end=std::clamp(int(std::lower_bound(lines.begin(),lines.end(),high)-lines.begin()),0,count);
    return {begin,end};
}
template<class Visitor> void visit_cells(const Rect& rect,const GCellGrid& grid,Visitor&& visitor) {
    const auto [cb,ce]=covered_cells(rect.x1,rect.x2,grid.x_lines);
    const auto [rb,re]=covered_cells(rect.y1,rect.y2,grid.y_lines);
    for(int r=rb;r<re;++r) for(int c=cb;c<ce;++c) visitor(c,r);
}

enum Source {OBS=0,SIGNAL_PIN=1,UNUSED_PIN=2,IO_PIN=3};
const char* source_name[]={"obs","signal_pin","unused_pin","io_pin"};
struct Span {uint32_t begin,end;int source;bool fallback;};
template<class Keep> size_t union_count(const std::vector<Span>& spans,Keep keep) {
    std::vector<std::pair<uint32_t,uint32_t>> v;
    for(const auto& s:spans) if(keep(s)) v.push_back({s.begin,s.end});
    if(v.empty()) return 0;
    std::sort(v.begin(),v.end());
    size_t count=0;auto [b,e]=v.front();
    for(size_t i=1;i<v.size();++i) {
        if(v[i].first<=e) e=std::max(e,v[i].second);
        else {count+=e-b;b=v[i].first;e=v[i].second;}
    }
    return count+(e-b);
}
struct CellInfo {
    int nets_no_congestion{0},nets_current{0},nets_fixed_walls{0},own_pin_nets{0};
    double attributed_dl{0},weighted_entries{0}; // scaled by nets per sampled net
    std::vector<Span> spans;
    std::vector<std::string> examples; // a few fixed shape owners, for inspection
    int official_lists{0},official_demand{-1},official_capacity{-1};
};

std::vector<std::string> split(const std::string& s,char c) {
    std::vector<std::string> out;std::string w;std::istringstream in(s);
    while(std::getline(in,w,c)) out.push_back(w);
    return out;
}
}

namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,
                  const std::map<std::string,std::vector<GuideRect>>& guides_in,size_t per_class,
                  const std::vector<std::string>& official_lists,const fs::path& out) {
    auto& g=router.graph_;const auto& mo=router.options_.official_metrics;
    const auto& grid=g.grid();
    router.initial_build_=false;router.history_scale_=0;
    const size_t nets=def.nets().size();
    std::vector<NetGuideMetrics> measured;measured.reserve(nets);
    long double total_l=0;
    for(size_t i=0;i<nets;++i) {
        const auto& net=def.nets()[i];
        measured.push_back(measure_official_net_guides(net,guides_in.at(net.name),g,lef,mo,0));
        total_l+=measured.back().wirelength;
        for(auto [n,d]:measured.back().demand) router.demand_[size_t(n)]+=d;
    }
    router.routes_.assign(nets,NetRoute{});
    const auto cq=[&](int n,int d){return (long double)official_resource_cell_contribution(g,n,d).squared;};
    long double total_q=0;for(size_t n=0;n<g.size();++n) total_q+=cq(int(n),router.demand_[n]);
    const long double reference=router.full_reference_;
    const auto score=[&](long double l,long double q){return .45*100*std::min(1.0L,reference/l)+.30*100/std::sqrt(1+std::max(0.0L,q));};
    const double s0=double(score(total_l,total_q));
    const auto fixed_full=[&](int n){return g.official_capacity(n)>0&&double(g.official_obstacle_demand(n))>=g.official_capacity(n);};
    // Whole-design count of fixed-full cells per layer, for scale.
    std::array<size_t,21> full_by_layer{},cells_by_layer{};
    for(size_t n=0;n<g.size();++n) {
        const int l=g.layer(int(n));++cells_by_layer[size_t(l)];
        if(fixed_full(int(n))) ++full_by_layer[size_t(l)];
    }
    std::unordered_map<int,int> cell_slot;
    const auto& slots=router.policy_.slots();
    for(size_t s=0;s<slots.size();++s) {
        const int c=grid.get_col(slots[s].point.x),r=grid.get_row(slots[s].point.y);
        if(c>=0&&r>=0) cell_slot.emplace(r*grid.cols+c,int(s));
    }
    std::map<std::string,std::vector<size_t>> classes;
    for(size_t i=0;i<nets;++i) classes[net_class(def.nets()[i])].push_back(i);
    std::vector<size_t> sample;std::unordered_map<size_t,double> weight; // nets each sampled net stands for
    for(auto& [name,members]:classes) {
        const size_t take=std::min(per_class,members.size());
        for(size_t k=0;k<take;++k) {
            const size_t i=members[(2*k+1)*members.size()/(2*take)];
            sample.push_back(i);weight[i]=double(members.size())/double(take);
        }
    }
    std::cout<<"replay Q="<<double(total_q)<<" L="<<double(total_l)<<" R="<<double(reference)<<" S_WC="<<s0
             <<" sample="<<sample.size()<<'\n'<<std::flush;
    const std::vector<Mode> modes{
        {"maze_large",1000000,5,1,false},
        {"maze_large_fixed_walls",1000000,5,1,true},
        {"maze_large_no_congestion",1000000,5,0,false}};
    std::vector<int> no_routes(router.demand_.size(),0);
    std::unordered_map<int,CellInfo> cells;
    std::ofstream csv(out/"replay.csv");csv<<std::setprecision(12);
    csv<<"net,class,pins,mode,status,old_L,dL,d_planar,d_via,dQ,dS,old_over_cells,new_over_cells,seconds,"
         "fixed_full_cells,zero_capacity_cells,old_fixed_full_cells\n";
    size_t done=0;const auto started=std::chrono::steady_clock::now();
    for(size_t i:sample) {
        const auto& net=def.nets()[i];auto& m=measured[i];
        int slot=-1;
        if(net.is_cross_die) {
            const auto found=m.tsv_cells.size()==1?cell_slot.find(m.tsv_cells[0]):cell_slot.end();
            if(found==cell_slot.end()) {csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<",all,no_slot,,,,,,,,,,,,\n";continue;}
            slot=found->second;
        }
        // Cells holding one of this net's own pin shapes on that shape's layer.
        std::unordered_set<int> own_pins;
        for(const auto& pin:net.pins) for(const auto& shape:pin.port_rects)
            if(shape.layer_idx>=1&&shape.layer_idx<=20)
                visit_cells(shape.rect,grid,[&](int c,int r){const int n=g.node(c,r,shape.layer_idx);if(n>=0) own_pins.insert(n);});
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]-=d;
        router.official_wirelength_=total_l-m.wirelength;
        long double q_without=total_q;for(auto [n,d]:m.demand) q_without+=cq(n,router.demand_[n])-cq(n,router.demand_[n]+d);
        router.official_squared_overflow_=q_without;
        const auto over_cells=[&](const std::vector<std::pair<int,int>>& demand){int c=0;for(auto [n,d]:demand) if(d>0&&double(g.official_obstacle_demand(n))+router.demand_[n]+d>g.official_capacity(n)) ++c;return c;};
        const auto full_cells=[&](const std::vector<std::pair<int,int>>& demand){std::vector<int> v;for(auto [n,d]:demand) if(d>0&&fixed_full(n)) v.push_back(n);return v;};
        const auto zero_cells=[&](const std::vector<std::pair<int,int>>& demand){int c=0;for(auto [n,d]:demand) if(d>0&&g.official_capacity(n)<=0) ++c;return c;};
        const int old_over=over_cells(m.demand);
        const auto old_full=full_cells(m.demand);
        for(int n:old_full) ++cells[n].nets_current;
        double l_fixed_walls=NAN;
        for(const auto& mode:modes) {
            router.present_=mode.present;router.pricing_generation_=std::numeric_limits<uint64_t>::max();++router.state_generation_;
            router.counters_.v7_topology_seconds=0;
            csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<','<<mode.name;
            const auto t0=std::chrono::steady_clock::now();
            try {
                router.v7_refinement_active_=true;
                router.options_.v7_rrr_astar_expansions=mode.expansions;
                router.options_.search_limit=std::max<size_t>(250000,mode.expansions);
                router.v7_refinement_expansion_start_=router.counters_.expansions;
                router.v7_refinement_deadline_=t0+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(mode.seconds));
                if(mode.ignore_routes) std::swap(router.demand_,no_routes);
                NetRoute trial;
                try {trial=router.build_route(i,slot);}
                catch(...) {if(mode.ignore_routes) std::swap(router.demand_,no_routes);throw;}
                if(mode.ignore_routes) std::swap(router.demand_,no_routes);
                router.v7_refinement_active_=false;
                const double secs=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
                if(net.is_cross_die && trial.measured.tsv_cells!=m.tsv_cells) throw std::runtime_error("TSV cell changed");
                long double q=q_without;for(auto [n,d]:trial.measured.demand) q+=cq(n,router.demand_[n]+d)-cq(n,router.demand_[n]);
                const long double l=total_l-m.wirelength+trial.measured.wirelength;
                const auto full=full_cells(trial.measured.demand);
                csv<<",ok,"<<m.wirelength<<','<<trial.measured.wirelength-m.wirelength<<','
                   <<trial.measured.planar_wirelength-m.planar_wirelength<<','<<trial.measured.via_wirelength-m.via_wirelength<<','
                   <<double(q-total_q)<<','<<double(score(l,q))-s0<<','<<old_over<<','<<over_cells(trial.measured.demand)<<','<<secs<<','
                   <<full.size()<<','<<zero_cells(trial.measured.demand)<<','<<old_full.size()<<'\n';
                if(mode.name=="maze_large_fixed_walls") {
                    l_fixed_walls=trial.measured.wirelength;
                    for(int n:full) ++cells[n].nets_fixed_walls;
                }
                if(mode.name=="maze_large_no_congestion") {
                    // The fixed-walls minus no-congestion gap is what fixed-full cells
                    // cost this net; split it evenly over the fixed-full cells entered.
                    const double gap=std::isfinite(l_fixed_walls)?std::max(0.0,l_fixed_walls-trial.measured.wirelength):0.0;
                    for(int n:full) {
                        auto& c=cells[n];++c.nets_no_congestion;
                        c.attributed_dl+=weight.at(i)*gap/double(full.size());c.weighted_entries+=weight.at(i);
                        if(own_pins.count(n)) ++c.own_pin_nets;
                    }
                }
            } catch(const std::exception& e) {
                router.v7_refinement_active_=false;
                std::string w=e.what();std::replace(w.begin(),w.end(),',',';');std::replace(w.begin(),w.end(),'\n',' ');
                csv<<','<<w.substr(0,160)<<','<<m.wirelength<<",,,,,,"<<old_over<<",,"
                   <<std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count()<<",,,\n";
            }
        }
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]+=d;
        if(++done%200==0) std::cout<<"[REPLAY] "<<done<<'/'<<sample.size()<<" seconds="
            <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<'\n'<<std::flush;
    }
    std::cout<<"[REPLAY] done cells="<<cells.size()<<'\n'<<std::flush;

    // ---- What fills each fixed-full cell that a sampled route entered ----
    std::array<std::vector<double>,21> tracks;std::array<Direction,21> dirs{};
    for(int l=1;l<=20;++l) dirs[size_t(l)]=lef.get_layer_direction(get_layer_name(l));
    for(const auto& p:def.tracks()) for(const auto& name:p.layers) {
        const int l=get_layer_index(name);
        if(!l||dirs[size_t(l)]==Direction::UNKNOWN) continue;
        const bool h=dirs[size_t(l)]==Direction::HORIZONTAL;
        if(p.axis!=(h?'Y':'X')) continue;
        const auto& lines=h?grid.y_lines:grid.x_lines;
        for(int k=0;k<p.count;++k) {
            const double x=p.start+k*p.step;
            if(x>=lines.front()&&x<=lines.back()) tracks[size_t(l)].push_back(x);
        }
    }
    for(auto& v:tracks) {std::sort(v.begin(),v.end());v.erase(std::unique(v.begin(),v.end()),v.end());}
    const auto half_open=[&](const Rect& rect,int l,int col,int row)->std::pair<uint32_t,uint32_t> {
        const auto& v=tracks[size_t(l)];const bool h=dirs[size_t(l)]==Direction::HORIZONTAL;
        const auto& lines=h?grid.y_lines:grid.x_lines;const size_t band=size_t(h?row:col);
        const double low=std::max(h?rect.y1:rect.x1,lines[band]),high=std::min(h?rect.y2:rect.x2,lines[band+1]);
        const auto b=std::lower_bound(v.begin(),v.end(),low),e=std::lower_bound(v.begin(),v.end(),high);
        if(b>=e) return {0,0};
        return {uint32_t(b-v.begin()),uint32_t(e-v.begin())};
    };
    const auto previous=[&](const Rect& rect,int l,int col,int row)->std::pair<uint32_t,uint32_t> {
        const auto& v=tracks[size_t(l)];const bool h=dirs[size_t(l)]==Direction::HORIZONTAL;
        const auto& lines=h?grid.y_lines:grid.x_lines;const size_t band=size_t(h?row:col);
        const double low=std::max(h?rect.y1:rect.x1,lines[band]);
        const size_t next=size_t(std::lower_bound(v.begin(),v.end(),low)-v.begin());
        const uint32_t idx=uint32_t(next>0?next-1:0);
        return {idx,idx+1};
    };
    const auto add_shape=[&](const Rect& rect,int l,int source,const std::string& owner) {
        if(l<1||l>20||tracks[size_t(l)].empty()) return;
        visit_cells(rect,grid,[&](int c,int r) {
            const int n=g.node(c,r,l);if(n<0) return;
            const auto found=cells.find(n);if(found==cells.end()) return;
            auto span=half_open(rect,l,c,r);bool fallback=false;
            if(span.first>=span.second) {
                if(source==OBS) return; // OBS has no fallback track.
                span=previous(rect,l,c,r);fallback=true;
            }
            found->second.spans.push_back({span.first,span.second,source,fallback});
            if(found->second.examples.size()<4) found->second.examples.push_back(std::string(source_name[source])+":"+owner);
        });
    };
    std::unordered_set<std::string> connected;
    for(const auto& net:def.nets()) for(const auto& pin:net.pins) if(!pin.instance_name.empty()) connected.insert(pin.name);
    for(const auto& [cname,comp]:def.components()) {
        const auto* macro=lef.get_macro(comp.macro_type);if(!macro) throw std::runtime_error("missing macro");
        for(const auto& shape:macro->obstructions)
            add_shape(transform_macro_rect(shape.rect,*macro,comp,def.dbu_per_micron()),get_layer_index(shape.layer),OBS,comp.macro_type);
        for(const auto& [pname,pin]:macro->pins) {
            const int source=connected.count(cname+"."+pname)?SIGNAL_PIN:UNUSED_PIN;
            for(const auto& shape:pin.port_rects)
                add_shape(transform_macro_rect(shape.rect,*macro,comp,def.dbu_per_micron()),get_layer_index(shape.layer),source,comp.macro_type+"/"+pname);
        }
    }
    for(const auto& [name,pin]:def.io_pins()) for(const auto& shape:pin.port_rects) add_shape(shape.rect,shape.layer_idx,IO_PIN,name);

    // ---- Official overflow lists ----
    std::vector<std::pair<std::string,size_t>> list_sizes;
    size_t official_capacity_mismatch=0;
    for(const auto& path:official_lists) {
        std::ifstream in(path);if(!in) throw std::runtime_error("missing official list "+path);
        std::string line;std::getline(in,line);
        const auto head=split(line,',');std::map<std::string,size_t> col;
        for(size_t k=0;k<head.size();++k) col[head[k]]=k;
        size_t rows=0;
        while(std::getline(in,line)) {
            if(line.empty()) continue;
            const auto v=split(line,',');++rows;
            const int n=g.node(std::stoi(v.at(col.at("gcell_col"))),std::stoi(v.at(col.at("gcell_row"))),get_layer_index(v.at(col.at("layer"))));
            if(n<0) throw std::runtime_error("official cell outside grid");
            const auto found=cells.find(n);if(found==cells.end()) continue;
            auto& c=found->second;++c.official_lists;
            c.official_demand=std::max(c.official_demand,std::stoi(v.at(col.at("demand"))));
            c.official_capacity=std::stoi(v.at(col.at("capacity")));
            if(double(c.official_capacity)!=g.official_capacity(n)) ++official_capacity_mismatch;
        }
        list_sizes.push_back({fs::path(path).parent_path().parent_path().parent_path().filename().string()+"/"+fs::path(path).filename().string(),rows});
    }

    std::ofstream cc(out/"fixed_full_cells.csv");cc<<std::setprecision(12);
    cc<<"col,row,layer,capacity,fixed,model_check,nets_no_congestion,nets_fixed_walls,nets_current,own_pin_nets,attributed_dL,weighted_entries,"
        "obs_tracks,signal_pin_tracks,unused_pin_tracks,io_pin_tracks,fallback_shapes,pin_tracks,"
        "full_without_obs,full_without_unused_pins,full_without_fallback,full_without_signal_pins,"
        "official_lists,official_demand,official_capacity,examples\n";
    size_t model_mismatch=0;
    for(const auto& [n,c]:cells) {
        const int cap=int(g.official_capacity(n));const int fixed=int(g.official_obstacle_demand(n));
        const auto all=union_count(c.spans,[](const Span&){return true;});
        const bool check=int(std::min<size_t>(all,size_t(cap)))==fixed;
        if(!check) ++model_mismatch;
        const auto by=[&](int s){return union_count(c.spans,[&](const Span& x){return x.source==s;});};
        const auto pins=union_count(c.spans,[](const Span& x){return x.source!=OBS;});
        const auto without_obs=pins;
        const auto without_unused=union_count(c.spans,[](const Span& x){return x.source!=UNUSED_PIN;});
        const auto without_fallback=union_count(c.spans,[](const Span& x){return !x.fallback;});
        const auto without_signal=union_count(c.spans,[](const Span& x){return x.source!=SIGNAL_PIN;});
        size_t fallbacks=0;for(const auto& s:c.spans) fallbacks+=s.fallback;
        std::string ex;for(const auto& e:c.examples) ex+=(ex.empty()?"":"|")+e;
        std::replace(ex.begin(),ex.end(),',',';');
        cc<<g.col(n)<<','<<g.row(n)<<','<<g.layer(n)<<','<<cap<<','<<fixed<<','<<check<<','
          <<c.nets_no_congestion<<','<<c.nets_fixed_walls<<','<<c.nets_current<<','<<c.own_pin_nets<<','<<c.attributed_dl<<','<<c.weighted_entries<<','
          <<by(OBS)<<','<<by(SIGNAL_PIN)<<','<<by(UNUSED_PIN)<<','<<by(IO_PIN)<<','<<fallbacks<<','<<pins<<','
          <<(int(without_obs)>=cap)<<','<<(int(without_unused)>=cap)<<','<<(int(without_fallback)>=cap)<<','<<(int(without_signal)>=cap)<<','
          <<c.official_lists<<','<<c.official_demand<<','<<c.official_capacity<<','<<ex<<'\n';
    }
    std::ofstream s(out/"classes.json");s<<std::setprecision(15);
    s<<"{\"scope\":\"one-net-at-a-time replay on a saved Guide; nothing committed; offline estimate\",\"L\":"<<double(total_l)
     <<",\"reference\":"<<double(reference)<<",\"Q\":"<<double(total_q)<<",\"model_mismatch_cells\":"<<model_mismatch
     <<",\"official_capacity_mismatch\":"<<official_capacity_mismatch<<",\"official_lists\":[";
    for(size_t k=0;k<list_sizes.size();++k) s<<(k?",":"")<<"{\"name\":\""<<list_sizes[k].first<<"\",\"rows\":"<<list_sizes[k].second<<'}';
    s<<"],\"fixed_full_by_layer\":{";
    bool first=true;
    for(int l=1;l<=20;++l) {s<<(first?"":",")<<'"'<<l<<"\":{\"full\":"<<full_by_layer[size_t(l)]<<",\"cells\":"<<cells_by_layer[size_t(l)]<<'}';first=false;}
    s<<"},\"classes\":{";
    first=true;
    for(const auto& [name,members]:classes) {
        long double l=0;for(size_t i:members) l+=measured[i].wirelength;
        s<<(first?"":",")<<'"'<<name<<"\":{\"nets\":"<<members.size()<<",\"L\":"<<double(l)<<'}';first=false;
    }
    s<<"}}\n";
  }
};
}
int main(int argc,char** argv) {
  try {
    std::map<std::string,std::string> a;for(int i=1;i+1<argc;i+=2) a[argv[i]]=argv[i+1];
    fs::path out=a.at("--output-dir");if(fs::exists(out)) throw std::runtime_error("output exists");fs::create_directories(out);
    LefParser lef;if(!lef.parse(a.at("--lef"))) throw std::runtime_error("LEF");
    DefParser def;if(!def.parse(a.at("--def"),lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF");
    ThermalParser thermal;
    RouterOptions o;o.official_model=true;o.official_runtime=a.at("--runtime");
    o.graph_options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;o.graph_options.allow_nonpreferred=false;
    o.graph_options.score_20261020=true;o.graph_options.documented_rules=true;o.graph_options.pin_zero_track_floor=true;
    o.graph_options.observed_fixed_20261104=true;o.graph_options.congestion_overflow_scale=1.0;
    o.official_metrics.score_20261020=true;o.official_metrics.a150_rules=true;o.official_metrics.tsv_track_demand=true;
    o.official_metrics.all_adjacent_overlaps=true;o.official_metrics.snap_reference_coordinates=true;
    o.official_metrics.thermal_gain=std::stod(a.at("--thermal-gain"));o.official_metrics.congestion_target_q=30;
    o.official_metrics.gcell_width_dbu=def.gcell_pitch_x();o.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
    o.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;o.tsv_policy.tsv_obstacle_scope=TsvObstacleScope::INTERFACE;o.official_metrics.tsv_congestion_ratio=1;
    o.fast_initial=true;o.fast_quality=true;o.v7_cell_cost=true;o.official_score_pricing=true;o.climb_escape_radius=std::stoi(a.at("--radius"));
    o.v7_topology=true;o.time_limit=40000;
    std::vector<std::string> lists;
    if(a.count("--official-overflow")) for(const auto& p:split(a.at("--official-overflow"),',')) if(!p.empty()) lists.push_back(p);
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,std::stoul(a.at("--per-class")),lists,out);
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
