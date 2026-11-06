// Read-only: how far each die's tree of a saved Guide is from a FLUTE tree on
// the same points.
//
// A cross-die net must be connected as two trees, one per die, joined at its one
// TSV. For every net and every die it touches, this tool compares
//   * the Guide's own wire length on that die: per layer, the deduplicated
//     covered GCells minus the number of straight runs, times the preferred
//     direction GCell length (a one-cell rectangle has zero length), summed over
//     the die's layers; with
//   * the native official FLUTE tree length on that die's pin points plus the
//     Guide's TSV point, all snapped to their GCell centres (the Guide's own
//     resolution).
// For cross-die nets it also writes the single FLUTE tree on all snapped pins of
// both dies, i.e. how much longer two trees must be than one on the same pins.
// The scorer's planar charge is cells times GCell length; cells minus runs is
// the part of that charge a shorter tree can remove, runs are the per-segment
// end cells, written separately. Vias are counted per die exactly as the scorer
// counts them (adjacent-layer positive overlaps), the M10/M11 TSV excluded.
//
// FLUTE ignores blockages and congestion, so the gap is an upper bound on what a
// better tree could save, not a reachable result. Nothing is routed, committed
// or written back; offline diagnosis only.
#include "objectives/official_metrics.hpp"
#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <unordered_set>
using namespace eda;
namespace fs=std::filesystem;
namespace {
std::string net_class(const Net& net) {
    const size_t p=net.pins.size();
    const char* bucket=p<=2?"2":p<=4?"3-4":p<=8?"5-8":p<=16?"9-16":p<=32?"17-32":p<=150?"33-150":"151+";
    return std::string(net.is_cross_die?"cross:":"same:")+bucket;
}
// Cells whose interiors a positive-area rectangle intersects (same rule as d08).
std::pair<int,int> span_cells(double low,double high,const std::vector<double>& lines) {
    if(high<low) std::swap(low,high);
    if(high<=lines.front()||low>=lines.back()) return {0,0};
    const int count=int(lines.size())-1;
    const int b=std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,count);
    const int e=std::clamp(int(std::lower_bound(lines.begin(),lines.end(),high)-lines.begin()),0,count);
    return {b,e};
}
bool positive_overlap(const GuideRect& a,const GuideRect& b) {
    return std::min(a.x2,b.x2)>std::max(a.x1,b.x1)&&std::min(a.y2,b.y2)>std::max(a.y1,b.y1);
}
int cell_index(double v,const std::vector<double>& lines) {
    const int count=int(lines.size())-1;
    return std::clamp(int(std::upper_bound(lines.begin(),lines.end(),v)-lines.begin())-1,0,count-1);
}
struct DieStats {
    size_t pins=0,cells=0,runs=0,vias=0,points=0;
    long double edge=0,planar=0,flute=0;bool skipped=false;
};
}
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,
                  const std::map<std::string,std::vector<GuideRect>>& guides,const fs::path& out) {
    const auto& g=router.graph_;const auto& mo=router.options_.official_metrics;const auto& grid=g.grid();
    const auto centre=[&](const Point& p) {
        const int c=cell_index(p.x,grid.x_lines),r=cell_index(p.y,grid.y_lines);
        return Point{std::floor((grid.x_lines[size_t(c)]+grid.x_lines[size_t(c)+1])*0.5),
                     std::floor((grid.y_lines[size_t(r)]+grid.y_lines[size_t(r)+1])*0.5)};
    };
    const auto flute=[&](std::vector<Point> pts,bool& skipped) -> long double {
        std::sort(pts.begin(),pts.end(),[](const Point& a,const Point& b){return a.x<b.x||(a.x==b.x&&a.y<b.y);});
        pts.erase(std::unique(pts.begin(),pts.end(),[](const Point& a,const Point& b){return a.x==b.x&&a.y==b.y;}),pts.end());
        if(pts.size()<=1) {skipped=false;return 0;}
        const auto f=router.flute_->evaluate_points(pts);skipped=f.skipped_degree_limit;
        return f.skipped_degree_limit?0:f.wirelength_dbu;
    };
    std::ofstream csv(out/"nets.csv");csv<<std::setprecision(12);
    csv<<"net,class,pins,cross,L,planar,via_L,tsv_L,ref,tsv_col,tsv_row,flute_one_tree,one_tree_skipped";
    for(const char* d:{"b","t"}) csv<<','<<d<<"_pins,"<<d<<"_points,"<<d<<"_cells,"<<d<<"_runs,"<<d<<"_edge,"
        <<d<<"_planar,"<<d<<"_vias,"<<d<<"_flute,"<<d<<"_skipped";
    csv<<'\n';
    long double total_l=0,total_ref=0,check_planar=0,measured_planar=0;size_t check_vias=0,measured_vias=0,tsv_bad=0;
    for(size_t i=0;i<def.nets().size();++i) {
        const auto& net=def.nets()[i];
        const auto found=guides.find(net.name);if(found==guides.end()) throw std::runtime_error("net without Guide: "+net.name);
        const auto m=measure_official_net_guides(net,found->second,g,lef,mo,router.reference_lengths_[i]);
        total_l+=m.wirelength;total_ref+=router.reference_lengths_[i];measured_planar+=m.planar_wirelength;
        measured_vias+=m.ordinary_connections;
        std::array<std::vector<const GuideRect*>,21> by_layer;
        for(const auto& r:found->second) {const int l=get_layer_index(r.layer);if(l>=1&&l<=20) by_layer[size_t(l)].push_back(&r);}
        std::array<DieStats,2> die{};
        for(int l=1;l<=20;++l) {
            auto& d=die[l>10?1:0];
            if(!by_layer[size_t(l)].empty()) {
                const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
                std::unordered_set<long long> cells;
                for(const auto* r:by_layer[size_t(l)]) {
                    const auto [cb,ce]=span_cells(r->x1,r->x2,grid.x_lines);const auto [rb,re]=span_cells(r->y1,r->y2,grid.y_lines);
                    for(int rr=rb;rr<re;++rr) for(int cc=cb;cc<ce;++cc) cells.insert((long long)rr*grid.cols+cc);
                }
                // A run starts at every cell whose predecessor along the preferred direction is absent.
                size_t runs=0;
                for(long long c:cells) {
                    const long long rr=c/grid.cols,cc=c%grid.cols;
                    const bool has_prev=h?(cc>0&&cells.count(c-1)):(rr>0&&cells.count(c-grid.cols));
                    if(!has_prev) ++runs;
                }
                const double step=h?mo.gcell_width_dbu:mo.gcell_height_dbu;
                d.cells+=cells.size();d.runs+=runs;
                d.planar+=(long double)cells.size()*step;d.edge+=(long double)(cells.size()-runs)*step;
                check_planar+=(long double)cells.size()*step;
            }
            if(l<20&&l!=10) for(const auto* a:by_layer[size_t(l)]) for(const auto* b:by_layer[size_t(l+1)])
                if(positive_overlap(*a,*b)) {++d.vias;++check_vias;}
        }
        // Same pin points as the official reference, then snapped to GCell centres.
        std::array<std::vector<Point>,2> points;std::vector<Point> all;std::set<std::string> names;
        for(const auto& pin:net.pins) {
            const int side=pin.layer_idx>10?1:0;++die[size_t(side)].pins;
            if(!names.insert(pin.name).second) continue;
            const Point p=centre(pin.port_rects.empty()?pin.coord:pin.port_rects.front().rect.center());
            points[size_t(side)].push_back(p);all.push_back(p);
        }
        int tsv_col=-1,tsv_row=-1;long double one_tree=0;bool one_skipped=false;
        if(net.is_cross_die) {
            if(m.tsv_points.size()!=1) {++tsv_bad;}
            else {
                const Point t=centre(m.tsv_points.front());
                tsv_col=cell_index(t.x,grid.x_lines);tsv_row=cell_index(t.y,grid.y_lines);
                for(auto& p:points) p.push_back(t);
            }
            one_tree=flute(all,one_skipped);
        }
        for(int s=0;s<2;++s) {
            if(points[size_t(s)].empty()) continue;
            die[size_t(s)].points=points[size_t(s)].size();
            die[size_t(s)].flute=flute(points[size_t(s)],die[size_t(s)].skipped);
        }
        csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<','<<net.is_cross_die<<','<<m.wirelength<<','
           <<m.planar_wirelength<<','<<m.via_wirelength<<','<<m.tsv_wirelength<<','<<router.reference_lengths_[i]<<','
           <<tsv_col<<','<<tsv_row<<','<<double(one_tree)<<','<<one_skipped;
        for(const auto& d:die) csv<<','<<d.pins<<','<<d.points<<','<<d.cells<<','<<d.runs<<','<<double(d.edge)<<','
            <<double(d.planar)<<','<<d.vias<<','<<double(d.flute)<<','<<d.skipped;
        csv<<'\n';
    }
    std::ofstream s(out/"totals.json");s<<std::setprecision(15);
    s<<"{\"scope\":\"read-only per-die comparison of a saved Guide's wire length with a FLUTE tree on the same snapped points; offline\","
     <<"\"L\":"<<double(total_l)<<",\"reference\":"<<double(total_ref)
     <<",\"gcell_width\":"<<mo.gcell_width_dbu<<",\"gcell_height\":"<<mo.gcell_height_dbu
     <<",\"planar_check\":{\"recomputed\":"<<double(check_planar)<<",\"measured\":"<<double(measured_planar)<<'}'
     <<",\"via_check\":{\"recomputed\":"<<check_vias<<",\"measured\":"<<measured_vias<<'}'
     <<",\"cross_nets_without_one_tsv\":"<<tsv_bad<<"}\n";
    std::cout<<"done L="<<double(total_l)<<" R="<<double(total_ref)<<" planar "<<double(check_planar)<<"/"<<double(measured_planar)
             <<" vias "<<check_vias<<"/"<<measured_vias<<" tsv_bad="<<tsv_bad<<'\n';
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
    // Same router construction as eda_diagnose_reference_gap (d08); only its
    // graph, metric options and cached references are used.
    RouterOptions o;o.official_model=true;o.official_runtime=a.at("--runtime");
    o.graph_options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;o.graph_options.allow_nonpreferred=false;
    o.graph_options.score_20261020=true;o.graph_options.documented_rules=true;o.graph_options.pin_zero_track_floor=true;
    o.graph_options.observed_fixed_20261104=true;o.graph_options.congestion_overflow_scale=1.0;
    o.official_metrics.score_20261020=true;o.official_metrics.a150_rules=true;o.official_metrics.tsv_track_demand=true;
    o.official_metrics.all_adjacent_overlaps=true;o.official_metrics.snap_reference_coordinates=true;
    o.official_metrics.thermal_gain=std::stod(a.at("--thermal-gain"));o.official_metrics.congestion_target_q=30;
    o.official_metrics.gcell_width_dbu=def.gcell_pitch_x();o.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
    o.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;o.tsv_policy.tsv_obstacle_scope=TsvObstacleScope::INTERFACE;o.official_metrics.tsv_congestion_ratio=1;
    o.fast_initial=true;o.fast_quality=true;o.v7_cell_cost=true;o.official_score_pricing=true;o.climb_escape_radius=3;
    o.v7_topology=true;o.time_limit=40000;
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,out);
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
