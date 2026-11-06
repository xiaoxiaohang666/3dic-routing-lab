// Read-only: where a saved Guide's length exceeds the official reference, net by net.
//
// The official reference of a net is the native FLUTE tree length on its pin
// points, plus (FLUTE junction via count, +10 for a cross-die net) ordinary via
// charges, plus one TSV charge for a cross-die net. For every net this tool
// writes the submitted Guide's planar / via / TSV charge next to the reference's
// planar / via / TSV parts, the pin bounding box, the pin layers and the layers
// the Guide uses, and per-case totals of planar length per layer and via
// connections per adjacent layer pair. The reference split is recomputed with the
// same points and options as the router's own reference and checked against it.
//
// Nothing is routed, committed or written back; offline diagnosis only.
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
#include <limits>
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
// Cells whose interiors a positive-area rectangle intersects (last cell closed).
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
}
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,
                  const std::map<std::string,std::vector<GuideRect>>& guides,const fs::path& out) {
    const auto& g=router.graph_;const auto& mo=router.options_.official_metrics;const auto& grid=g.grid();
    const double pitch=std::max(mo.gcell_width_dbu,mo.gcell_height_dbu);
    std::array<long double,21> planar_by_layer{};std::array<size_t,21> vias_by_pair{}; // pair l means l..l+1
    std::array<long double,21> planar_by_layer_check{};
    size_t reference_mismatch=0,rect_count=0;long double total_l=0,total_ref=0,check_planar=0,check_conn=0,measured_planar=0,measured_conn=0;
    std::ofstream csv(out/"nets.csv");csv<<std::setprecision(12);
    csv<<"net,class,pins,cross,L,planar,via_L,tsv_L,vias,ref,ref_planar,ref_vias,ref_tsv,ref_skipped,hpwl,"
         "pin_layer_min,pin_layer_max,guide_layers,guide_layer_max_in_die,rects\n";
    for(size_t i=0;i<def.nets().size();++i) {
        const auto& net=def.nets()[i];
        const auto found=guides.find(net.name);if(found==guides.end()) throw std::runtime_error("net without Guide: "+net.name);
        const auto m=measure_official_net_guides(net,found->second,g,lef,mo,router.reference_lengths_[i]);
        // Same points as NativeFluteBaseline::measure_net.
        std::vector<Point> points;std::set<std::string> names;
        for(const auto& pin:net.pins) if(names.insert(pin.name).second)
            points.push_back(pin.port_rects.empty()?pin.coord:pin.port_rects.front().rect.center());
        if(mo.snap_reference_coordinates) for(auto& p:points) {
            const auto stabilize=[](double x){const double n=std::round(x);
                return std::abs(x-n)<=8*std::numeric_limits<double>::epsilon()*std::max(1.0,std::abs(x))?n:x;};
            p.x=stabilize(p.x);p.y=stabilize(p.y);
        }
        const auto f=router.flute_->evaluate_points(points);
        const double ref_planar=f.skipped_degree_limit?0:f.wirelength_dbu;
        const int ref_vias=f.skipped_degree_limit?0:f.via_count+(net.is_cross_die?10:0);
        const double ref_tsv=(!f.skipped_degree_limit&&net.is_cross_die)?mo.tsv_cost_ratio*pitch:0;
        const double ref=ref_planar+ref_vias*mo.via_cost_ratio*pitch+ref_tsv;
        if(std::abs(ref-router.reference_lengths_[i])>1e-6*std::max(1.0,ref)) ++reference_mismatch;
        double x1=1e300,y1=1e300,x2=-1e300,y2=-1e300;
        for(const auto& p:points) {x1=std::min(x1,p.x);y1=std::min(y1,p.y);x2=std::max(x2,p.x);y2=std::max(y2,p.y);}
        int pin_min=99,pin_max=0;
        for(const auto& pin:net.pins) {pin_min=std::min(pin_min,pin.layer_idx);pin_max=std::max(pin_max,pin.layer_idx);}
        // Per-layer planar cells (deduplicated within the net) and adjacent-layer overlaps.
        std::array<std::vector<const GuideRect*>,21> by_layer;
        for(const auto& r:found->second) {const int l=get_layer_index(r.layer);if(l>=1&&l<=20) by_layer[size_t(l)].push_back(&r);}
        int layers=0,max_in_die=0;
        for(int l=1;l<=20;++l) {
            if(by_layer[size_t(l)].empty()) continue;
            ++layers;max_in_die=std::max(max_in_die,l>10?l-10:l);rect_count+=by_layer[size_t(l)].size();
            const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
            std::unordered_set<long long> cells;
            for(const auto* r:by_layer[size_t(l)]) {
                const auto [cb,ce]=span_cells(r->x1,r->x2,grid.x_lines);const auto [rb,re]=span_cells(r->y1,r->y2,grid.y_lines);
                for(int rr=rb;rr<re;++rr) for(int cc=cb;cc<ce;++cc) cells.insert((long long)rr*grid.cols+cc);
            }
            const long double len=(long double)cells.size()*(h?mo.gcell_width_dbu:mo.gcell_height_dbu);
            planar_by_layer[size_t(l)]+=len;check_planar+=len;
            if(l<20) for(const auto* a:by_layer[size_t(l)]) for(const auto* b:by_layer[size_t(l+1)])
                if(positive_overlap(*a,*b)) {++vias_by_pair[size_t(l)];++check_conn;}
        }
        measured_planar+=m.planar_wirelength;measured_conn+=m.ordinary_connections+m.tsv_connections;
        total_l+=m.wirelength;total_ref+=router.reference_lengths_[i];
        csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<','<<net.is_cross_die<<','<<m.wirelength<<','
           <<m.planar_wirelength<<','<<m.via_wirelength<<','<<m.tsv_wirelength<<','<<m.ordinary_connections<<','
           <<router.reference_lengths_[i]<<','<<ref_planar<<','<<ref_vias<<','<<ref_tsv<<','<<f.skipped_degree_limit<<','
           <<(x2-x1)+(y2-y1)<<','<<pin_min<<','<<pin_max<<','<<layers<<','<<max_in_die<<','<<found->second.size()<<'\n';
    }
    std::ofstream s(out/"totals.json");s<<std::setprecision(15);
    s<<"{\"scope\":\"read-only per-net comparison of a saved Guide with the official reference; offline\","
     <<"\"L\":"<<double(total_l)<<",\"reference\":"<<double(total_ref)<<",\"pitch\":"<<pitch
     <<",\"gcell_width\":"<<mo.gcell_width_dbu<<",\"gcell_height\":"<<mo.gcell_height_dbu
     <<",\"reference_split_mismatch_nets\":"<<reference_mismatch
     <<",\"per_layer_planar_check\":{\"recomputed\":"<<double(check_planar)<<",\"measured\":"<<double(measured_planar)<<'}'
     <<",\"connection_check\":{\"recomputed\":"<<double(check_conn)<<",\"measured\":"<<double(measured_conn)<<'}'
     <<",\"rects\":"<<rect_count<<",\"planar_by_layer\":{";
    for(int l=1;l<=20;++l) s<<(l>1?",":"")<<'"'<<l<<"\":"<<double(planar_by_layer[size_t(l)]);
    s<<"},\"connections_by_lower_layer\":{";
    for(int l=1;l<20;++l) s<<(l>1?",":"")<<'"'<<l<<"\":"<<vias_by_pair[size_t(l)];
    s<<"}}\n";
    std::cout<<"done L="<<double(total_l)<<" R="<<double(total_ref)<<" reference_mismatch="<<reference_mismatch
             <<" planar "<<double(check_planar)<<"/"<<double(measured_planar)<<" conn "<<double(check_conn)<<"/"<<double(measured_conn)<<'\n';
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
    o.fast_initial=true;o.fast_quality=true;o.v7_cell_cost=true;o.official_score_pricing=true;o.climb_escape_radius=3;
    o.v7_topology=true;o.time_limit=40000;
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,out);
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
