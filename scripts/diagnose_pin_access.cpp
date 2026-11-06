// Read-only: how much freedom the input pins give for where a Guide may reach
// them, against the single access the official-model router uses.
//
// In official mode NegotiatedRouter::accesses offers exactly one access per
// pin: the GCell holding the pin's reference point (centre of its first LEF
// RECT) on the layer of that first RECT. The rules allow reaching any suitable
// RECT of a PIN, and every real layer of a multi-layer PIN. For every unique
// pin of every net this tool records
//   * the layers its PORT RECTs are on, and whether one is the next routing
//     layer of its die (an access there would skip the pin-layer cell and via);
//   * the GCells its RECTs cover on the reference layer and on every layer, and
//     whether the reference GCell is one of them;
//   * whether the reference GCell is full of fixed PIN/OBS demand on the pin
//     layer and on the next layer.
// Nothing is routed or written back; offline diagnosis only.
#include "router/negotiated_router.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
using namespace eda;
namespace fs=std::filesystem;
namespace {
std::pair<int,int> span_cells(double low,double high,const std::vector<double>& lines) {
    if(high<low) std::swap(low,high);
    if(high<=lines.front()||low>=lines.back()) return {0,0};
    const int count=int(lines.size())-1;
    const int b=std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,count);
    const int e=std::clamp(int(std::lower_bound(lines.begin(),lines.end(),high)-lines.begin()),0,count);
    return {b,e};
}
std::string bucket(size_t p) {
    return p<=2?"2":p<=4?"3-4":p<=8?"5-8":p<=16?"9-16":p<=32?"17-32":p<=150?"33-150":"151+";
}
}
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const fs::path& out) {
    const auto& g=router.graph_;const auto& grid=g.grid();
    std::ofstream csv(out/"pins.csv");
    csv<<"net,class,pin,owner,ref_layer,layers,next_layer_rect,rects,ref_layer_cells,all_cells,ref_cell_covered,"
         "ref_full,ref_capacity,ref_fixed,next_full,next_capacity,next_fixed\n";
    std::map<std::string,std::array<size_t,8>> by;  // class -> counters
    size_t pins=0;
    for(const auto& net:def.nets()) {
        const std::string cls=std::string(net.is_cross_die?"cross:":"same:")+bucket(net.pins.size());
        std::set<std::string> seen;
        for(const auto& pin:net.pins) {
            if(!seen.insert(pin.name).second) continue;
            ++pins;
            const int l0=pin.layer_idx;
            const int next=l0==10?9:l0==20?19:l0+1;  // next routing layer inside the same die
            std::set<int> layers;std::set<long long> ref_cells,all_cells;bool next_rect=false;
            for(const auto& s:pin.port_rects) {
                layers.insert(s.layer_idx);if(s.layer_idx==next) next_rect=true;
                const auto [cb,ce]=span_cells(s.rect.x1,s.rect.x2,grid.x_lines);
                const auto [rb,re]=span_cells(s.rect.y1,s.rect.y2,grid.y_lines);
                for(int r=rb;r<re;++r) for(int c=cb;c<ce;++c) {
                    const long long key=(long long)r*grid.cols+c;
                    all_cells.insert(key*32+s.layer_idx);
                    if(s.layer_idx==l0) ref_cells.insert(key);
                }
            }
            const int c=grid.get_col(pin.coord.x),r=grid.get_row(pin.coord.y);
            const long long ref_key=(long long)r*grid.cols+c;
            const bool covered=ref_cells.count(ref_key)>0;
            const int n0=g.node(c,r,l0),n1=g.node(c,r,next);
            const double cap0=g.official_capacity(n0),cap1=g.official_capacity(n1);
            const double fix0=g.official_obstacle_demand(n0),fix1=g.official_obstacle_demand(n1);
            const bool full0=fix0>=cap0,full1=fix1>=cap1;
            std::string ls;for(int l:layers) ls+=(ls.empty()?"":"|")+std::to_string(l);
            const char* owner=pin.owner_type==PinOwnerType::CORE_CELL?"core":pin.owner_type==PinOwnerType::BLOCK_MACRO?"block":
                pin.instance_name.empty()?"io":"other";
            csv<<net.name<<','<<cls<<','<<pin.name<<','<<owner<<','<<l0<<','<<ls<<','<<next_rect<<','<<pin.port_rects.size()<<','
               <<ref_cells.size()<<','<<all_cells.size()<<','<<covered<<','<<full0<<','<<cap0<<','<<fix0<<','
               <<full1<<','<<cap1<<','<<fix1<<'\n';
            auto& b=by[cls];++b[0];b[1]+=layers.size()>1;b[2]+=next_rect;b[3]+=ref_cells.size()>1;b[4]+=all_cells.size()>1;
            b[5]+=!covered;b[6]+=full0;b[7]+=full1;
        }
    }
    std::ofstream s(out/"summary.json");
    s<<"{\"scope\":\"read-only pin access freedom of the input; offline\",\"pins\":"<<pins<<",\"classes\":{";
    bool first=true;std::array<size_t,8> all{};
    for(const auto& [k,b]:by) {
        for(size_t i=0;i<8;++i) all[i]+=b[i];
        s<<(first?"":",")<<'"'<<k<<"\":{\"pins\":"<<b[0]<<",\"multi_layer\":"<<b[1]<<",\"next_layer_rect\":"<<b[2]
         <<",\"ref_layer_multi_cell\":"<<b[3]<<",\"any_layer_multi_cell\":"<<b[4]<<",\"ref_cell_not_covered\":"<<b[5]
         <<",\"ref_cell_full\":"<<b[6]<<",\"next_layer_cell_full\":"<<b[7]<<'}';first=false;
    }
    s<<"},\"all\":{\"pins\":"<<all[0]<<",\"multi_layer\":"<<all[1]<<",\"next_layer_rect\":"<<all[2]
     <<",\"ref_layer_multi_cell\":"<<all[3]<<",\"any_layer_multi_cell\":"<<all[4]<<",\"ref_cell_not_covered\":"<<all[5]
     <<",\"ref_cell_full\":"<<all[6]<<",\"next_layer_cell_full\":"<<all[7]<<"}}\n";
    std::cout<<"done pins="<<all[0]<<" multi_layer="<<all[1]<<" next_layer_rect="<<all[2]<<" ref_multi_cell="<<all[3]
             <<" any_multi_cell="<<all[4]<<" ref_not_covered="<<all[5]<<" ref_full="<<all[6]<<" next_full="<<all[7]<<'\n';
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
    // Same router construction as eda_diagnose_reference_gap (d08); only its graph is used.
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
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,out);
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
