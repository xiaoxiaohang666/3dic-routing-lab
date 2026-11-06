// Read-only replay on a saved Guide: load every net's demand into the real router,
// then for an evenly spaced sample of nets per class (same/cross die, pin count),
// take one net out and ask the real route builder to route it again with the
// quick template search and with the maze search at two budgets, plus a maze
// run that ignores congestion (the router's own length floor). Cross-die nets
// keep their current TSV cell. Nothing is committed and no Guide is written;
// the deltas are one-net-at-a-time offline estimates, not achievable totals.
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
using namespace eda;
namespace fs=std::filesystem;
namespace {
std::string net_class(const Net& net) {
    const size_t p=net.pins.size();
    const char* bucket=p<=2?"2":p<=4?"3-4":p<=8?"5-8":p<=16?"9-16":p<=32?"17-32":"33+";
    return std::string(net.is_cross_die?"cross:":"same:")+bucket;
}
struct Mode {std::string name;bool maze;size_t expansions;double seconds;double present;bool ignore_routes{false};};
}
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,
                  const std::map<std::string,std::vector<GuideRect>>& guides_in,size_t per_class,const fs::path& out) {
    auto& g=router.graph_;const auto& mo=router.options_.official_metrics;
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
    // TSV cell -> policy slot, so a cross-die net is rebuilt on its own landing.
    const auto& grid=g.grid();std::unordered_map<int,int> cell_slot;
    const auto& slots=router.policy_.slots();
    for(size_t s=0;s<slots.size();++s) {
        const int c=grid.get_col(slots[s].point.x),r=grid.get_row(slots[s].point.y);
        if(c>=0&&r>=0) cell_slot.emplace(r*grid.cols+c,int(s));
    }
    std::map<std::string,std::vector<size_t>> classes;
    for(size_t i=0;i<nets;++i) classes[net_class(def.nets()[i])].push_back(i);
    std::vector<size_t> sample;
    for(auto& [name,members]:classes) {
        const size_t take=std::min(per_class,members.size());
        for(size_t k=0;k<take;++k) sample.push_back(members[(2*k+1)*members.size()/(2*take)]);
    }
    std::cout<<"replay Q="<<double(total_q)<<" L="<<double(total_l)<<" R="<<double(reference)<<" S_WC="<<s0
             <<" sample="<<sample.size()<<" price="<<router.official_congestion_price()<<'\n'<<std::flush;
    const std::vector<Mode> modes{
        {"template",false,0,0,1},
        {"maze_production",true,20000,0.03,1},
        {"maze_large",true,1000000,5,1},
        {"maze_large_no_congestion",true,1000000,5,0},
        // Other nets' routes and TSVs are hidden from the search; only cells that
        // PIN/OBS alone fill still cost. Shows how much detour other routes force.
        {"maze_large_fixed_walls",true,1000000,5,1,true}};
    std::vector<int> no_routes(router.demand_.size(),0);
    std::ofstream csv(out/"replay.csv");csv<<std::setprecision(12);
    csv<<"net,class,pins,mode,status,old_L,dL,d_planar,d_via,dQ,dS,old_over_cells,new_over_cells,seconds,new_over_fixed_full\n";
    size_t done=0;const auto started=std::chrono::steady_clock::now();
    for(size_t i:sample) {
        const auto& net=def.nets()[i];auto& m=measured[i];
        int slot=-1;
        if(net.is_cross_die) {
            const auto found=m.tsv_cells.size()==1?cell_slot.find(m.tsv_cells[0]):cell_slot.end();
            if(found==cell_slot.end()) {csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<",all,no_slot,,,,,,,,,\n";continue;}
            slot=found->second;
        }
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]-=d;
        router.official_wirelength_=total_l-m.wirelength;
        long double q_without=total_q;for(auto [n,d]:m.demand) q_without+=cq(n,router.demand_[n])-cq(n,router.demand_[n]+d);
        router.official_squared_overflow_=q_without;
        const auto over_cells=[&](const std::vector<std::pair<int,int>>& demand){int c=0;for(auto [n,d]:demand) if(d>0&&double(g.official_obstacle_demand(n))+router.demand_[n]+d>g.official_capacity(n)) ++c;return c;};
        const int old_over=over_cells(m.demand);
        const auto fixed_full=[&](const std::vector<std::pair<int,int>>& demand){int c=0;for(auto [n,d]:demand) if(d>0&&g.official_capacity(n)>0&&double(g.official_obstacle_demand(n))>=g.official_capacity(n)&&double(g.official_obstacle_demand(n))+router.demand_[n]+d>g.official_capacity(n)) ++c;return c;};
        for(const auto& mode:modes) {
            router.present_=mode.present;router.pricing_generation_=std::numeric_limits<uint64_t>::max();++router.state_generation_;
            router.counters_.v7_topology_seconds=0;
            csv<<net.name<<','<<net_class(net)<<','<<net.pins.size()<<','<<mode.name;
            const auto t0=std::chrono::steady_clock::now();
            try {
                router.v7_refinement_active_=mode.maze;
                if(mode.maze) {
                    router.options_.v7_rrr_astar_expansions=mode.expansions;
                    router.options_.search_limit=std::max<size_t>(250000,mode.expansions);
                    router.v7_refinement_expansion_start_=router.counters_.expansions;
                    router.v7_refinement_deadline_=t0+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(mode.seconds));
                }
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
                csv<<",ok,"<<m.wirelength<<','<<trial.measured.wirelength-m.wirelength<<','
                   <<trial.measured.planar_wirelength-m.planar_wirelength<<','<<trial.measured.via_wirelength-m.via_wirelength<<','
                   <<double(q-total_q)<<','<<double(score(l,q))-s0<<','<<old_over<<','<<over_cells(trial.measured.demand)<<','<<secs<<','
                   <<fixed_full(trial.measured.demand)<<'\n';
            } catch(const std::exception& e) {
                router.v7_refinement_active_=false;
                std::string w=e.what();std::replace(w.begin(),w.end(),',',';');std::replace(w.begin(),w.end(),'\n',' ');
                csv<<','<<w.substr(0,160)<<','<<m.wirelength<<",,,,,,"<<old_over<<",,"
                   <<std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count()<<'\n';
            }
        }
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]+=d;
        if(++done%200==0) std::cout<<"[REPLAY] "<<done<<'/'<<sample.size()<<" seconds="
            <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<'\n'<<std::flush;
    }
    std::ofstream s(out/"classes.json");
    s<<"{\"scope\":\"one-net-at-a-time replay on a saved Guide; nothing committed; offline estimate\",\"L\":"<<double(total_l)
     <<",\"reference\":"<<double(reference)<<",\"Q\":"<<double(total_q)<<",\"classes\":{";
    bool first=true;
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
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,std::stoul(a.at("--per-class")),out);
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
