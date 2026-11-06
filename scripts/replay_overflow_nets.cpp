// Read-only replay for overflowing cells: load a saved Guide's demand into the real
// router, take one overflowing net out, and ask the real route builder to route it
// again, once with the quick template search and once with a large maze-search
// budget. Nothing is committed and no Guide is written; results are offline
// estimates. Tells whether the builder itself can find a way around the full cell.
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
struct Row {int layer{},col{},row{};std::string net;};
std::vector<Row> read_rows(const std::string& file) {
    std::ifstream in(file);std::string line;std::getline(in,line);std::map<std::string,size_t> at;
    {std::stringstream s(line);std::string x;size_t i=0;while(std::getline(s,x,',')) at[x]=i++;}
    std::vector<Row> rows;
    while(std::getline(in,line)) {
        std::vector<std::string> f;std::stringstream s(line);std::string x;while(std::getline(s,x,',')) f.push_back(x);
        Row r;r.layer=std::stoi(f.at(at.at("layer")));r.col=std::stoi(f.at(at.at("col")));r.row=std::stoi(f.at(at.at("row")));
        r.net=f.at(at.at("net"));rows.push_back(r);
    }
    return rows;
}
}
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,const std::map<std::string,std::vector<GuideRect>>& guides_in,
                  const std::vector<Row>& rows,const fs::path& out,double present) {
    auto& g=router.graph_;const auto& mo=router.options_.official_metrics;
    router.initial_build_=false;router.present_=present;
    const size_t nets=def.nets().size();
    std::vector<NetGuideMetrics> measured;measured.reserve(nets);std::unordered_map<std::string,size_t> index;
    long double total_l=0;
    for(size_t i=0;i<nets;++i) {
        const auto& net=def.nets()[i];index[net.name]=i;
        measured.push_back(measure_official_net_guides(net,guides_in.at(net.name),g,lef,mo,0));
        total_l+=measured.back().wirelength;
        for(auto [n,d]:measured.back().demand) router.demand_[size_t(n)]+=d;
    }
    router.routes_.assign(nets,NetRoute{});
    const auto cq=[&](int n,int d){return (long double)official_resource_cell_contribution(g,n,d).squared;};
    long double total_q=0;for(size_t n=0;n<g.size();++n) total_q+=cq(int(n),router.demand_[n]);
    const long double reference=router.full_reference_;
    router.official_wirelength_=total_l;router.official_reference_=reference;router.official_squared_overflow_=total_q;
    const auto score=[&](long double l,long double q){return .45*100*std::min(1.0L,reference/l)+.30*100/std::sqrt(1+std::max(0.0L,q));};
    const double s0=double(score(total_l,total_q));
    std::cout<<"replay Q="<<double(total_q)<<" L="<<double(total_l)<<" R="<<double(reference)<<" present="<<present
             <<" price="<<router.official_congestion_price()<<" ell0="<<g.ell0()<<'\n';
    const auto show=[&](const std::vector<std::pair<int,int>>& demand){
        std::ostringstream p;std::vector<int> ns;for(auto [n,d]:demand) if(d>0) ns.push_back(n);
        std::sort(ns.begin(),ns.end(),[&](int a,int b){return std::tuple(g.layer(a),g.row(a),g.col(a))<std::tuple(g.layer(b),g.row(b),g.col(b));});
        for(int n:ns) p<<' '<<g.layer(n)<<'/'<<g.col(n)<<'/'<<g.row(n);
        std::string s=p.str();if(s.size()>240) s=s.substr(0,240);return s;};
    std::ofstream csv(out/"replay.csv");csv<<std::setprecision(10);
    csv<<"layer,col,row,net,pins,mode,status,old_over_cells,new_over_cells,keeps_cell,dQ,dL,dS,new_cells\n";
    std::map<std::string,std::array<size_t,4>> tally;  // tried, no overflow, keeps the cell, score improves
    std::set<std::string> done;
    for(const auto& row:rows) {
        if(!done.insert(row.net).second) continue;
        const size_t i=index.at(row.net);const auto& net=def.nets()[i];auto& m=measured[i];
        if(net.is_cross_die) {csv<<row.layer<<','<<row.col<<','<<row.row<<','<<row.net<<','<<net.pins.size()<<",skip,cross_die,,,,,,,\n";continue;}
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]-=d;
        router.official_wirelength_=total_l-m.wirelength;
        long double q_without=total_q;for(auto [n,d]:m.demand) q_without+=cq(n,router.demand_[n])-cq(n,router.demand_[n]+d);
        router.official_squared_overflow_=q_without;router.pricing_generation_=std::numeric_limits<uint64_t>::max();++router.state_generation_;
        const auto over_cells=[&](const std::vector<std::pair<int,int>>& demand){int c=0;for(auto [n,d]:demand) if(d>0&&double(g.official_obstacle_demand(n))+router.demand_[n]+d>g.official_capacity(n)) ++c;return c;};
        const int target=g.node(row.col,row.row,row.layer);
        const int old_over=over_cells(m.demand);
        for(const std::string mode:{"template","maze_large_budget"}) {
            csv<<row.layer<<','<<row.col<<','<<row.row<<','<<row.net<<','<<net.pins.size()<<','<<mode;
            auto& t=tally[mode];
            try {
                if(mode=="maze_large_budget") {
                    router.options_.v7_rrr_astar_expansions=1000000;router.options_.search_limit=1000000;
                    router.v7_refinement_active_=true;router.v7_refinement_expansion_start_=router.counters_.expansions;
                    router.v7_refinement_deadline_=std::chrono::steady_clock::now()+std::chrono::seconds(10);
                } else router.v7_refinement_active_=false;
                auto trial=router.build_route(i,-1);
                router.v7_refinement_active_=false;
                const int new_over=over_cells(trial.measured.demand);
                bool keeps=false;for(auto [n,d]:trial.measured.demand) if(n==target&&d>0) keeps=true;
                long double q=q_without;for(auto [n,d]:trial.measured.demand) q+=cq(n,router.demand_[n]+d)-cq(n,router.demand_[n]);
                const long double l=total_l-m.wirelength+trial.measured.wirelength;
                const double ds=double(score(l,q))-s0;
                csv<<",ok,"<<old_over<<','<<new_over<<','<<keeps<<','<<double(q-total_q)<<','<<double(l-total_l)<<','<<ds<<','<<show(trial.measured.demand)<<'\n';
                ++t[0];if(!new_over) ++t[1];if(keeps) ++t[2];if(ds>1e-9) ++t[3];
            } catch(const std::exception& e) {
                router.v7_refinement_active_=false;
                std::string w=e.what();std::replace(w.begin(),w.end(),',',';');
                csv<<','<<w<<','<<old_over<<",,,,,,\n";
            }
        }
        for(auto [n,d]:m.demand) router.demand_[size_t(n)]+=d;
    }
    std::ofstream s(out/"replay-summary.json");
    s<<"{\"scope\":\"one-net replay on a saved Guide; nothing committed; offline estimate\",\"Q\":"<<double(total_q)<<",\"present\":"<<present<<",\"modes\":{";
    bool first=true;
    for(const auto& [k,t]:tally){s<<(first?"":",")<<'"'<<k<<"\":{\"tried\":"<<t[0]<<",\"no_overflow\":"<<t[1]<<",\"still_uses_cell\":"<<t[2]<<",\"score_improves\":"<<t[3]<<'}';first=false;}
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
    o.time_limit=3000;
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    const auto rows=read_rows(a.at("--cells"));
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,rows,out,std::stod(a.at("--present")));
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
