// Offline estimate of the end-of-run overflow cleanup on a saved Guide. Loads
// the Guide's demand into the real router, then repeats in rounds: each net in
// an overflowing cell is taken out and rerouted by the real route builder; a
// move is kept only when wirelength+congestion score improves. With blockers
// on, a net that cannot leave alone is rerouted together with the nets filling
// the full cells next to it (same selection rule as overflow_blockers). Cross-
// die nets are skipped, so T is unchanged. Nothing is written back to a Guide;
// results are offline estimates, not official scores.
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
namespace eda {
struct PinLocalTestAccess {
  static void run(NegotiatedRouter& router,const DefParser& def,const LefParser& lef,const std::map<std::string,std::vector<GuideRect>>& guides_in,
                  const fs::path& out,bool blockers,size_t group,int rounds,size_t maze_expansions,double maze_seconds) {
    auto& g=router.graph_;const auto& mo=router.options_.official_metrics;
    router.initial_build_=false;router.present_=1;router.history_scale_=0;
    const size_t nets=def.nets().size();
    std::vector<std::vector<std::pair<int,int>>> demand(nets);std::vector<long double> wl(nets,0);
    std::unordered_map<int,std::set<size_t>> owners;
    long double total_l=0;
    for(size_t i=0;i<nets;++i) {
      const auto& net=def.nets()[i];
      auto m=measure_official_net_guides(net,guides_in.at(net.name),g,lef,mo,0);
      demand[i]=m.demand;wl[i]=m.wirelength;total_l+=m.wirelength;
      for(auto [n,d]:m.demand) if(d>0) {router.demand_[size_t(n)]+=d;owners[n].insert(i);}
    }
    router.routes_.assign(nets,NetRoute{});
    const auto cq=[&](int n,int d){return (long double)official_resource_cell_contribution(g,n,d).squared;};
    long double total_q=0;for(size_t n=0;n<g.size();++n) total_q+=cq(int(n),router.demand_[n]);
    const long double reference=router.full_reference_;
    const auto score=[&](long double l,long double q){return double(.45L*100*std::min(1.0L,reference/l)+.30L*100/std::sqrt(1+std::max(0.0L,q)));};
    const auto sync=[&]{
      router.official_wirelength_=total_l;router.official_reference_=reference;router.official_squared_overflow_=total_q;
      router.pricing_generation_=std::numeric_limits<uint64_t>::max();++router.state_generation_;};
    const auto over=[&](int n){return g.official_capacity(n)>0 && double(g.official_obstacle_demand(n))+router.demand_[size_t(n)]>g.official_capacity(n);};
    const auto count_over=[&]{std::map<int,int> by;int c=0;for(size_t n=0;n<g.size();++n) if(cq(int(n),router.demand_[n])>0) {++c;++by[g.layer(int(n))];}return std::pair(c,by);};
    // Remove or add one net's demand, keeping Q, L and owners exact.
    const auto apply=[&](size_t i,int sign) {
      for(auto [n,d]:demand[i]) {
        if(d<=0) continue;
        const int before=router.demand_[size_t(n)],after=before+sign*d;
        total_q+=cq(n,after)-cq(n,before);router.demand_[size_t(n)]=after;
        if(sign>0) owners[n].insert(i); else owners[n].erase(i);
      }
      total_l+=sign*wl[i];sync();
    };
    const auto priority=[&](size_t i){
      long double p=0;for(auto [n,d]:demand[i]) if(d>0) p+=cq(n,router.demand_[size_t(n)])-cq(n,router.demand_[size_t(n)]-d);return p;};
    const auto route=[&](size_t i) {
      const bool maze=maze_expansions>0 && maze_seconds>0;
      router.options_.v7_rrr_astar_expansions=maze_expansions;
      router.v7_refinement_active_=maze;
      if(maze) {
        router.v7_refinement_expansion_start_=router.counters_.expansions;
        router.v7_refinement_deadline_=std::chrono::steady_clock::now()+
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(maze_seconds));
      }
      try {auto r=router.build_route(i,-1);router.v7_refinement_active_=false;demand[i]=r.measured.demand;wl[i]=r.measured.wirelength;}
      catch(...) {router.v7_refinement_active_=false;throw;}
    };
    // Same rule as NegotiatedRouter::overflow_blockers, on Guide demand.
    const auto find_blockers=[&](size_t i) {
      std::set<int> own;for(auto [n,d]:demand[i]) if(d>0) own.insert(n);
      if(i<router.own_pin_nodes_.size()) own.insert(router.own_pin_nodes_[i].begin(),router.own_pin_nodes_[i].end());
      std::set<int> around;
      for(auto [n,d]:demand[i]) {
        if(d<=0 || !over(n)) continue;
        std::vector<int> anchors{n};for(int x:g.neighbors(n)) if(own.count(x)) anchors.push_back(x);
        for(int a:anchors) for(int x:g.neighbors(a)) {
          if(x==n || own.count(x)) continue;
          const double c=g.official_capacity(x),fixed=g.official_obstacle_demand(x);
          if(c<=0 || fixed>=c || fixed+router.demand_[size_t(x)]<c) continue;
          around.insert(x);
        }
      }
      std::map<size_t,size_t> who;
      for(int cell:around) for(size_t o:owners[cell]) if(o!=i && !def.nets()[o].is_cross_die) who.emplace(o,demand[o].size());
      std::vector<std::pair<size_t,size_t>> ranked;for(auto [net,size]:who) ranked.push_back({size,net});
      std::sort(ranked.begin(),ranked.end());
      std::vector<size_t> result;for(auto [s,net]:ranked) {(void)s;if(result.size()>=group) break;result.push_back(net);}
      return result;
    };
    // Lift the scope, reroute in order, keep only a strict score gain.
    const auto attempt=[&](const std::vector<size_t>& scope) {
      const double s_before=score(total_l,total_q);
      std::vector<std::pair<std::vector<std::pair<int,int>>,long double>> saved;
      for(size_t i:scope) {saved.push_back({demand[i],wl[i]});apply(i,-1);}
      size_t placed=0;bool ok=true;
      for(size_t i:scope) {
        try {route(i);apply(i,1);++placed;} catch(const std::exception&) {ok=false;break;}
      }
      if(ok && score(total_l,total_q)>s_before+1e-12) return true;
      for(size_t k=0;k<placed;++k) apply(scope[k],-1);
      for(size_t k=0;k<scope.size();++k) {demand[scope[k]]=saved[k].first;wl[scope[k]]=saved[k].second;apply(scope[k],1);}
      return false;
    };
    sync();
    const double s0=score(total_l,total_q);const auto [c0,by0]=count_over();
    std::ofstream log(out/"rounds.jsonl");log<<std::setprecision(10);
    std::cout<<"start Q="<<double(total_q)<<" cells="<<c0<<" S_wc="<<s0<<'\n';
    const auto started=std::chrono::steady_clock::now();
    size_t single_ok=0,joint_ok=0,joint_tried=0,joint_nets=0;
    for(int round=0;round<rounds;++round) {
      std::vector<std::pair<long double,size_t>> queue;
      for(size_t i=0;i<nets;++i) if(!def.nets()[i].is_cross_die) {const auto p=priority(i);if(p>0) queue.push_back({-p,i});}
      std::sort(queue.begin(),queue.end());
      size_t s=0,j=0;
      for(auto [p,i]:queue) {
        (void)p;if(priority(i)<=0) continue;
        if(attempt({i})) {++s;continue;}
        if(!blockers) continue;
        const auto b=find_blockers(i);if(b.empty()) continue;
        std::vector<size_t> scope{i};scope.insert(scope.end(),b.begin(),b.end());
        ++joint_tried;joint_nets+=scope.size();
        if(attempt(scope)) ++j;
      }
      single_ok+=s;joint_ok+=j;
      const auto [c,by]=count_over();
      log<<"{\"round\":"<<round+1<<",\"queued\":"<<queue.size()<<",\"single_accepted\":"<<s<<",\"joint_accepted\":"<<j
         <<",\"Q\":"<<double(total_q)<<",\"over_cells\":"<<c<<",\"L\":"<<double(total_l)<<",\"dS_wc\":"<<score(total_l,total_q)-s0<<"}\n";
      std::cout<<"round "<<round+1<<" queued="<<queue.size()<<" single="<<s<<" joint="<<j<<" Q="<<double(total_q)<<" cells="<<c
               <<" dS_wc="<<score(total_l,total_q)-s0<<std::endl;
      if(!s && !j) break;
    }
    const auto [c1,by1]=count_over();
    std::ofstream sum(out/"summary.json");sum<<std::setprecision(10);
    sum<<"{\"scope\":\"offline estimate on a saved Guide; W and C only, cross-die nets skipped; not an official score\","
       <<"\"blockers\":"<<blockers<<",\"group\":"<<group<<",\"maze_expansions\":"<<maze_expansions<<",\"maze_seconds\":"<<maze_seconds
       <<",\"start_cells\":"<<c0<<",\"end_cells\":"<<c1<<",\"end_Q\":"<<double(total_q)<<",\"end_L\":"<<double(total_l)
       <<",\"dS_wc\":"<<score(total_l,total_q)-s0<<",\"single_accepted\":"<<single_ok<<",\"joint_tried\":"<<joint_tried
       <<",\"joint_nets\":"<<joint_nets<<",\"joint_accepted\":"<<joint_ok
       <<",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<",\"end_cells_by_layer\":{";
    bool first=true;for(auto [l,c]:by1) {sum<<(first?"":",")<<'"'<<l<<"\":"<<c;first=false;}
    sum<<"}}\n";
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
    o.time_limit=3000;o.search_limit=1000000;
    const auto guides=compat::read_guides(a.at("--guide"));
    std::map<std::string,std::vector<GuideRect>> copy(guides.begin(),guides.end());
    NegotiatedRouter router(def,lef,thermal,o);
    PinLocalTestAccess::run(router,def,lef,copy,out,a.at("--blockers")=="1",std::stoul(a.at("--group")),std::stoi(a.at("--rounds")),
                            std::stoul(a.at("--maze-expansions")),std::stod(a.at("--maze-seconds")));
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
