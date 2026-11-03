#include "guide_via_stack_relocator.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
using namespace eda;namespace fs=std::filesystem;
namespace {
std::string quoted(const std::string& s){std::string o="\"";for(char c:s){if(c=='\"'||c=='\\')o+='\\';o+=c;}return o+'\"';}
struct Options {std::string def,lef,thermal,guide,runtime,out;double seconds=3300,gain=0,growth=.001;size_t nets=1024,trials=256,rounds=64;int radius=8;bool reject=false;};
Options parse(int argc,char** argv) {
 Options o;for(int i=1;i<argc;++i){const std::string k=argv[i];if(i+1==argc)throw std::runtime_error("missing value");const std::string v=argv[++i];
 if(k=="--def")o.def=v;else if(k=="--lef")o.lef=v;else if(k=="--thermal")o.thermal=v;else if(k=="--guide")o.guide=v;else if(k=="--runtime")o.runtime=v;else if(k=="--output-dir")o.out=v;
 else if(k=="--seconds")o.seconds=std::stod(v);else if(k=="--thermal-gain")o.gain=std::stod(v);else if(k=="--max-L-growth")o.growth=std::stod(v);else if(k=="--nets")o.nets=std::stoul(v);else if(k=="--trials")o.trials=std::stoul(v);else if(k=="--rounds")o.rounds=std::stoul(v);else if(k=="--radius")o.radius=std::stoi(v);else if(k=="--reject-all")o.reject=std::stoi(v)!=0;else throw std::runtime_error("unknown argument "+k);}
 if(o.def.empty()||o.lef.empty()||o.thermal.empty()||o.guide.empty()||o.runtime.empty()||o.out.empty()||!std::isfinite(o.seconds)||o.seconds<=0||o.seconds>36000||!std::isfinite(o.gain)||o.gain<=0||!std::isfinite(o.growth)||o.growth<0||o.growth>.02||o.radius<1||o.radius>32||!o.nets||o.nets>16384||!o.trials||o.trials>16384||!o.rounds||o.rounds>1024)throw std::runtime_error("required --def --lef --thermal --guide --runtime --thermal-gain --output-dir NEW; invalid/missing parameter");
 if(fs::exists(o.out))throw std::runtime_error("output must be new");
 return o;
}
void metrics(std::ostream& s,const OfficialGuideMetrics& m){s<<"{\"L\":"<<m.guide.wirelength<<",\"Q\":"<<m.squared_normalized_overflow<<",\"rho\":"<<m.guide.correlation<<",\"W\":"<<m.wirelength_score<<",\"C\":"<<m.congestion_score<<",\"T\":"<<m.thermal_score<<",\"S\":"<<m.guide.quality*100<<'}';}
}
int main(int argc,char** argv){try{
 using Clock=std::chrono::steady_clock;const auto start=Clock::now();const auto o=parse(argc,argv);fs::create_directories(o.out);
 LefParser lef;if(!lef.parse(o.lef))throw std::runtime_error("LEF");DefParser def;if(!def.parse(o.def,lef,DefGridMapping::OFFICIAL_FULL))throw std::runtime_error("DEF");
 RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});ThermalParser thermal;if(!thermal.parse_json(o.thermal,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))throw std::runtime_error("thermal");
 auto guides=compat::read_guides(o.guide);if(guides.size()!=def.nets().size())throw std::runtime_error("net count mismatch");
 OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;mo.snap_reference_coordinates=true;mo.thermal_gain=o.gain;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
 NativeFluteBaseline flute(o.runtime);std::vector<NetGuideMetrics> measured;measured.reserve(def.nets().size());std::vector<int> demand(graph.size()),mandatory(graph.size());size_t ordinary_vias=0;
 for(const auto& net:def.nets()) {
  const auto it=guides.find(net.name);std::string error;if(it==guides.end()||!validate_gcell_encoding(net,def.die_area(),graph.grid(),lef,it->second,error)||!validate_net_guides(net,def.die_area(),lef,it->second,error,mo.gcell_width_dbu,mo.gcell_height_dbu))throw std::runtime_error("baseline net invalid "+net.name+" "+error);
  measured.push_back(measure_official_net_guides(net,it->second,graph,lef,mo,flute.measure_net(net,std::max(def.gcell_pitch_x(),def.gcell_pitch_y()),mo)));const auto& m=measured.back();ordinary_vias+=m.ordinary_connections;
  for(auto [n,d]:m.demand)demand[n]+=d;
  for(auto p:m.tsv_points){if(!graph.tsv_landing_legal(graph.grid().get_col(p.x),graph.grid().get_row(p.y),TsvObstacleScope::ALL_LAYERS))throw std::runtime_error("baseline A150");for(auto [n,d]:mandatory_tsv_contribution(graph,p,1,true,{},true))mandatory[n]+=d;}
 }
 const auto aggregate=[&](){std::vector<const NetGuideMetrics*> cache;for(const auto& m:measured)cache.push_back(&m);return aggregate_official_metrics(cache,demand,graph,thermal,mo,false);};
 const auto before=aggregate();double L=before.guide.wirelength,Q=before.squared_normalized_overflow;const size_t initial_vias=ordinary_vias;const auto deadline=start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(o.seconds));
 std::ofstream changes(fs::path(o.out)/"changes.jsonl");changes<<std::setprecision(17);size_t attempted=0,accepted=0,trials=0,invalid=0,pin_rejections=0,canonical_rejections=0,windows=0,rounds=0;bool timed_out=false;std::vector<double> ranks(graph.size());
 for(size_t round=0;round<o.rounds;++round){if(Clock::now()>=deadline){timed_out=true;break;}++rounds;
  for(size_t n=0;n<graph.size();++n){ranks[n]=0;const int layer=graph.layer(int(n));if(demand[n]<=mandatory[n]||(graph.tsv_obstacle_layers(graph.col(int(n)),graph.row(int(n)),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(layer-1))))continue;const auto x=official_resource_cell_contribution(graph,int(n),demand[n]);ranks[n]=double(x.squared-official_resource_cell_contribution(graph,int(n),demand[n]-1).squared);}
  std::vector<std::pair<double,size_t>> order;const int plane=graph.grid().rows*graph.grid().cols;
  for(size_t i=0;i<measured.size();++i){double rank=0;for(auto [n,d]:measured[i].demand){if((graph.layer(n)==10||graph.layer(n)==11)&&std::find(measured[i].tsv_cells.begin(),measured[i].tsv_cells.end(),n%plane)!=measured[i].tsv_cells.end())continue;rank+=ranks[n]*d;}if(rank>0)order.emplace_back(rank,i);}
  std::sort(order.begin(),order.end(),[](auto a,auto b){return a.first!=b.first?a.first>b.first:a.second<b.second;});if(order.size()>o.nets)order.resize(o.nets);size_t round_accepted=0;
  for(auto [rank,i]:order){(void)rank;if(Clock::now()>=deadline){timed_out=true;break;}++attempted;
   GuideViaStackOptions opt;opt.deadline=deadline;opt.max_trials=o.trials;opt.radius=o.radius;opt.total_L=L;opt.total_Q=Q;opt.reference=before.guide.reference;opt.max_total_L=before.guide.wirelength*(1+o.growth);opt.reject_all=o.reject;
   const auto& net=def.nets()[i];const auto result=relocate_guide_via_stack(net,guides.at(net.name),graph,lef,mo,opt,demand,[&](int n){return ranks[n];});trials+=result.trials;invalid+=result.invalid;pin_rejections+=result.pin_rejections;canonical_rejections+=result.canonical_rejections;windows+=result.windows;
   if(result.timed_out||Clock::now()>=deadline){timed_out=true;break;}if(result.removed.empty())continue;
   auto next=measure_official_net_guides(net,result.guides,graph,lef,mo,measured[i].reference);std::map<int,int> delta;for(auto [n,d]:measured[i].demand)delta[n]-=d;for(auto [n,d]:next.demand)delta[n]+=d;long double dq=0;
   for(auto [n,d]:delta)if(d)dq+=official_resource_cell_contribution(graph,n,demand[n]+d).squared-official_resource_cell_contribution(graph,n,demand[n]).squared;
   if(std::abs(double(dq)-result.delta_Q)>1e-10||std::abs(next.wirelength-measured[i].wirelength-result.delta_L)>1e-7)throw std::runtime_error("candidate ledger disagreement");
   changes<<"{\"net\":"<<quoted(net.name)<<",\"round\":"<<round<<",\"delta_L\":"<<result.delta_L<<",\"delta_Q\":"<<double(dq)<<",\"via_before\":"<<measured[i].ordinary_connections<<",\"via_after\":"<<next.ordinary_connections<<",\"removed\":"<<result.removed.size()<<",\"added\":"<<result.added.size()<<"}\n";
   for(auto [n,d]:delta)demand[n]+=d;
   L+=result.delta_L;Q+=double(dq);ordinary_vias=ordinary_vias-measured[i].ordinary_connections+next.ordinary_connections;measured[i]=std::move(next);guides.at(net.name)=result.guides;++accepted;++round_accepted;
  }
  std::cout<<"round="<<round<<" accepted="<<round_accepted<<" total="<<accepted<<" L="<<std::setprecision(17)<<L<<" Q="<<Q<<std::endl;if(timed_out||!round_accepted)break;
 }
 const auto after=aggregate();if(std::abs(after.guide.wirelength-L)>1e-6||std::abs(after.squared_normalized_overflow-Q)>1e-9||after.guide.correlation!=before.guide.correlation||after.thermal_score!=before.thermal_score)throw std::runtime_error("full aggregate/invariant disagreement");
 const fs::path target=fs::path(o.out)/"result.guide";if(!accepted)fs::copy_file(o.guide,target);else {std::ofstream out(target.string()+".tmp");out<<std::setprecision(17);for(const auto& [name,rects]:guides){out<<name<<"\n(\n";for(const auto& r:rects)out<<r.x1<<' '<<r.y1<<' '<<r.x2<<' '<<r.y2<<' '<<r.layer<<'\n';out<<")\n";}out.close();if(!out)throw std::runtime_error("Guide export failed");fs::rename(target.string()+".tmp",target);}
 changes.close();if(!changes)throw std::runtime_error("change log failed");std::ofstream out(fs::path(o.out)/"stats.json");out<<std::setprecision(17)<<"{\"scope\":\"fixed-TSV ordinary via-stack relocation\",\"official_evaluation\":false,\"seconds_limit\":"<<o.seconds<<",\"max_L_growth\":"<<o.growth<<",\"radius\":"<<o.radius<<",\"net_limit\":"<<o.nets<<",\"trial_limit\":"<<o.trials<<",\"round_limit\":"<<o.rounds<<",\"rounds\":"<<rounds<<",\"attempted\":"<<attempted<<",\"accepted\":"<<accepted<<",\"trials\":"<<trials<<",\"windows\":"<<windows<<",\"invalid\":"<<invalid<<",\"pin_rejections\":"<<pin_rejections<<",\"canonical_rejections\":"<<canonical_rejections<<",\"timed_out\":"<<(timed_out?"true":"false")<<",\"ordinary_vias_before\":"<<initial_vias<<",\"ordinary_vias_after\":"<<ordinary_vias<<",\"before\":";metrics(out,before);out<<",\"after\":";metrics(out,after);out<<",\"wall_seconds\":"<<std::chrono::duration<double>(Clock::now()-start).count()<<"}\n";if(!out)throw std::runtime_error("stats failed");return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
