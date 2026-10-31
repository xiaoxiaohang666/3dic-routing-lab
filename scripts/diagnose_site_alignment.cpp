// Observation-only readback. Never called by production candidate acceptance.
#include "objectives/official_metrics.hpp"
#include "scoring/compat_resources.hpp"
#include "scoring/site_wirelength.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"
#include <array>
#include <set>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace eda;
int main(int argc,char** argv) { try {
 std::string df,lf,gf,expected,out,csv;double gain=20;
 for(int i=1;i<argc;i+=2) {
  if(i+1==argc) throw std::runtime_error("missing value");
  std::string k=argv[i],v=argv[i+1];
  if(k=="--def")df=v;else if(k=="--lef")lf=v;else if(k=="--guide")gf=v;
  else if(k=="--expected-csv")expected=v;else if(k=="--output")out=v;
  else if(k=="--cells-csv")csv=v;else if(k=="--thermal-gain")gain=std::stod(v);else throw std::runtime_error("unknown option");
 }
 if(df.empty()||lf.empty()||gf.empty()||out.empty()||expected.empty()||csv.empty())throw std::runtime_error("required def lef guide expected-csv output cells-csv");
 if(std::filesystem::exists(out)||std::filesystem::exists(csv))throw std::runtime_error("output must be new");
 LefParser lef;if(!lef.parse(lf))throw std::runtime_error("LEF parse failed");
 DefParser def;if(!def.parse(df,lef,DefGridMapping::OFFICIAL_FULL))throw std::runtime_error("DEF parse failed");
 RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true});
 auto guides=compat::read_guides(gf);if(guides.size()!=def.nets().size())throw std::runtime_error("net count differs");
 OfficialMetricOptions opt;opt.score_20261020=true;opt.a150_rules=true;opt.tsv_track_demand=true;
 opt.gcell_width_dbu=def.gcell_pitch_x();opt.gcell_height_dbu=def.gcell_pitch_y();opt.thermal_gain=gain;
 const double pitch=std::max(def.gcell_pitch_x(),def.gcell_pitch_y());
 std::array<std::vector<int>,6> demand;for(auto& d:demand)d.resize(graph.size());
 double documented_L=0,mst_raw=0,mst_scaled=0,mst_tsvraw=0,strict_mst=0,closed_contact_L=0,ordinary_vias=0,closed_vias=0,all_pair_L=0,all_pair_vias=0;
 for(const auto& net:def.nets()) {
  const auto f=guides.find(net.name);std::string error;
  if(f==guides.end()||!validate_gcell_encoding(net,def.die_area(),graph.grid(),lef,f->second,error)||
    !validate_net_guides(net,def.die_area(),lef,f->second,error,def.gcell_pitch_x(),def.gcell_pitch_y()))throw std::runtime_error("invalid Guide "+net.name+" "+error);
  auto physical=measure_official_net_guides(net,f->second,graph,lef,opt,0);
  for(auto p:physical.tsv_points)if(!graph.tsv_landing_legal(graph.grid().get_col(p.x),graph.grid().get_row(p.y),TsvObstacleScope::ALL_LAYERS))throw std::runtime_error("A150 forbidden landing");
  auto limited=net;
  for(auto& pin:limited.pins)if(pin.port_rects.size()>1)pin.port_rects.resize(1);
  const auto first_port=measure_official_net_guides(limited,f->second,graph,lef,opt,0);
  for(auto [n,d]:first_port.demand)demand[4][n]+=d;
  documented_L+=physical.wirelength;ordinary_vias+=physical.ordinary_connections;
  double contact=physical.planar_wirelength;
  for(const auto& a:f->second) {
   const int l=get_layer_index(a.layer);if(l==20)continue;
   for(const auto& b:f->second)if(get_layer_index(b.layer)==l+1&&a.x1<=b.x2&&b.x1<=a.x2&&a.y1<=b.y2&&b.y1<=a.y2) {
    contact+=(l==10?6:3)*pitch;if(l!=10)++closed_vias;break;
   }
  }
  closed_contact_L+=contact;
  double pairs=physical.planar_wirelength;
  for(const auto& a:f->second)for(const auto& b:f->second)if(get_layer_index(b.layer)==get_layer_index(a.layer)+1 &&
      std::min(a.x2,b.x2)>std::max(a.x1,b.x1)&&std::min(a.y2,b.y2)>std::max(a.y1,b.y1)) {
   bool cross=get_layer_index(a.layer)==10;pairs+=(cross?6:3)*pitch;if(!cross)++all_pair_vias;
  }
  all_pair_L+=pairs;for(auto [n,d]:physical.demand)demand[0][n]+=d;
  const auto raw=measure_official_net_guides(net,f->second,graph.grid(),graph.die_area(),lef,opt,0);
  const auto pins=graph.own_pin_nodes(net);
  std::set<int> exempt_via_cells;
  for(const auto& a:f->second)for(const auto& b:f->second) {
   int l=get_layer_index(a.layer);if(l==10||get_layer_index(b.layer)!=l+1)continue;
   double x1=std::max(a.x1,b.x1),x2=std::min(a.x2,b.x2),y1=std::max(a.y1,b.y1),y2=std::min(a.y2,b.y2);
   if(x1>=x2||y1>=y2)continue;
   int c=graph.grid().get_col((x1+x2)/2),r=graph.grid().get_row((y1+y2)/2);
   for(int layer:{l,l+1}) {int n=graph.node(c,r,layer);if(net_cell_pin_exempt(pins,n))exempt_via_cells.insert(n);}
  }
  for(int n:exempt_via_cells)++demand[5][n];
  for(auto [n,d]:raw.demand) {
   demand[3][n]+=1;
   if(!net_cell_pin_exempt(pins,n)) {demand[1][n]+=d;demand[2][n]+=1;}
  }
  // Historical rectangle MST is diagnostic only: its ordering/omitted via
  // choices cannot replace documented physical charging in the router.
  strict_mst+=site_rectangle_mst_diagnostic(f->second,lef,pitch);
  mst_raw+=measure_net_guides(f->second,graph,lef,6,3).wirelength;
  mst_scaled+=measure_net_guides(f->second,graph,lef,6*pitch,3*pitch).wirelength;
  mst_tsvraw+=measure_net_guides(f->second,graph,lef,6,3*pitch).wirelength;
 }
 struct Expected {int cap,demand,overflow;};std::map<int,Expected> observed;
 std::ifstream e(expected);std::string line;std::getline(e,line);
 auto fields=[](const std::string& s) {std::vector<std::string> v;std::stringstream in(s);std::string w;while(std::getline(in,w,',')){if(!w.empty()&&w.back()=='\r')w.pop_back();v.push_back(w);}return v;};
 auto head=fields(line);std::map<std::string,size_t> columns;for(size_t i=0;i<head.size();++i)columns[head[i]]=i;
 while(std::getline(e,line)) {auto v=fields(line);if(v.size()!=head.size())continue;
  auto value=[&](const std::string& name) {return v.at(columns.at(name));};
  const int col=std::stoi(value("gcell_col")),row=std::stoi(value("gcell_row")),layer=get_layer_index(value("layer"));
  const int n=graph.node(col,row,layer);observed[n]={std::stoi(value("capacity")),std::stoi(value("demand")),std::stoi(value("overflow"))};
 }
 if(observed.empty())throw std::runtime_error("no official overflow cells read");
 struct Sum {uint64_t overflow=0,entries=0,extra=0,missing=0,different=0,matched=0,capdiff=0;long double q=0;};std::array<Sum,8> sums;
 const std::array<std::string,8> names={"physical_floor_a150","physical_no_floor_a150","physical_no_floor_additive_tsv","physical_no_floor_guide_only","no_floor_unmasked_guide","no_floor_first_port_credit","nofloor_ownpin_via_per_net","nofloor_ownpin_via_per_cell"};
 std::ofstream cells(csv);cells<<"node,layer,col,row,capacity,fixed_tracks,pin_floor,physical_guide,additive_guide,guide_only,unmasked_guide,official_capacity,official_demand,official_overflow\n";
 for(size_t n=0;n<graph.size();++n) {
  const int cap=graph.official_capacity(int(n)),floor=graph.official_pin_zero_track_floor_demand(int(n)),fixed=graph.official_obstacle_demand(int(n))-floor;
  const std::array<int,8> total={fixed+floor+demand[0][n],fixed+demand[0][n],fixed+demand[1][n],fixed+demand[2][n],fixed+demand[3][n],fixed+demand[4][n],fixed+demand[0][n]+demand[5][n],fixed+demand[0][n]+int(demand[5][n]>0)};
  auto f=observed.find(int(n));bool interesting=f!=observed.end();
  for(size_t k=0;k<sums.size();++k) {
   auto& s=sums[k];const int excess=std::max(0,total[k]-cap);s.overflow+=excess;s.entries+=excess>0;
   s.q+=congestion_cell_penalty(total[k],cap,1);interesting=interesting||excess>0;
   if(f==observed.end())s.extra+=excess>0;else {
    s.capdiff+=cap!=f->second.cap;s.missing+=excess==0;
    s.matched+=(cap==f->second.cap&&total[k]==f->second.demand);
    s.different+=(cap!=f->second.cap||total[k]!=f->second.demand);
   }
  }
  if(interesting)cells<<n<<','<<graph.layer(n)<<','<<graph.col(n)<<','<<graph.row(n)<<','<<cap<<','<<fixed<<','<<floor<<','<<demand[0][n]<<','<<demand[1][n]<<','<<demand[2][n]<<','<<demand[3][n]<<','
   <<(f==observed.end()?-1:f->second.cap)<<','<<(f==observed.end()?-1:f->second.demand)<<','<<(f==observed.end()?0:f->second.overflow)<<'\n';
 }
 std::ofstream report(out);report<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"production_alignment_passed\":false,\"official_cells\":"<<observed.size()<<",\"documented_L\":"<<documented_L<<",\"all_pair_L\":"<<all_pair_L<<",\"all_pair_vias\":"<<all_pair_vias<<",\"closed_contact_L\":"<<closed_contact_L<<",\"ordinary_vias\":"<<ordinary_vias<<",\"closed_vias\":"<<closed_vias<<",\"strict_mst\":"<<strict_mst<<",\"mst_raw\":"<<mst_raw<<",\"mst_scaled\":"<<mst_scaled<<",\"mst_tsvraw\":"<<mst_tsvraw<<",\"resources\":{";
 for(size_t k=0;k<sums.size();++k) {if(k)report<<',';const auto& s=sums[k];report<<'"'<<names[k]<<"\":{\"overflow\":"<<s.overflow<<",\"entries\":"<<s.entries<<",\"Q\":"<<double(s.q)<<",\"C\":"<<100/std::sqrt(1+double(s.q))<<",\"extra\":"<<s.extra<<",\"missing\":"<<s.missing<<",\"matched\":"<<s.matched<<",\"different\":"<<s.different<<",\"capdiff\":"<<s.capdiff<<'}';}
 report<<"}}\n";std::cout<<"diagnosis complete\n";return 0;
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}}
