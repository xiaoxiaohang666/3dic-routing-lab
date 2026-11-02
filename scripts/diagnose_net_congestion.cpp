// Read-only attribution of a saved Guide. Official sparse CSV is diagnostic
// evidence only; no resource mutation, candidate acceptance or Guide output.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
using namespace eda;
namespace fs=std::filesystem;
struct Hot {int cap{},demand{};};
struct Footprint {NetGuideMetrics metric;};
std::string quoted(const std::string& s) {
    std::string out="\"";for(char c:s){if(c=='\"')out+='\"';out+=c;}return out+'\"';
}
int main(int argc,char** argv) {try {
    if(argc!=6)throw std::runtime_error("DEF LEF GUIDE official-overflow.csv NEW_OUTPUT_DIR required");
    if(fs::exists(argv[5]))throw std::runtime_error("output must be new");
    LefParser lef;if(!lef.parse(argv[2]))throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(argv[1],lef,DefGridMapping::OFFICIAL_FULL))throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});
    const auto& grid=graph.grid();const int plane=grid.cols*grid.rows;
    std::map<int,Hot> hot;std::ifstream csv(argv[4]);std::string line;
    if(!std::getline(csv,line)||line.find("gcell_col,gcell_row,layer,direction,demand,capacity,overflow")!=0)throw std::runtime_error("CSV header");
    while(std::getline(csv,line)) {
        if(line.empty())continue;
        std::vector<std::string> v;std::stringstream in(line);std::string x;
        while(std::getline(in,x,','))v.push_back(x);
        if(v.size()!=7)throw std::runtime_error("CSV width");
        const int n=graph.node(std::stoi(v[0]),std::stoi(v[1]),get_layer_index(v[2]));
        if(n<0||std::stoi(v[6])<=0||!hot.emplace(n,Hot{std::stoi(v[5]),std::stoi(v[4])}).second)throw std::runtime_error("CSV coordinate/duplicate");
        if(graph.official_capacity(n)!=std::stoi(v[5]))throw std::runtime_error("CSV capacity mismatch");
    }
    const auto guides=compat::read_guides(argv[3]);if(guides.size()!=def.nets().size())throw std::runtime_error("net set mismatch");
    fs::create_directory(argv[5]);const fs::path output=argv[5];
    std::ofstream owners(output/"owners.csv"),nets(output/"nets.csv"),cells(output/"cells.csv");
    owners<<"net_index,net,layer,col,row,demand,mandatory_tsv,pin_geometry,own_pin,planar_neighbors,layer_contacts,kind\n";
    nets<<std::setprecision(17)<<"net_index,net,pins,L,planar_L,via_L,tsv_L,ordinary_vias,tsv_count,legacy_rank,ordinary_official_marginal,official_hot_cells\n";
    cells<<"layer,col,row,capacity,official_total,route,mandatory_tsv,production_fixed,pin_geometry\n";
    OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;
    mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
    std::vector<int> total(graph.size()),mandatory(graph.size());std::vector<Footprint> footprints;footprints.reserve(def.nets().size());
    double L=0;size_t index=0;
    for(const auto& net:def.nets()) {
        const auto it=guides.find(net.name);if(it==guides.end())throw std::runtime_error("missing net");
        auto metric=measure_official_net_guides(net,it->second,graph,lef,mo,0);L+=metric.wirelength;
        std::map<int,int> required;const auto pins=graph.own_pin_nodes(net);
        for(auto p:metric.tsv_points)for(auto [n,d]:mandatory_tsv_contribution(graph,p,1,true,pins,true)) {required[n]+=d;mandatory[n]+=d;}
        bool targeted=false;for(auto [n,d]:metric.demand){total[n]+=d;if(hot.count(n))targeted=true;}
        if(targeted) {
            std::unordered_set<int> nodes;
            for(const auto& r:it->second) {
                const int layer=get_layer_index(r.layer);
                const int cb=std::max(0,int(std::upper_bound(grid.x_lines.begin(),grid.x_lines.end(),r.x1)-grid.x_lines.begin())-1);
                const int ce=std::min(grid.cols,int(std::lower_bound(grid.x_lines.begin(),grid.x_lines.end(),r.x2)-grid.x_lines.begin()));
                const int rb=std::max(0,int(std::upper_bound(grid.y_lines.begin(),grid.y_lines.end(),r.y1)-grid.y_lines.begin())-1);
                const int re=std::min(grid.rows,int(std::lower_bound(grid.y_lines.begin(),grid.y_lines.end(),r.y2)-grid.y_lines.begin()));
                for(int row=rb;row<re;++row)for(int col=cb;col<ce;++col)nodes.insert(graph.node(col,row,layer));
            }
            for(auto [n,d]:metric.demand)if(hot.count(n)) {
                const int l=graph.layer(n),c=graph.col(n),r=graph.row(n);const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
                const int step=h?1:grid.cols;int planar=0,contacts=0;
                if((h?c:r)>0)planar+=nodes.count(n-step);
                if((h?c:r)+1<(h?grid.cols:grid.rows))planar+=nodes.count(n+step);
                if(l>1)contacts+=nodes.count(n-plane);
                if(l<20)contacts+=nodes.count(n+plane);
                const bool pin=(graph.tsv_obstacle_layers(c,r,TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(l-1)))!=0;
                owners<<index<<','<<quoted(net.name)<<','<<l<<','<<c<<','<<r<<','<<d<<','<<required[n]<<','<<pin<<','<<std::binary_search(pins.begin(),pins.end(),n)<<','<<planar<<','<<contacts<<','
                    <<(planar==2&&!contacts?"straight":contacts?"via_or_branch":"endpoint")<<'\n';
            }
        }
        footprints.push_back({std::move(metric)});++index;
    }
    std::vector<double> ranks(graph.size());long double production_Q=0;
    for(size_t n=0;n<graph.size();++n) {
        const auto x=official_resource_cell_contribution(graph,int(n),total[n]);production_Q+=x.squared;
        const int l=graph.layer(int(n));
        if(graph.tsv_obstacle_layers(graph.col(int(n)),graph.row(int(n)),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(l-1)))continue;
        if(total[n]>mandatory[n]) {ranks[n]=double(x.squared-official_resource_cell_contribution(graph,int(n),total[n]-1).squared);if(mandatory[n])ranks[n]+=1./std::max(1.,graph.official_capacity(int(n)));}
    }
    index=0;for(const auto& net:def.nets()) {
        const auto& m=footprints[index].metric;double rank=0,marginal=0;size_t hot_count=0;
        for(auto [n,d]:m.demand) {
            const int l=graph.layer(n);const bool tsv=(l==10||l==11)&&std::find(m.tsv_cells.begin(),m.tsv_cells.end(),n%plane)!=m.tsv_cells.end();
            if(!tsv)rank+=ranks[n]*d;
            const auto it=hot.find(n);if(it!=hot.end()) {++hot_count;if(!tsv){const auto& h=it->second;const double e=std::max(0,h.demand-h.cap),after=std::max(0,h.demand-d-h.cap);marginal+=(e*e-after*after)/(double(h.cap)*h.cap);}}
        }
        nets<<index<<','<<quoted(net.name)<<','<<net.pins.size()<<','<<m.wirelength<<','<<m.planar_wirelength<<','<<m.via_wirelength<<','<<m.tsv_wirelength<<','<<m.ordinary_connections<<','<<m.tsv_points.size()<<','<<rank<<','<<marginal<<','<<hot_count<<'\n';++index;
    }
    for(auto [n,h]:hot) {
        const int l=graph.layer(n);const bool pin=(graph.tsv_obstacle_layers(graph.col(n),graph.row(n),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(l-1)))!=0;
        cells<<l<<','<<graph.col(n)<<','<<graph.row(n)<<','<<h.cap<<','<<h.demand<<','<<total[n]<<','<<mandatory[n]<<','<<graph.official_obstacle_demand(n)<<','<<pin<<'\n';
    }
    if(!owners||!nets||!cells)throw std::runtime_error("output write failed");
    std::ofstream summary(output/"summary.json");summary<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"routing_run\":false,\"official_csv_used_only_for_attribution\":true,\"nets\":"<<index<<",\"L\":"<<L<<",\"production_Q\":"<<double(production_Q)<<",\"official_hot_cells\":"<<hot.size()<<"}\n";
    if(!summary)throw std::runtime_error("summary write failed");
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
