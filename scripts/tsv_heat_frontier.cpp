// Read-only diagnostic: for every cross-die net, the legal TSV landing cells
// that are not dominated in (attachment length, heat). Attachment length of a
// landing p is the distance from p to the bottom-die tree plus the distance to
// the top-die tree, each tree being the rectilinear MST of that die's pins and
// "distance to the tree" the distance to the nearest MST edge's bounding box.
// For a two-pin net this is the exact per-die length change; otherwise it is a
// Steiner-style estimate. Nothing is routed; the output feeds an offline
// estimate of how well TSV positions can trade wire length for heat.
#include "parser/def_parser.hpp"
#include "parser/lef_parser.hpp"
#include "parser/thermal_parser.hpp"
#include "router/routing_graph.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
using namespace eda;
namespace fs=std::filesystem;
namespace {
struct Edge { double x1,y1,x2,y2; };
std::vector<Edge> mst_edges(const std::vector<Point>& pins) {
    std::vector<Edge> edges;const size_t n=pins.size();if(n<2) return edges;
    std::vector<double> dist(n,std::numeric_limits<double>::infinity());std::vector<int> from(n,-1);std::vector<bool> used(n,false);
    dist[0]=0;
    for(size_t k=0;k<n;++k) {
        int u=-1;for(size_t j=0;j<n;++j) if(!used[j]&&(u<0||dist[j]<dist[u])) u=int(j);
        used[u]=true;
        if(from[u]>=0) {const auto& a=pins[u];const auto& b=pins[from[u]];
            edges.push_back({std::min(a.x,b.x),std::min(a.y,b.y),std::max(a.x,b.x),std::max(a.y,b.y)});}
        for(size_t v=0;v<n;++v) if(!used[v]) {
            const double d=std::abs(pins[u].x-pins[v].x)+std::abs(pins[u].y-pins[v].y);
            if(d<dist[v]) {dist[v]=d;from[v]=u;}
        }
    }
    return edges;
}
double attach(const std::vector<Point>& pins,const std::vector<Edge>& edges,double x,double y) {
    if(edges.empty()) return std::abs(pins.front().x-x)+std::abs(pins.front().y-y);
    double best=std::numeric_limits<double>::infinity();
    for(const auto& e:edges) {
        const double dx=x<e.x1?e.x1-x:(x>e.x2?x-e.x2:0),dy=y<e.y1?e.y1-y:(y>e.y2?y-e.y2:0);
        best=std::min(best,dx+dy);if(best==0) break;
    }
    return best;
}
}
int main(int argc,char** argv) {
 try {
    std::string def_file,thermal_file,output;std::vector<std::string> lefs;int margin=16;size_t cell_limit=20000,keep=64;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];if(i+1>=argc) throw std::runtime_error("expected option value");
        const std::string value=argv[++i];
        if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);
        else if(key=="--thermal") thermal_file=value;else if(key=="--output") output=value;
        else if(key=="--margin") margin=std::stoi(value);else if(key=="--cell-limit") cell_limit=std::stoul(value);
        else if(key=="--keep") keep=std::stoul(value);else throw std::runtime_error("unknown option "+key);
    }
    if(def_file.empty()||lefs.empty()||thermal_file.empty()||output.empty()) throw std::runtime_error("required --def --lef --thermal --output");
    if(fs::exists(output)) throw std::runtime_error("output must be new");
    LefParser lef;for(const auto& f:lefs) if(!lef.parse(f)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,true});
    const auto& grid=graph.grid();
    ThermalParser thermal;if(!thermal.parse_json(thermal_file,grid,def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))
        throw std::runtime_error("thermal parse failed");
    auto heat=[&](int c,int r){return std::max(0.0,thermal.get_temperature(c,r)-thermal.ambient_temperature());};
    auto cx=[&](int c){return 0.5*(grid.x_lines[c]+grid.x_lines[c+1]);};
    auto cy=[&](int r){return 0.5*(grid.y_lines[r]+grid.y_lines[r+1]);};
    std::ofstream out(output);if(!out) throw std::runtime_error("cannot create output");
    out<<std::setprecision(10)<<"# net forced pins_bottom pins_top candidates | col:row:attach_dbu:heat ...\n";
    size_t legal=0;for(int r=0;r<grid.rows;++r) for(int c=0;c<grid.cols;++c) legal+=graph.tsv_landing_legal(c,r,TsvObstacleScope::INTERFACE);
    out<<"# grid "<<grid.cols<<" "<<grid.rows<<" legal "<<legal<<"\n";
    for(const auto& net:def.nets()) {
        if(!net.is_cross_die) continue;
        std::vector<Point> side[2];std::set<std::string> names;bool forced=false;
        for(const auto& p:net.pins) {
            if(p.layer_idx==10||p.layer_idx==11) forced=true;
            if(names.insert(p.name).second) side[p.layer_idx>=11].push_back(p.coord);
        }
        if(side[0].empty()||side[1].empty()) {out<<net.name<<" 1 "<<side[0].size()<<' '<<side[1].size()<<" 0\n";continue;}
        const auto e0=mst_edges(side[0]),e1=mst_edges(side[1]);
        double x1=std::numeric_limits<double>::infinity(),y1=x1,x2=-x1,y2=-x1;
        for(const auto& s:side) for(const auto& p:s) {x1=std::min(x1,p.x);y1=std::min(y1,p.y);x2=std::max(x2,p.x);y2=std::max(y2,p.y);}
        int c1=std::max(0,grid.get_col(x1)-margin),c2=std::min(grid.cols-1,grid.get_col(x2)+margin);
        int r1=std::max(0,grid.get_row(y1)-margin),r2=std::min(grid.rows-1,grid.get_row(y2)+margin);
        const double area=double(c2-c1+1)*(r2-r1+1);
        const int stride=std::max(1,int(std::ceil(std::sqrt(area/double(cell_limit)))));
        struct Cand{int c,r;double a,h;};std::vector<Cand> cands;
        for(int r=r1;r<=r2;r+=stride) for(int c=c1;c<=c2;c+=stride) {
            if(!graph.tsv_landing_legal(c,r,TsvObstacleScope::INTERFACE)) continue;
            const double x=cx(c),y=cy(r);
            cands.push_back({c,r,attach(side[0],e0,x,y)+attach(side[1],e1,x,y),heat(c,r)});
        }
        std::sort(cands.begin(),cands.end(),[](const Cand& a,const Cand& b){return a.a<b.a||(a.a==b.a&&a.h>b.h);});
        std::vector<Cand> front;double best=-1;
        for(const auto& k:cands) if(k.h>best) {front.push_back(k);best=k.h;}
        if(front.size()>keep) {
            std::vector<Cand> thin;for(size_t i=0;i<keep;++i) thin.push_back(front[i*(front.size()-1)/(keep-1)]);
            front.swap(thin);
        }
        out<<net.name<<' '<<int(forced)<<' '<<side[0].size()<<' '<<side[1].size()<<' '<<front.size();
        for(const auto& k:front) out<<' '<<k.c<<':'<<k.r<<':'<<k.a<<':'<<k.h;
        out<<'\n';
    }
    return 0;
 } catch(const std::exception& e) {std::cerr<<"[ERROR] "<<e.what()<<'\n';return 1;}
}
