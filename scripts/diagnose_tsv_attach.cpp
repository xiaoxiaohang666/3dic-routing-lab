// Read-only diagnostic: for every cross-die net of a saved Guide, the attachment
// length of its actual TSV cell, with the same definition as the TSV frontier
// tool (scripts/tsv_heat_frontier.cpp on codex/tsv-heat-assignment-20261106):
// distance from the landing to the bottom-die pin MST plus distance to the
// top-die pin MST, "distance to the tree" being the distance to the nearest MST
// edge's bounding box. Comparing it with that tool's smallest legal landing
// bounds how much wire a TSV move alone could save (an estimate, not routing).
#include "parser/def_parser.hpp"
#include "parser/lef_parser.hpp"
#include "router/routing_graph.hpp"
#include "scoring/compat_validation.hpp"
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
    std::map<std::string,std::string> a;for(int i=1;i+1<argc;i+=2) a[argv[i]]=argv[i+1];
    const std::string output=a.at("--output");if(fs::exists(output)) throw std::runtime_error("output must be new");
    LefParser lef;if(!lef.parse(a.at("--lef"))) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(a.at("--def"),lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,true});
    const auto& grid=graph.grid();
    const auto guides=compat::read_guides(a.at("--guide"));
    std::ofstream out(output);if(!out) throw std::runtime_error("cannot create output");
    out<<std::setprecision(10)<<"net,pins,bottom,top,forced,col,row,attach_dbu\n";
    for(const auto& net:def.nets()) {
        if(!net.is_cross_die) continue;
        std::vector<Point> side[2];std::set<std::string> names;bool forced=false;
        for(const auto& p:net.pins) {
            if(p.layer_idx==10||p.layer_idx==11) forced=true;
            if(names.insert(p.name).second) side[p.layer_idx>=11].push_back(p.coord);
        }
        out<<net.name<<','<<net.pins.size()<<','<<side[0].size()<<','<<side[1].size()<<','<<int(forced);
        const auto found=guides.find(net.name);
        if(side[0].empty()||side[1].empty()||found==guides.end()) {out<<",,,\n";continue;}
        // The TSV is the cell where an M10 and an M11 rectangle overlap.
        int col=-1,row=-1;
        for(const auto& g:found->second) if(get_layer_index(g.layer)==10)
            for(const auto& h:found->second) if(get_layer_index(h.layer)==11) {
                const double x1=std::max(g.x1,h.x1),x2=std::min(g.x2,h.x2),y1=std::max(g.y1,h.y1),y2=std::min(g.y2,h.y2);
                if(x2>x1 && y2>y1 && col<0) {col=grid.get_col(0.5*(x1+x2));row=grid.get_row(0.5*(y1+y2));}
            }
        if(col<0) {out<<",,,\n";continue;}
        const double x=0.5*(grid.x_lines[col]+grid.x_lines[col+1]),y=0.5*(grid.y_lines[row]+grid.y_lines[row+1]);
        const auto e0=mst_edges(side[0]),e1=mst_edges(side[1]);
        out<<','<<col<<','<<row<<','<<attach(side[0],e0,x,y)+attach(side[1],e1,x,y)<<'\n';
    }
    return 0;
 } catch(const std::exception& e) {std::cerr<<"[ERROR] "<<e.what()<<'\n';return 1;}
}
