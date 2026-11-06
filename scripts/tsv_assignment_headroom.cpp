// Read-only estimate: how much could moving the TSVs of many cross-die nets
// together gain over moving one net at a time? Nothing is routed and no Guide
// is written.
//
// The Guide gives each cross-die net's TSV cell (official selection), every
// net's M10/M11 demand and, with the given L, reference, C and G, the current
// score. Moving net i's TSV from cell s to cell t changes its wire by
// a(t)-a(s), where a(p) is the distance from p to the bottom-die pin MST plus
// the distance to the top-die pin MST (nearest edge bounding box, as in
// tsv_heat_frontier). Routing detours, layer directions and vias are ignored,
// so the wire change is an estimate. For each heat price lambda (cells of wire
// per standard deviation of heat) a cell is worth lambda*heat - a. Compared at
// every lambda:
//   ub-any, ub-free  every net takes its own best cell; cells may be shared
//   seq-free         one net at a time takes its best free cell not used by
//                    another TSV, repeated until no net improves
//   joint-free       best one-to-one assignment over every net's top cells
//   joint-any        the same, ignoring other nets' wires on M10/M11
//                    (both joint methods skipped above --joint-max-nets)
// "free" means the landing tracks fit on M10 and M11 together with the other
// nets' ordinary demand and fixed objects, without overflow. Every solution is
// scored with the exact Pearson correlation and the official formulas; L moves
// by the summed wire changes and C is kept.
#include "objectives/net_cell_contribution.hpp"
#include "objectives/official_metrics.hpp"
#include "parser/def_parser.hpp"
#include "parser/lef_parser.hpp"
#include "parser/thermal_parser.hpp"
#include "router/routing_graph.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>
using namespace eda;
namespace fs=std::filesystem;
namespace {
struct Edge {double x1,y1,x2,y2;};
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
struct Cand {int cell;double a,h,v;};
struct XNet {
    size_t index{};int cur{-1};double cur_a{};
    std::vector<Point> side[2];std::vector<Edge> edges[2];
    std::unordered_map<int,int> own; // node -> this net's ordinary M10/M11 demand
};
// Max-value assignment of every net to a distinct cell (successive shortest
// paths, Bellman-Ford queue). Each list holds the net's current cell, so the
// current assignment proves a complete one exists.
std::vector<int> assign(const std::vector<std::vector<Cand>>& lists) {
    std::unordered_map<int,int> id;std::vector<int> cells;
    for(const auto& l:lists) for(const auto& c:l) if(id.emplace(c.cell,int(cells.size())).second) cells.push_back(c.cell);
    const int n=int(lists.size()),m=int(cells.size()),S=n+m,T=n+m+1,V=n+m+2;
    struct E {int to,rev,cap;double cost;};
    std::vector<std::vector<E>> g(V);
    auto add=[&](int u,int v,double cost) {
        g[u].push_back({v,int(g[v].size()),1,cost});g[v].push_back({u,int(g[u].size())-1,0,-cost});
    };
    for(int i=0;i<n;++i) add(S,i,0);
    for(int i=0;i<n;++i) for(const auto& c:lists[i]) add(i,n+id.at(c.cell),-c.v);
    for(int j=0;j<m;++j) add(n+j,T,0);
    std::vector<double> dist(V);std::vector<int> pv(V),pe(V);std::vector<char> inq(V);
    for(int flow=0;flow<n;++flow) {
        std::fill(dist.begin(),dist.end(),std::numeric_limits<double>::infinity());dist[S]=0;
        std::fill(inq.begin(),inq.end(),0);std::deque<int> q{S};inq[S]=1;
        while(!q.empty()) {
            const int u=q.front();q.pop_front();inq[u]=0;
            for(int k=0;k<int(g[u].size());++k) {
                const auto& e=g[u][k];if(!e.cap) continue;
                const double nd=dist[u]+e.cost;
                if(nd<dist[e.to]-1e-3) {dist[e.to]=nd;pv[e.to]=u;pe[e.to]=k;if(!inq[e.to]) {inq[e.to]=1;q.push_back(e.to);}}
            }
        }
        if(!std::isfinite(dist[T])) throw std::runtime_error("no complete assignment");
        for(int v=T;v!=S;v=pv[v]) {auto& e=g[pv[v]][pe[v]];e.cap-=1;g[v][e.rev].cap+=1;}
    }
    std::vector<int> out(n,-1);
    for(int i=0;i<n;++i) for(const auto& e:g[i]) if(e.to>=n&&e.to<n+m&&e.cap==0) out[i]=cells[e.to-n];
    for(int c:out) if(c<0) throw std::logic_error("assignment left a net unmatched");
    return out;
}
std::vector<double> parse_list(const std::string& s) {
    std::vector<double> out;std::stringstream in(s);std::string item;
    while(std::getline(in,item,',')) if(!item.empty()) out.push_back(std::stod(item));
    if(out.empty()) throw std::runtime_error("empty lambda list");
    return out;
}
}
int main(int argc,char** argv) {
 try {
    std::string def_file,thermal_file,guide_file,output,fixed_model="observed-20261104",lambda_text="25,50,75,100,125,150,200,300,500,800";
    std::vector<std::string> lefs;size_t top=48,joint_max=4000;int margin=1<<20,passes=100;
    double gain=0,length=0,reference=0,congestion=-1;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];if(i+1>=argc) throw std::runtime_error("expected option value for "+key);
        const std::string value=argv[++i];
        if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);
        else if(key=="--thermal") thermal_file=value;else if(key=="--guide") guide_file=value;
        else if(key=="--output") output=value;else if(key=="--fixed-resource-model") fixed_model=value;
        else if(key=="--lambda-cells") lambda_text=value;else if(key=="--top") top=std::stoul(value);
        else if(key=="--margin") margin=std::stoi(value);else if(key=="--passes") passes=std::stoi(value);
        else if(key=="--joint-max-nets") joint_max=std::stoul(value);
        else if(key=="--thermal-gain") gain=std::stod(value);else if(key=="--length") length=std::stod(value);
        else if(key=="--reference") reference=std::stod(value);else if(key=="--congestion") congestion=std::stod(value);
        else throw std::runtime_error("unknown option "+key);
    }
    if(def_file.empty()||lefs.empty()||thermal_file.empty()||guide_file.empty()||output.empty()||
       !(gain>0)||!(length>0)||!(reference>0)||congestion<0||top<1)
        throw std::runtime_error("required --def --lef --thermal --guide --output --thermal-gain --length --reference --congestion");
    if(fixed_model!="production"&&fixed_model!="observed-20261104") throw std::runtime_error("invalid fixed resource model");
    if(fs::exists(output)) throw std::runtime_error("output must be new");
    const auto lambdas=parse_list(lambda_text);
    LefParser lef;for(const auto& f:lefs) if(!lef.parse(f)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,fixed_model=="observed-20261104"});
    const auto& grid=graph.grid();const int cols=grid.cols,rows=grid.rows;const size_t N=size_t(cols)*rows;
    ThermalParser thermal;if(!thermal.parse_json(thermal_file,grid,def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))
        throw std::runtime_error("thermal parse failed");
    const auto guides=compat::read_guides(guide_file);
    if(guides.size()!=def.nets().size()) throw std::runtime_error("Guide net count mismatch");
    OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;
    mo.snap_reference_coordinates=true;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();mo.thermal_gain=gain;
    const double pitch=std::max(def.gcell_pitch_x(),def.gcell_pitch_y());
    std::vector<double> heat(N);long double hs=0,hss=0;
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
        const double h=std::max(0.0,thermal.get_temperature(c,r)-thermal.ambient_temperature());
        heat[size_t(r)*cols+c]=h;hs+=h;
    }
    const double hmean=double(hs/N);for(double h:heat) hss+=(h-hmean)*(h-hmean);
    const double hsd=std::sqrt(double(hss/N));if(!(hsd>0)) throw std::runtime_error("flat heat map");
    auto cx=[&](int c){return 0.5*(grid.x_lines[c]+grid.x_lines[c+1]);};
    auto cy=[&](int r){return 0.5*(grid.y_lines[r]+grid.y_lines[r+1]);};
    // Ordinary M10/M11 demand of all nets (TSV landing reservations removed) and TSV cells.
    std::vector<int> ord(graph.size(),0),base_counts(N,0);std::vector<XNet> xs;size_t fixed_tsv=0,multi=0;
    for(size_t i=0;i<def.nets().size();++i) {
        const auto& net=def.nets()[i];
        const auto found=guides.find(net.name);if(found==guides.end()) throw std::runtime_error("Guide misses net "+net.name);
        const auto measured=measure_official_net_guides(net,found->second,graph,lef,mo,0);
        std::set<int> reserved;
        for(int cell:measured.tsv_cells) for(int l:{10,11}) reserved.insert(graph.node(cell%cols,cell/cols,l));
        XNet x;x.index=i;
        for(auto [node,amount]:measured.demand) {
            const int l=graph.layer(node);if((l!=10&&l!=11)||reserved.count(node)) continue;
            ord[node]+=amount;if(net.is_cross_die) x.own[node]+=amount;
        }
        if(!net.is_cross_die) continue;
        std::set<std::string> names;
        for(const auto& p:net.pins) if(names.insert(p.name).second) x.side[p.layer_idx>=11].push_back(p.coord);
        if(measured.tsv_cells.size()!=1||x.side[0].empty()||x.side[1].empty()) {
            multi+=measured.tsv_cells.size()!=1;
            for(int cell:measured.tsv_cells) {++base_counts[size_t(cell)];++fixed_tsv;}
            continue;
        }
        x.cur=measured.tsv_cells.front();
        for(int s=0;s<2;++s) x.edges[s]=mst_edges(x.side[s]);
        const int c=x.cur%cols,r=x.cur/cols;
        x.cur_a=attach(x.side[0],x.edges[0],cx(c),cy(r))+attach(x.side[1],x.edges[1],cx(c),cy(r));
        xs.push_back(std::move(x));
    }
    std::vector<char> legal(N,0),free_all(N,0);std::vector<int> land10(N,0),land11(N,0);size_t legal_n=0,free_n=0;
    const auto fits_with=[&](int cell,const std::unordered_map<int,int>* own)->bool {
        const int c=cell%cols,r=cell/cols;
        for(int l:{10,11}) {
            const int node=graph.node(c,r,l);if(node<0) return false;
            int other=ord[node];
            if(own) {const auto it=own->find(node);if(it!=own->end()) other-=it->second;}
            const int landing=l==10?land10[size_t(cell)]:land11[size_t(cell)];
            if(official_resource_cell_contribution(graph,node,other+landing).overflow) return false;
        }
        return true;
    };
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
        const size_t cell=size_t(r)*cols+c;
        legal[cell]=graph.tsv_landing_legal(c,r,TsvObstacleScope::INTERFACE);if(!legal[cell]) continue;++legal_n;
        const Point p{cx(c),cy(r)};land10[cell]=graph.tsv_landing_tracks(p,10);land11[cell]=graph.tsv_landing_tracks(p,11);
        free_all[cell]=fits_with(int(cell),nullptr);free_n+=free_all[cell];
    }
    const size_t K=xs.size(),L=lambdas.size();
    std::vector<int> counts=base_counts;for(const auto& x:xs) ++counts[size_t(x.cur)];
    const double rho0=PearsonState(heat,counts).correlation();
    const auto score=[&](double rho,double len,double& W,double& T){
        W=official_wirelength_score(len,reference,mo);T=100.0*(0.5+0.5*std::tanh(gain*rho));
        return 0.45*W+0.30*congestion+0.25*T;
    };
    double W0,T0;const double S0=score(rho0,length,W0,T0);
    // Candidate lists per lambda: [lambda][net] best cells, current cell included.
    std::vector<std::vector<std::vector<Cand>>> any(L,std::vector<std::vector<Cand>>(K)),fre(L,std::vector<std::vector<Cand>>(K));
    double detour_any=0,detour_free=0,cur_heat_sd=0;size_t cur_not_free=0,cur_not_legal=0;
    for(size_t k=0;k<K;++k) {
        auto& x=xs[k];
        double x1=std::numeric_limits<double>::infinity(),y1=x1,x2=-x1,y2=-x1;
        for(const auto& s:x.side) for(const auto& p:s) {x1=std::min(x1,p.x);y1=std::min(y1,p.y);x2=std::max(x2,p.x);y2=std::max(y2,p.y);}
        const int c1=std::max(0,grid.get_col(x1)-margin),c2=std::min(cols-1,grid.get_col(x2)+margin);
        const int r1=std::max(0,grid.get_row(y1)-margin),r2=std::min(rows-1,grid.get_row(y2)+margin);
        std::vector<Cand> all;std::vector<char> fit;double min_any=x.cur_a,min_free=x.cur_a;
        for(int r=r1;r<=r2;++r) for(int c=c1;c<=c2;++c) {
            const int cell=r*cols+c;if(!legal[size_t(cell)]||base_counts[size_t(cell)]||cell==x.cur) continue;
            const double a=attach(x.side[0],x.edges[0],cx(c),cy(r))+attach(x.side[1],x.edges[1],cx(c),cy(r));
            all.push_back({cell,a,heat[size_t(cell)],0});
            const bool f=free_all[size_t(cell)]||(!x.own.empty()&&fits_with(cell,&x.own));
            fit.push_back(f);min_any=std::min(min_any,a);if(f) min_free=std::min(min_free,a);
        }
        detour_any+=x.cur_a-min_any;detour_free+=x.cur_a-min_free;
        cur_heat_sd+=(heat[size_t(x.cur)]-hmean)/hsd;
        cur_not_legal+=!legal[size_t(x.cur)];cur_not_free+=!fits_with(x.cur,&x.own);
        std::vector<Cand> fits;for(size_t j=0;j<all.size();++j) if(fit[j]) fits.push_back(all[j]);
        for(size_t li=0;li<L;++li) {
            const double lam=lambdas[li]*pitch/hsd;
            const auto pick=[&](std::vector<Cand>& pool,std::vector<Cand>& out) {
                for(auto& c:pool) c.v=lam*c.h-c.a;
                const size_t n=std::min(top,pool.size());
                std::nth_element(pool.begin(),pool.begin()+std::ptrdiff_t(n),pool.end(),[](const Cand& a,const Cand& b){return a.v>b.v;});
                out.assign(pool.begin(),pool.begin()+std::ptrdiff_t(n));
                out.push_back({x.cur,x.cur_a,heat[size_t(x.cur)],lam*heat[size_t(x.cur)]-x.cur_a});
                std::sort(out.begin(),out.end(),[](const Cand& a,const Cand& b){return a.v>b.v;});
            };
            pick(all,any[li][k]);pick(fits,fre[li][k]);
        }
    }
    std::ofstream out(output);if(!out) throw std::runtime_error("cannot create output");
    out<<std::setprecision(10);
    out<<"{\"N\":"<<N<<",\"cross_nets_movable\":"<<K<<",\"fixed_tsvs\":"<<fixed_tsv<<",\"nets_not_one_tsv\":"<<multi
       <<",\"legal_cells\":"<<legal_n<<",\"free_legal_cells\":"<<free_n<<",\"heat_mean\":"<<hmean<<",\"heat_sd\":"<<hsd
       <<",\"pitch\":"<<pitch<<",\"top\":"<<top<<",\"margin\":"<<margin<<",\"joint_skipped\":"<<(K>joint_max)
       <<",\"rho\":"<<rho0<<",\"W\":"<<W0<<",\"T\":"<<T0<<",\"S\":"<<S0
       <<",\"current_mean_heat_sd\":"<<cur_heat_sd/std::max<size_t>(K,1)
       <<",\"model_detour_cells_any\":"<<detour_any/pitch<<",\"model_detour_cells_free\":"<<detour_free/pitch
       <<",\"current_not_legal\":"<<cur_not_legal<<",\"current_not_free\":"<<cur_not_free<<",\"results\":[\n";
    bool first=true;
    std::unordered_map<int,size_t> old_owner;for(size_t k=0;k<K;++k) old_owner[xs[k].cur]=k;
    for(size_t li=0;li<L;++li) {
        const double lam=lambdas[li]*pitch/hsd;
        const auto report=[&](const char* name,const std::vector<Cand>& chosen) {
            std::vector<int> cnt=base_counts;double dA=0,dH=0;size_t moved=0,shared=0,chains=0;
            for(size_t k=0;k<K;++k) {
                if(++cnt[size_t(chosen[k].cell)]>1) ++shared;
                if(chosen[k].cell!=xs[k].cur) {const auto it=old_owner.find(chosen[k].cell);if(it!=old_owner.end()&&it->second!=k) ++chains;}
                dA+=chosen[k].a-xs[k].cur_a;dH+=chosen[k].h-heat[size_t(xs[k].cur)];moved+=chosen[k].cell!=xs[k].cur;
            }
            const double rho=PearsonState(heat,cnt).correlation();double W,T;const double S=score(rho,length+dA,W,T);
            out<<(first?"":",\n")<<"{\"lambda_cells_per_sd\":"<<lambdas[li]<<",\"method\":\""<<name<<"\",\"moved\":"<<moved
               <<",\"cells_shared\":"<<shared<<",\"moves_into_vacated_cells\":"<<chains
               <<",\"wire_change_cells\":"<<dA/pitch<<",\"heat_change_sd\":"<<dH/hsd
               <<",\"rho\":"<<rho<<",\"W\":"<<W<<",\"T\":"<<T<<",\"S\":"<<S<<",\"dS\":"<<S-S0<<'}';
            first=false;
        };
        std::vector<Cand> ub_any(K),ub_free(K);
        for(size_t k=0;k<K;++k) {ub_any[k]=any[li][k].front();ub_free[k]=fre[li][k].front();}
        report("ub-any",ub_any);report("ub-free",ub_free);
        // One net at a time.
        std::vector<int> occ(N,-1);for(size_t cell=0;cell<N;++cell) if(base_counts[cell]) occ[cell]=-2;
        std::vector<Cand> now(K);
        for(size_t k=0;k<K;++k) {occ[size_t(xs[k].cur)]=int(k);now[k]={xs[k].cur,xs[k].cur_a,heat[size_t(xs[k].cur)],lam*heat[size_t(xs[k].cur)]-xs[k].cur_a};}
        for(int pass=0;pass<passes;++pass) {
            bool changed=false;
            for(size_t k=0;k<K;++k) for(const auto& c:fre[li][k]) {
                if(c.v<=now[k].v+1e-6||c.cell==now[k].cell) break;
                if(occ[size_t(c.cell)]!=-1) continue;
                occ[size_t(now[k].cell)]=-1;occ[size_t(c.cell)]=int(k);now[k]=c;changed=true;break;
            }
            if(!changed) break;
        }
        report("seq-free",now);
        for(int variant=0;variant<2&&K<=joint_max;++variant) {
            const auto& lists=variant?any[li]:fre[li];
            const auto cells=assign(lists);
            std::vector<Cand> chosen(K);
            for(size_t k=0;k<K;++k) {
                for(const auto& c:lists[k]) if(c.cell==cells[k]) {chosen[k]=c;break;}
                if(base_counts[size_t(cells[k])]) throw std::logic_error("assignment uses a fixed TSV cell");
            }
            report(variant?"joint-any":"joint-free",chosen);
        }
    }
    out<<"\n]}\n";
    return 0;
 } catch(const std::exception& e) {std::cerr<<"[ERROR] "<<e.what()<<'\n';return 1;}
}
