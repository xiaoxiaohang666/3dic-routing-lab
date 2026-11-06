// Read-only look at every overflowing cell of a Guide: which nets put demand
// there and in what shape, whether the cell holds pins or is next to TSVs, how
// much room the neighbourhood and the other layers have, and whether each net
// has a simple way around (shift its run one row/column, move its run to the
// next layer of the same direction, or move its layer-change stack one cell).
// Never writes a Guide and never linked into the router. The "simple way
// around" checks ignore connections at the ends of a run, so they show where
// room exists, not that a legal reroute exists.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
using namespace eda;
namespace fs=std::filesystem;
namespace {
int die_low(int layer){return layer<=10?1:11;}
int die_high(int layer){return layer<=10?10:20;}
struct Count {uint64_t n{};double q{};};
void add(std::map<std::string,Count>& m,const std::string& k,double q){auto& c=m[k];++c.n;c.q+=q;}
void dump(std::ostream& s,const char* name,const std::map<std::string,Count>& m) {
    s<<'"'<<name<<"\":{";bool first=true;
    for(const auto& [k,v]:m){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"n\":"<<v.n<<",\"Q_share\":"<<v.q<<'}';}
    s<<'}';
}
}
int main(int argc,char** argv) {
 try {
    std::string def_file,guide_file,fixed_model="documented";std::vector<std::string> lefs;fs::path out;
    for(int i=1;i<argc;++i) {
        const std::string k=argv[i];if(i+1==argc) throw std::runtime_error("missing value for "+k);
        const std::string v=argv[++i];
        if(k=="--def") def_file=v;else if(k=="--lef") lefs.push_back(v);else if(k=="--guide") guide_file=v;
        else if(k=="--fixed-resource-model") fixed_model=v;else if(k=="--output-dir") out=v;
        else throw std::runtime_error("unknown option "+k);
    }
    if(def_file.empty()||lefs.empty()||guide_file.empty()||out.empty()) throw std::runtime_error("need --def --lef --guide --output-dir");
    if(fixed_model!="documented"&&fixed_model!="observed-20261104") throw std::runtime_error("bad --fixed-resource-model");
    if(fs::exists(out)) throw std::runtime_error("output directory must be new");
    fs::create_directories(out);
    LefParser lef;for(const auto& f:lefs) if(!lef.parse(f)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,fixed_model=="observed-20261104"});
    const auto guides=compat::read_guides(guide_file);
    if(guides.size()!=def.nets().size()) throw std::runtime_error("Guide net count mismatch");
    OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;
    mo.snap_reference_coordinates=true;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
    const auto& grid=graph.grid();
    std::vector<int> demand(graph.size());
    std::vector<std::vector<std::pair<int,int>>> net_demand(def.nets().size());
    for(size_t i=0;i<def.nets().size();++i) {
        const auto found=guides.find(def.nets()[i].name);
        if(found==guides.end()) throw std::runtime_error("Guide misses net "+def.nets()[i].name);
        auto measured=measure_official_net_guides(def.nets()[i],found->second,graph,lef,mo,0);
        for(auto [node,amount]:measured.demand) demand[node]+=amount;
        net_demand[i]=std::move(measured.demand);
    }
    std::vector<double> cell_q(graph.size());double total_q=0;uint64_t total_overflow=0;std::vector<int> over_nodes;
    for(size_t n=0;n<graph.size();++n) {
        const auto c=official_resource_cell_contribution(graph,int(n),demand[n]);
        if(!c.overflow) continue;
        cell_q[n]=double(c.squared);total_q+=cell_q[n];total_overflow+=c.overflow;over_nodes.push_back(int(n));
    }
    const auto spare=[&](int node)->double{
        if(node<0||graph.official_capacity(node)<=0) return 0;
        return graph.official_capacity(node)-double(graph.official_obstacle_demand(node))-demand[node];
    };
    const auto horizontal=[&](int layer){return lef.get_layer_direction(get_layer_name(layer))==Direction::HORIZONTAL;};
    // TSV cells: a net covering the same cell on M10 and M11.
    std::set<std::pair<int,int>> tsv_cells;std::vector<std::pair<int,int>> net_tsv(def.nets().size(),{-1,-1});
    std::vector<std::unordered_map<int,std::array<int,4>>> net_runs(def.nets().size());  // node -> run extent cb,ce,rb,re
    std::vector<char> touches(def.nets().size(),0);
    for(size_t i=0;i<def.nets().size();++i) {
        for(auto [node,amount]:net_demand[i]) {(void)amount;if(cell_q[node]>0){touches[i]=1;break;}}
        std::set<std::pair<int,int>> m10,m11;
        for(const auto& s:guides.at(def.nets()[i].name)) {
            const int l=get_layer_index(s.layer);if(l!=10&&l!=11&&!touches[i]) continue;
            const int cb=std::max(0,int(std::upper_bound(grid.x_lines.begin(),grid.x_lines.end(),s.x1)-grid.x_lines.begin())-1);
            const int ce=std::min(grid.cols,int(std::lower_bound(grid.x_lines.begin(),grid.x_lines.end(),s.x2)-grid.x_lines.begin()));
            const int rb=std::max(0,int(std::upper_bound(grid.y_lines.begin(),grid.y_lines.end(),s.y1)-grid.y_lines.begin())-1);
            const int re=std::min(grid.rows,int(std::lower_bound(grid.y_lines.begin(),grid.y_lines.end(),s.y2)-grid.y_lines.begin()));
            for(int r=rb;r<re;++r) for(int c=cb;c<ce;++c) {
                if(l==10) m10.emplace(c,r);
                if(l==11) m11.emplace(c,r);
                if(touches[i]) {
                    const int node=graph.node(c,r,l);
                    auto it=net_runs[i].find(node);
                    const int len=(ce-cb)*(re-rb);
                    if(it==net_runs[i].end()||len>(it->second[1]-it->second[0])*(it->second[3]-it->second[2])) net_runs[i][node]={cb,ce,rb,re};
                }
            }
        }
        for(const auto& p:m10) if(m11.count(p)) {tsv_cells.insert(p);net_tsv[i]=p;break;}
    }
    const auto tsv_near=[&](int c,int r,int radius){int n=0;for(int dr=-radius;dr<=radius;++dr)for(int dc=-radius;dc<=radius;++dc) n+=tsv_cells.count({c+dc,r+dr});return n;};
    // Cell table.
    std::ofstream cells(out/"cells.csv");cells.exceptions(std::ios::badbit|std::ios::failbit);cells<<std::setprecision(10);
    cells<<"layer,col,row,capacity,fixed,guide,overflow,Q,pin_here,tsv_here,tsv_r1,tsv_r2,nets,room_r1,room_r2,room_r4,spare_r2,spare_up1,spare_down1,spare_up2,spare_down2\n";
    std::map<std::string,Count> by_layer,fixed_level,tsv_zone,neigh;
    std::unordered_map<int,std::vector<size_t>> cell_nets;
    for(size_t i=0;i<def.nets().size();++i) if(touches[i]) for(auto [node,amount]:net_demand[i]) if(cell_q[node]>0&&amount>0) cell_nets[node].push_back(i);
    for(int node:over_nodes) {
        const int l=graph.layer(node),c=graph.col(node),r=graph.row(node);
        const double cap=graph.official_capacity(node);const uint32_t fixed=graph.official_obstacle_demand(node);
        const auto c0=official_resource_cell_contribution(graph,node,demand[node]);
        int room[3]={0,0,0};double spare2=0;const int radii[3]={1,2,4};
        for(int k=0;k<3;++k) for(int dr=-radii[k];dr<=radii[k];++dr) for(int dc=-radii[k];dc<=radii[k];++dc) {
            if(!dc&&!dr) continue;const int nb=graph.node(c+dc,r+dr,l);if(nb<0) continue;
            if(spare(nb)>=1) ++room[k];
            if(k==1) spare2+=std::max(0.0,spare(nb));
        }
        const auto at=[&](int y){return (y<die_low(l)||y>die_high(l))?-1.0:spare(graph.node(c,r,y));};
        const bool pin_here=graph.tsv_obstacle_layers(c,r,TSV_MACRO_PIN|TSV_IO_PIN)>>l&1u;
        const int t0=tsv_cells.count({c,r}),t1=tsv_near(c,r,1),t2=tsv_near(c,r,2);
        cells<<l<<','<<c<<','<<r<<','<<cap<<','<<fixed<<','<<demand[node]<<','<<c0.overflow<<','<<cell_q[node]<<','<<pin_here<<','<<t0<<','<<t1<<','<<t2<<','
             <<cell_nets[node].size()<<','<<room[0]<<','<<room[1]<<','<<room[2]<<','<<spare2<<','<<at(l+1)<<','<<at(l-1)<<','<<at(l+2)<<','<<at(l-2)<<'\n';
        const std::string L="M"+std::to_string(l);
        add(by_layer,L,cell_q[node]);
        const double ff=cap>0?double(fixed)/cap:0;
        add(fixed_level,L+(ff==0?":fixed_0":ff<0.34?":fixed_lt_1/3":ff<0.67?":fixed_lt_2/3":ff<1?":fixed_lt_full":":fixed_full"),cell_q[node]);
        add(tsv_zone,L+(t0?":tsv_in_cell":t1?":tsv_within_1":t2?":tsv_within_2":":no_tsv_within_2"),cell_q[node]);
        add(neigh,L+(room[0]==0?":no_room_r1":room[0]<=2?":room_r1_1to2":":room_r1_3plus"),cell_q[node]);
    }
    // Net-in-cell table.
    std::ofstream nets(out/"cell_nets.csv");nets.exceptions(std::ios::badbit|std::ios::failbit);nets<<std::setprecision(10);
    nets<<"layer,col,row,net,pins,cross_die,amount,shape,run_cells,own_pin_this_layer,own_pin_this_column,tsv_dist,bbox_hp,shift_run_room,other_layer_run_room,stack_move1_room\n";
    // One JSON line per (cell, net): pins (access point cell, shape cells), exempt cells, the net's Guide in cells,
    // and spare tracks around every Guide cell, so the choice can be replayed by hand.
    std::ofstream detail(out/"cell_nets_detail.jsonl");detail.exceptions(std::ios::badbit|std::ios::failbit);detail<<std::setprecision(10);
    std::map<std::string,Count> shape,pin_rel,fix_kind,tsv_dist_k,cross;
    for(int node:over_nodes) {
        const int l=graph.layer(node),c=graph.col(node),r=graph.row(node);
        const auto& list=cell_nets[node];
        for(size_t i:list) {
            const auto& net=def.nets()[i];int amount=0;for(auto [nd,a]:net_demand[i]) if(nd==node) amount+=a;
            const double share=cell_q[node]*amount/std::max(demand[node],1);
            const auto pin_list=graph.own_pin_nodes(net);std::set<int> pins(pin_list.begin(),pin_list.end());
            bool lowdie=false,highdie=false;int pc0=grid.cols,pc1=-1,pr0=grid.rows,pr1=-1;
            for(int p:pins){(graph.layer(p)<=10?lowdie:highdie)=true;pc0=std::min(pc0,graph.col(p));pc1=std::max(pc1,graph.col(p));pr0=std::min(pr0,graph.row(p));pr1=std::max(pr1,graph.row(p));}
            const bool crossdie=lowdie&&highdie;
            const bool pin_layer=pins.count(node);bool pin_col=false;
            for(int y=die_low(l);y<=die_high(l);++y) pin_col=pin_col||pins.count(graph.node(c,r,y));
            const auto& runs=net_runs[i];
            const auto run_it=runs.find(node);
            const auto covered=[&](int c2,int r2,int y){return y>=die_low(l)&&y<=die_high(l)&&runs.count(graph.node(c2,r2,y));};
            const bool up=covered(c,r,l+1),down=covered(c,r,l-1);
            int run_cells=1;std::array<int,4> ext{c,c+1,r,r+1};
            if(run_it!=runs.end()){ext=run_it->second;run_cells=(ext[1]-ext[0])*(ext[3]-ext[2]);}
            const bool planar=run_cells>1;
            const std::string sh=planar?(up||down?"run_with_layer_change":"run_passing_through"):(up&&down?"stack_passing_through":(up||down?"stack_end":"single_cell"));
            // Simple room checks.
            const auto room_ok=[&](int c2,int r2,int y){const int nb=graph.node(c2,r2,y);if(nb<0) return false;if(runs.count(nb)) return true;return spare(nb)>=1;};
            int shift_ok=0;
            if(planar) {
                const bool h=horizontal(l);
                for(int d:{-1,1}) {
                    bool ok=true;
                    for(int rr=ext[2];rr<ext[3]&&ok;++rr) for(int cc=ext[0];cc<ext[1]&&ok;++cc) ok=room_ok(cc+(h?0:d),rr+(h?d:0),l);
                    if(ok) shift_ok=1;
                }
            }
            int layer_ok=0;
            if(planar) for(int y:{l-2,l+2}) {
                if(y<die_low(l)||y>die_high(l)) continue;
                bool ok=true;for(int rr=ext[2];rr<ext[3]&&ok;++rr) for(int cc=ext[0];cc<ext[1]&&ok;++cc) ok=room_ok(cc,rr,y);
                if(ok) layer_ok=1;
            }
            int stack_ok=0;
            if(!planar) {
                int lo=l,hi=l;while(covered(c,r,lo-1)) --lo;while(covered(c,r,hi+1)) ++hi;
                for(int dr=-1;dr<=1&&!stack_ok;++dr) for(int dc=-1;dc<=1&&!stack_ok;++dc) {
                    if(!dc&&!dr) continue;bool ok=true;for(int y=lo;y<=hi&&ok;++y) ok=room_ok(c+dc,r+dr,y);if(ok) stack_ok=1;
                }
            }
            int tdist=-1;if(net_tsv[i].first>=0) tdist=std::max(std::abs(net_tsv[i].first-c),std::abs(net_tsv[i].second-r));
            {
                detail<<"{\"cell\":["<<l<<','<<c<<','<<r<<"],\"net\":\""<<net.name<<"\",\"pins\":[";
                for(size_t k=0;k<net.pins.size();++k) {
                    const auto& pn=net.pins[k];if(k) detail<<',';
                    detail<<"{\"name\":\""<<pn.name<<"\",\"layer\":"<<pn.layer_idx<<",\"coord\":["<<pn.coord.x<<','<<pn.coord.y<<"],\"coord_cell\":["<<grid.get_col(pn.coord.x)<<','<<grid.get_row(pn.coord.y)<<"],\"shapes\":[";
                    for(size_t q=0;q<pn.port_rects.size();++q) {const auto& sh=pn.port_rects[q];if(q) detail<<',';
                        detail<<'['<<sh.layer_idx<<','<<sh.rect.x1<<','<<sh.rect.y1<<','<<sh.rect.x2<<','<<sh.rect.y2<<']';}
                    detail<<"]}";
                }
                detail<<"],\"exempt\":[";bool f1=true;
                for(int pnode:pins){if(!f1)detail<<',';f1=false;detail<<'['<<graph.layer(pnode)<<','<<graph.col(pnode)<<','<<graph.row(pnode)<<']';}
                detail<<"],\"guide_cells\":[";bool f2=true;
                for(auto [gn,ga]:net_demand[i]){if(!f2)detail<<',';f2=false;
                    detail<<'['<<graph.layer(gn)<<','<<graph.col(gn)<<','<<graph.row(gn)<<','<<ga<<','<<graph.official_capacity(gn)<<','<<graph.official_obstacle_demand(gn)<<','<<demand[gn]<<']';}
                detail<<"],\"around\":[";bool f3=true;
                // Spare tracks on the net's die layers in the 5x5 box around the overflow cell.
                for(int y=die_low(l);y<=std::min(die_high(l),die_low(l)+3);++y) for(int dr=-2;dr<=2;++dr) for(int dc=-2;dc<=2;++dc) {
                    const int nb=graph.node(c+dc,r+dr,y);if(nb<0) continue;if(!f3)detail<<',';f3=false;
                    detail<<'['<<y<<','<<c+dc<<','<<r+dr<<','<<graph.official_capacity(nb)<<','<<graph.official_obstacle_demand(nb)<<','<<demand[nb]<<']';}
                detail<<"]}\n";
            }
            nets<<l<<','<<c<<','<<r<<','<<net.name<<','<<net.pins.size()<<','<<crossdie<<','<<amount<<','<<sh<<','<<run_cells<<','<<pin_layer<<','<<pin_col<<','<<tdist<<','
                <<(pc1-pc0)+(pr1-pr0)<<','<<shift_ok<<','<<layer_ok<<','<<stack_ok<<'\n';
            const std::string L="M"+std::to_string(l);
            add(shape,L+":"+sh,share);
            add(pin_rel,L+(pin_layer?":own_pin_on_this_layer":pin_col?":own_pin_other_layer_same_cell":":no_own_pin_in_cell"),share);
            add(fix_kind,L+(planar?(shift_ok?":run_shift_has_room":layer_ok?":run_other_layer_has_room":":run_no_simple_room"):(stack_ok?":stack_move1_has_room":":stack_no_simple_room")),share);
            add(cross,L+(crossdie?":cross_die":":same_die"),share);
            if(crossdie) add(tsv_dist_k,L+(tdist<0?":tsv_unknown":tdist==0?":tsv_in_cell":tdist<=2?":tsv_1to2":tdist<=8?":tsv_3to8":":tsv_far"),share);
        }
    }
    std::ofstream s(out/"summary.json");s.exceptions(std::ios::badbit|std::ios::failbit);s<<std::setprecision(10);
    s<<"{\"scope\":\"read-only per-cell overflow breakdown; offline, not an official result; room checks ignore run-end connections\",\"fixed_resource_model\":\""<<fixed_model
     <<"\",\"Q\":"<<total_q<<",\"overflow\":"<<total_overflow<<",\"overflow_cells\":"<<over_nodes.size()<<",\"tsv_cells\":"<<tsv_cells.size()<<",\n";
    dump(s,"cells_by_layer",by_layer);s<<",\n";dump(s,"cells_by_fixed_share",fixed_level);s<<",\n";dump(s,"cells_by_tsv_nearby",tsv_zone);s<<",\n";
    dump(s,"cells_by_room_in_8_neighbours",neigh);s<<",\n";dump(s,"nets_by_shape",shape);s<<",\n";dump(s,"nets_by_own_pin",pin_rel);s<<",\n";
    dump(s,"nets_by_simple_room",fix_kind);s<<",\n";dump(s,"nets_cross_die",cross);s<<",\n";dump(s,"cross_die_nets_by_tsv_distance",tsv_dist_k);s<<"}\n";
    std::cout<<"PASS Q="<<total_q<<" overflow="<<total_overflow<<" cells="<<over_nodes.size()<<" tsv_cells="<<tsv_cells.size()<<std::endl;
    return 0;
 } catch(const std::exception& e) {std::cerr<<"diagnose_overflow_cells: "<<e.what()<<'\n';return 1;}
}
