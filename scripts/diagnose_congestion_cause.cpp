// Read-only breakdown of where a Guide's congestion comes from: which layers and
// cells, how full they already are before routing, whether the wires there are
// long runs or layer-change stacks, whether the stacks start at the net's own
// pin, and how far a stack would have to move to find room. Never writes a
// Guide and never linked into the router.
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
#include <queue>
#include <set>
#include <unordered_map>
using namespace eda;
namespace fs=std::filesystem;
namespace {
struct LayerRow {
    size_t cells{},full_before_routing{},with_fixed{},over_cells{},over_full{};
    uint64_t fixed{},guide{},overflow{};double q{},q_full{},capacity_sum{};
};
struct KindRow {uint64_t units{};double q{};};
struct Chain {
    std::string net;size_t pins{};int col{},row{},lo{},hi{};
    uint64_t units{};double q{};bool pin_in_chain{},pin_in_column{};int move_radius{-1},pin_layer{0};
    bool pin_cell_full{};std::string pin_exit{"no_pin_at_stack"};
};
int die_low(int layer){return layer<=10?1:11;}
int die_high(int layer){return layer<=10?10:20;}
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
    std::array<LayerRow,21> layers{};std::map<int,KindRow> overflow_sizes;std::map<std::pair<int,int>,double> windows;
    std::vector<double> cell_q(graph.size());double total_q=0;uint64_t total_overflow=0;
    std::ofstream cells(out/"cells.csv");cells.exceptions(std::ios::badbit|std::ios::failbit);
    cells<<"layer,col,row,capacity,fixed,free_before_routing,guide,overflow,Q,obs_here,pin_here\n"<<std::setprecision(17);
    for(size_t n=0;n<graph.size();++n) {
        const int node=int(n),l=graph.layer(node);
        const double cap=graph.official_capacity(node);const uint32_t fixed=graph.official_obstacle_demand(node);
        const auto c=official_resource_cell_contribution(graph,node,demand[n]);
        auto& row=layers[l];++row.cells;row.fixed+=fixed;row.guide+=demand[n];row.capacity_sum+=cap;
        const bool full=cap>0&&double(fixed)>=cap;
        if(fixed) ++row.with_fixed;
        if(full) ++row.full_before_routing;
        if(!c.overflow) continue;
        const double q=double(c.squared);cell_q[n]=q;total_q+=q;total_overflow+=c.overflow;
        ++row.over_cells;row.overflow+=c.overflow;row.q+=q;if(full){++row.over_full;row.q_full+=q;}
        auto& size=overflow_sizes[int(c.overflow)];++size.units;size.q+=q;
        windows[{graph.col(node)/32,graph.row(node)/32}]+=q;
        cells<<l<<','<<graph.col(node)<<','<<graph.row(node)<<','<<cap<<','<<fixed<<','<<cap-double(fixed)<<','<<demand[n]<<','<<c.overflow<<','<<q
             <<','<<bool(graph.tsv_obstacle_layers(graph.col(node),graph.row(node),TSV_OBS)>>l&1u)<<','<<bool(graph.tsv_obstacle_layers(graph.col(node),graph.row(node),TSV_MACRO_PIN|TSV_IO_PIN)>>l&1u)<<'\n';
    }
    const auto has_room=[&](int node){return node>=0&&graph.official_capacity(node)>0&&double(graph.official_obstacle_demand(node))+demand[node]<graph.official_capacity(node);};
    const auto is_full=[&](int node){return node>=0&&double(graph.official_obstacle_demand(node))>=graph.official_capacity(node);};
    const auto sideways=[&](int c,int r,int layer){
        const bool horizontal=lef.get_layer_direction(get_layer_name(layer))==Direction::HORIZONTAL;
        return std::array<int,2>{graph.node(c+(horizontal?1:0),r+(horizontal?0:1),layer),graph.node(c-(horizontal?1:0),r-(horizontal?0:1),layer)};
    };
    // How the net could leave its pin without climbing straight up the same column.
    const auto pin_exit=[&](int c,int r,int p,const std::set<int>& own){
        for(int next:sideways(c,r,p)) if(own.count(next)||has_room(next)) return "sideways_on_pin_layer";
        for(int y:{p-1,p+1}) {
            if(y<die_low(p)||y>die_high(p)) continue;
            const int via=graph.node(c,r,y);
            if(!own.count(via)&&!has_room(via)) continue;
            for(int next:sideways(c,r,y)) if(has_room(next)) return "one_layer_change_then_sideways";
        }
        return "has_to_cross_a_full_cell";
    };
    std::map<std::string,KindRow> kinds,exits;std::map<int,KindRow> pin_layers;std::map<std::pair<int,int>,KindRow> spans;std::vector<Chain> chains;
    for(size_t i=0;i<def.nets().size();++i) {
        bool touches=false;for(auto [node,amount]:net_demand[i]){(void)amount;if(cell_q[node]>0){touches=true;break;}}
        if(!touches) continue;
        const auto& net=def.nets()[i];std::unordered_map<int,bool> covered;
        for(const auto& s:guides.at(net.name)) {
            const int cb=std::max(0,int(std::upper_bound(grid.x_lines.begin(),grid.x_lines.end(),s.x1)-grid.x_lines.begin())-1);
            const int ce=std::min(grid.cols,int(std::lower_bound(grid.x_lines.begin(),grid.x_lines.end(),s.x2)-grid.x_lines.begin()));
            const int rb=std::max(0,int(std::upper_bound(grid.y_lines.begin(),grid.y_lines.end(),s.y1)-grid.y_lines.begin())-1);
            const int re=std::min(grid.rows,int(std::lower_bound(grid.y_lines.begin(),grid.y_lines.end(),s.y2)-grid.y_lines.begin()));
            const bool longer=(ce-cb)*(re-rb)>1;
            for(int r=rb;r<re;++r) for(int c=cb;c<ce;++c) {auto& flag=covered[graph.node(c,r,get_layer_index(s.layer))];flag=flag||longer;}
        }
        const auto pin_list=graph.own_pin_nodes(net);std::set<int> pins(pin_list.begin(),pin_list.end());
        std::map<std::tuple<int,int,int,int>,Chain> own;
        for(auto [node,amount]:net_demand[i]) {
            if(cell_q[node]<=0) continue;
            const int l=graph.layer(node),c=graph.col(node),r=graph.row(node);
            const double share=cell_q[node]*amount/std::max(demand[node],1);
            const bool planar=covered.count(node)&&covered.at(node);
            const auto cov=[&](int layer){return layer>=die_low(l)&&layer<=die_high(l)&&covered.count(graph.node(c,r,layer));};
            const bool up=cov(l+1),down=cov(l-1);
            const char* kind=planar?(up||down?"long_run_with_layer_change":"long_run_passing_through"):(up&&down?"stack_passing_through":"stack_end");
            auto& k=kinds[kind];k.units+=amount;k.q+=share;
            if(planar) continue;
            int lo=l,hi=l;
            while(cov(lo-1)&&!covered.at(graph.node(c,r,lo-1))) --lo;
            while(cov(hi+1)&&!covered.at(graph.node(c,r,hi+1))) ++hi;
            auto& chain=own[{c,r,lo,hi}];chain.net=net.name;chain.pins=net.pins.size();chain.col=c;chain.row=r;chain.lo=lo;chain.hi=hi;
            chain.units+=amount;chain.q+=share;
            auto& span=spans[{lo,hi}];span.units+=amount;span.q+=share;
            for(int y=die_low(l);y<=die_high(l);++y) if(pins.count(graph.node(c,r,y))) {chain.pin_in_column=true;if(y>=lo-1&&y<=hi+1&&!chain.pin_in_chain){chain.pin_in_chain=true;chain.pin_layer=y;chain.pin_cell_full=is_full(graph.node(c,r,y));chain.pin_exit=pin_exit(c,r,y,pins);}}
        }
        for(auto& [key,chain]:own) {
            (void)key;
            for(int radius=1;radius<=16&&chain.move_radius<0;++radius)
                for(int dr=-radius;dr<=radius&&chain.move_radius<0;++dr) for(int dc=-radius;dc<=radius;++dc) {
                    if(std::max(std::abs(dc),std::abs(dr))!=radius) continue;
                    if(chain.col+dc<0||chain.col+dc>=grid.cols||chain.row+dr<0||chain.row+dr>=grid.rows) continue;
                    bool room=true;
                    for(int y=chain.lo;y<=chain.hi&&room;++y) room=has_room(graph.node(chain.col+dc,chain.row+dr,y));
                    if(room){chain.move_radius=radius;break;}
                }
            chains.push_back(chain);
        }
    }
    // Pin escape estimate: one by one, every pin confined to one cell walks to the
    // nearest cell with room; each step pays its exact added Q on top of fixed
    // occupancy and earlier escapes. Ignores all other routing, so it is only an
    // estimate of how much pin access alone forces, not a proof of a lower limit.
    std::vector<int> escape(graph.size());double escape_q=0;size_t escaped=0,walked=0,unreached=0;
    for(const auto& net:def.nets()) {
        const auto pin_list=graph.own_pin_nodes(net);std::set<int> own(pin_list.begin(),pin_list.end());
        if(own.size()<2) continue;
        std::set<std::pair<int,int>> pin_columns;
        for(int pin:own) pin_columns.emplace(graph.col(pin),graph.row(pin));
        if(pin_columns.size()<2) continue;
        for(int start:own) {
            using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
            std::unordered_map<int,double> best;std::unordered_map<int,int> parent;open.push({0,start});best[start]=0;
            int target=-1;size_t expansions=0;
            while(!open.empty()&&expansions<2000) {
                auto [cost,node]=open.top();open.pop();if(cost>best[node]) continue;++expansions;
                if(!own.count(node)) {
                    const auto next=official_resource_cell_contribution(graph,node,escape[node]+1);
                    if(!next.overflow&&!next.zero_capacity){target=node;break;}
                }
                for(int next:graph.neighbors(node)) {
                    if(next<0||die_low(graph.layer(next))!=die_low(graph.layer(start))) continue;
                    double step=1e-7;
                    if(!own.count(next)) step+=double(official_resource_cell_contribution(graph,next,escape[next]+1).squared-official_resource_cell_contribution(graph,next,escape[next]).squared);
                    const auto it=best.find(next);
                    if(it==best.end()||cost+step<it->second){best[next]=cost+step;parent[next]=node;open.push({cost+step,next});}
                }
            }
            ++escaped;
            if(target<0){++unreached;continue;}
            if(parent[target]!=start) ++walked;
            for(int node=target;node!=start;node=parent[node]) if(!own.count(node)) ++escape[node];
        }
    }
    std::array<double,21> escape_by_layer{};
    for(size_t n=0;n<graph.size();++n) if(escape[n]) {
        const double q=double(official_resource_cell_contribution(graph,int(n),escape[n]).squared);
        escape_q+=q;escape_by_layer[graph.layer(int(n))]+=q;
    }
    std::ofstream chain_out(out/"chains.csv");chain_out.exceptions(std::ios::badbit|std::ios::failbit);
    chain_out<<"net,pins,col,row,lowest_layer,highest_layer,units_on_overflow,Q_share,pin_at_chain_end,pin_in_same_column,move_radius_with_room,pin_layer,pin_cell_full,pin_exit\n"<<std::setprecision(17);
    std::sort(chains.begin(),chains.end(),[](const Chain& a,const Chain& b){return a.q!=b.q?a.q>b.q:std::tie(a.net,a.col,a.row)<std::tie(b.net,b.col,b.row);});
    std::map<int,KindRow> radius_hist;KindRow with_pin,without_pin;
    for(const auto& c:chains) {
        chain_out<<c.net<<','<<c.pins<<','<<c.col<<','<<c.row<<','<<c.lo<<','<<c.hi<<','<<c.units<<','<<c.q<<','<<c.pin_in_chain<<','<<c.pin_in_column<<','<<c.move_radius<<','<<c.pin_layer<<','<<c.pin_cell_full<<','<<c.pin_exit<<'\n';
        auto& e=exits[c.pin_exit];++e.units;e.q+=c.q;auto& pl=pin_layers[c.pin_layer];++pl.units;pl.q+=c.q;
        auto& r=radius_hist[c.move_radius];++r.units;r.q+=c.q;
        auto& p=c.pin_in_chain?with_pin:without_pin;++p.units;p.q+=c.q;
    }
    std::vector<std::pair<double,std::pair<int,int>>> hot;for(const auto& [k,v]:windows) hot.push_back({v,k});
    std::sort(hot.rbegin(),hot.rend());double top10=0;for(size_t i=0;i<std::min<size_t>(10,hot.size());++i) top10+=hot[i].first;
    std::ofstream s(out/"summary.json");s.exceptions(std::ios::badbit|std::ios::failbit);s<<std::setprecision(10);
    s<<"{\"scope\":\"read-only congestion breakdown; offline estimate, not an official result\",\"fixed_resource_model\":\""<<fixed_model<<"\",\"Q\":"<<total_q<<",\"overflow\":"<<total_overflow<<",\"grid\":["<<grid.cols<<','<<grid.rows<<"],\n\"layers\":[";
    bool first=true;
    for(int l=1;l<=20;++l) {const auto& r=layers[l];if(!first)s<<",\n";first=false;
        s<<"{\"layer\":"<<l<<",\"cells\":"<<r.cells<<",\"mean_capacity\":"<<(r.cells?r.capacity_sum/double(r.cells):0)<<",\"cells_with_fixed\":"<<r.with_fixed<<",\"cells_full_before_routing\":"<<r.full_before_routing
         <<",\"fixed\":"<<r.fixed<<",\"guide\":"<<r.guide<<",\"overflow_cells\":"<<r.over_cells<<",\"overflow_cells_full_before_routing\":"<<r.over_full<<",\"overflow\":"<<r.overflow<<",\"Q\":"<<r.q<<",\"Q_in_cells_full_before_routing\":"<<r.q_full<<",\"pin_escape_Q\":"<<escape_by_layer[l]<<"}";}
    s<<"],\n\"overflow_size\":{";first=true;for(const auto& [k,v]:overflow_sizes){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"cells\":"<<v.units<<",\"Q\":"<<v.q<<'}';}
    s<<"},\n\"wire_kind\":{";first=true;for(const auto& [k,v]:kinds){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"units\":"<<v.units<<",\"Q_share\":"<<v.q<<'}';}
    s<<"},\n\"stacks\":{\"count\":"<<chains.size()<<",\"touching_own_pin\":{\"count\":"<<with_pin.units<<",\"Q_share\":"<<with_pin.q<<"},\"not_touching_pin\":{\"count\":"<<without_pin.units<<",\"Q_share\":"<<without_pin.q<<"}},\n\"stack_move_radius\":{";
    first=true;for(const auto& [k,v]:radius_hist){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"stacks\":"<<v.units<<",\"Q_share\":"<<v.q<<'}';}
    std::vector<std::pair<double,std::pair<int,int>>> span_rank;for(const auto& [k,v]:spans) span_rank.push_back({v.q,k});std::sort(span_rank.rbegin(),span_rank.rend());
    s<<"},\n\"stack_pin_exit\":{";first=true;for(const auto& [k,v]:exits){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"stacks\":"<<v.units<<",\"Q_share\":"<<v.q<<'}';}
    s<<"},\n\"stack_pin_layer\":{";first=true;for(const auto& [k,v]:pin_layers){if(!first)s<<',';first=false;s<<'"'<<k<<"\":{\"stacks\":"<<v.units<<",\"Q_share\":"<<v.q<<'}';}
    s<<"},\n\"top_stack_spans\":[";for(size_t i=0;i<std::min<size_t>(20,span_rank.size());++i){if(i)s<<',';const auto k=span_rank[i].second;s<<"{\"low\":"<<k.first<<",\"high\":"<<k.second<<",\"units\":"<<spans[k].units<<",\"Q_share\":"<<spans[k].q<<'}';}
    s<<"],\n\"windows32\":{\"count\":"<<hot.size()<<",\"top10_Q\":"<<top10<<"},\n\"pin_escape_estimate\":{\"Q\":"<<escape_q<<",\"pins\":"<<escaped<<",\"needed_more_than_one_step\":"<<walked<<",\"no_room_within_search_limit\":"<<unreached<<",\"note\":\"ignores all other routing; not a proven lower limit\"}}\n";
    std::cout<<"PASS Q="<<total_q<<" overflow="<<total_overflow<<" stacks="<<chains.size()<<" pin_escape_Q="<<escape_q<<std::endl;
    return 0;
 } catch(const std::exception& e) {std::cerr<<"diagnose_congestion_cause: "<<e.what()<<'\n';return 1;}
}
