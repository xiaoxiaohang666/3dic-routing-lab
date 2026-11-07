// Read-only estimate: how low Q could go if via stacks crossing overflowing
// cells moved sideways together, competing for the same free tracks.
// Optimistic on purpose: only the cells of each move are charged, and whether
// the endpoints can really reach the new column is ignored. It is neither a
// proven lower bound nor an official result. It never writes a Guide.
#include "objectives/net_cell_contribution.hpp"
#include "objectives/official_metrics.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
struct Stack {size_t net{};int col{},row{},low{},high{};};
struct Move {double gain{};size_t stack{};int col{},row{};};
long double cell_q(const RoutingGraph& graph,int node,int demand) {
    return official_resource_cell_contribution(graph,node,demand).squared;
}
std::vector<int> split_ints(const std::string& text) {
    std::vector<int> values;std::stringstream in(text);std::string item;
    while(std::getline(in,item,',')) values.push_back(std::stoi(item));
    if(values.empty()) throw std::runtime_error("empty radius list");
    return values;
}
}
int main(int argc,char** argv) {
    try {
        std::string def_file,lef_file,guide_file,output,fixed_model="production",radii_text="1,2,4,8";
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if(i+1>=argc) throw std::runtime_error("missing value for "+key);
            const std::string value=argv[++i];
            if(key=="--def") def_file=value;else if(key=="--lef") lef_file=value;else if(key=="--guide") guide_file=value;
            else if(key=="--output") output=value;else if(key=="--fixed-resource-model") fixed_model=value;
            else if(key=="--radii") radii_text=value;else throw std::runtime_error("unknown option "+key);
        }
        if(def_file.empty()||lef_file.empty()||guide_file.empty()||output.empty())
            throw std::runtime_error("required: --def --lef --guide --output NEW");
        if(fixed_model!="production"&&fixed_model!="observed-20261104") throw std::runtime_error("invalid fixed resource model");
        if(fs::exists(output)) throw std::runtime_error("output must be new");
        LefParser lef;if(!lef.parse(lef_file)) throw std::runtime_error("LEF");
        DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF");
        RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,fixed_model=="observed-20261104"});
        const auto guides=compat::read_guides(guide_file);
        OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;
        mo.snap_reference_coordinates=true;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
        std::vector<int> base(graph.size());std::vector<Stack> stacks;
        const auto horizontal=[&](int layer){return lef.get_layer_direction(get_layer_name(layer))==Direction::HORIZONTAL;};
        for(size_t i=0;i<def.nets().size();++i) {
            const auto& net=def.nets()[i];const auto found=guides.find(net.name);
            if(found==guides.end()) throw std::runtime_error("net missing from Guide: "+net.name);
            const auto measured=measure_official_net_guides(net,found->second,graph,lef,mo,0);
            std::set<int> covered;
            for(auto [n,d]:measured.demand) {base[n]+=d;covered.insert(n);}
            const auto pins=graph.own_pin_nodes(net);
            const int plane=graph.grid().rows*graph.grid().cols;
            const auto locked=[&](int n) {
                if(std::binary_search(pins.begin(),pins.end(),n)) return true;
                return (graph.layer(n)==10||graph.layer(n)==11) &&
                    std::find(measured.tsv_cells.begin(),measured.tsv_cells.end(),n%plane)!=measured.tsv_cells.end();
            };
            // A cell carrying wire along its layer anchors the stack; only cells
            // that do nothing but pass the via through are free to move.
            const auto through=[&](int n) {
                if(locked(n)) return false;
                const int c=graph.col(n),r=graph.row(n),l=graph.layer(n);const bool h=horizontal(l);
                return !covered.count(graph.node(c+(h?1:0),r+(h?0:1),l)) && !covered.count(graph.node(c-(h?1:0),r-(h?0:1),l));
            };
            for(int n:covered) {
                const int c=graph.col(n),r=graph.row(n),l=graph.layer(n);
                if(through(n)) continue;
                int top=l;
                while(top<20 && (top-1)/10==(l-1)/10 && covered.count(graph.node(c,r,top+1)) && through(graph.node(c,r,top+1))) ++top;
                const int cap=top+1;
                if(top>l && (cap-1)/10==(l-1)/10 && covered.count(graph.node(c,r,cap))) stacks.push_back({i,c,r,l,cap});
            }
        }
        long double base_q=0;size_t overflow_cells=0;
        for(size_t n=0;n<graph.size();++n) {
            const long double q=cell_q(graph,int(n),base[n]);
            base_q+=q;overflow_cells+=q>0;
        }
        std::ofstream out(output);out<<std::setprecision(12)
            <<"{\"scope\":\"optimistic estimate of Q if via stacks moved sideways together; endpoint reachability ignored; not an official result\""
            <<",\"fixed_resource_model\":\""<<fixed_model<<"\",\"Q\":"<<double(base_q)<<",\"overflow_cells\":"<<overflow_cells
            <<",\"stacks\":"<<stacks.size()<<",\"radii\":[";
        bool first=true;
        for(int radius:split_ints(radii_text)) {
            auto demand=base;size_t moves=0,rounds=0;
            std::vector<char> done(stacks.size());
            const auto move_gain=[&](const Stack& s,int col,int row)->double {
                std::map<int,int> delta;
                for(int l=s.low+1;l<s.high;++l) {
                    delta[graph.node(s.col,s.row,l)]-=1;
                    const int target=graph.node(col,row,l);
                    if(target<0) return 0;
                    delta[target]+=1;
                }
                // Both ends must run to the new column on their own layers.
                const int steps=std::abs(col-s.col)+std::abs(row-s.row);
                for(int end:{s.low,s.high}) for(int k=1;k<=steps;++k) {
                    const int dc=col==s.col?0:(col>s.col?1:-1),dr=row==s.row?0:(row>s.row?1:-1);
                    const int n=graph.node(s.col+dc*std::min(k,std::abs(col-s.col)),s.row+dr*std::max(0,k-std::abs(col-s.col)),end);
                    if(n<0) return 0;
                    delta[n]+=1;
                }
                long double dq=0;
                for(auto [n,d]:delta) if(d) dq+=cell_q(graph,n,demand[n]+d)-cell_q(graph,n,demand[n]);
                return double(-dq);
            };
            for(;rounds<64;++rounds) {
                std::vector<Move> proposals;
                for(size_t k=0;k<stacks.size();++k) {
                    if(done[k]) continue;
                    const auto& s=stacks[k];Move best;best.stack=k;
                    for(int dr=-radius;dr<=radius;++dr) for(int dc=-radius;dc<=radius;++dc) {
                        if((dc==0&&dr==0) || std::abs(dc)+std::abs(dr)>radius) continue;
                        const double gain=move_gain(s,s.col+dc,s.row+dr);
                        if(gain>best.gain) {best.gain=gain;best.col=s.col+dc;best.row=s.row+dr;}
                    }
                    if(best.gain>1e-14) proposals.push_back(best);
                }
                if(proposals.empty()) break;
                std::sort(proposals.begin(),proposals.end(),[](const Move& a,const Move& b){return a.gain!=b.gain?a.gain>b.gain:a.stack<b.stack;});
                size_t applied=0;
                for(const auto& p:proposals) {
                    // Earlier moves this round may have taken the free tracks.
                    const auto& s=stacks[p.stack];
                    if(move_gain(s,p.col,p.row)<=1e-14) continue;
                    for(int l=s.low+1;l<s.high;++l) {--demand[graph.node(s.col,s.row,l)];++demand[graph.node(p.col,p.row,l)];}
                    const int steps=std::abs(p.col-s.col)+std::abs(p.row-s.row);
                    for(int end:{s.low,s.high}) for(int k=1;k<=steps;++k) {
                        const int dc=p.col==s.col?0:(p.col>s.col?1:-1),dr=p.row==s.row?0:(p.row>s.row?1:-1);
                        ++demand[graph.node(s.col+dc*std::min(k,std::abs(p.col-s.col)),s.row+dr*std::max(0,k-std::abs(p.col-s.col)),end)];
                    }
                    done[p.stack]=1;++applied;
                }
                moves+=applied;
                if(!applied) break;
            }
            long double q=0,in_full=0;size_t left=0;
            for(size_t n=0;n<graph.size();++n) {
                const long double value=cell_q(graph,int(n),demand[n]);
                q+=value;left+=value>0;
                if(graph.official_obstacle_demand(int(n))>=graph.official_capacity(int(n))) in_full+=value;
            }
            out<<(first?"":",")<<"{\"radius\":"<<radius<<",\"moves\":"<<moves<<",\"rounds\":"<<rounds<<",\"Q\":"<<double(q)
               <<",\"C\":"<<100/std::sqrt(1+double(q))<<",\"Q_in_cells_full_before_routing\":"<<double(in_full)<<",\"overflow_cells\":"<<left<<'}';
            first=false;
        }
        out<<"]}\n";out.close();if(!out) throw std::runtime_error("report failed");
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
