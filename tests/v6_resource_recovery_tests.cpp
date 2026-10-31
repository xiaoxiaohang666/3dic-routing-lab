#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool value,const std::string& reason) {++checks;if(!value) throw std::runtime_error(reason);}
void near(double a,double b,const std::string& reason) {
    require(std::abs(a-b)<=1e-10*std::max({1.0,std::abs(a),std::abs(b)}),reason);
}
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
}
namespace eda {
struct V6ResourceRecoveryTestAccess {
    static void exact(NegotiatedRouter& r) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<int> demand(r.graph_.size());GuideMap guides;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto fresh=measure_official_net_guides(r.def_.nets()[i],r.routes_[i].guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]);
            require(fresh.demand==r.routes_[i].measured.demand,"serialized net demand equals route cache");
            for(auto [node,count]:fresh.demand) demand[size_t(node)]+=count;
            guides[r.def_.nets()[i].name]=r.routes_[i].guides;
        }
        require(demand==r.demand_,"full serialized demand equals transactional ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"encoded net connectivity is valid");
        const auto actual=r.measure();r.verify_official_ledger(actual);
        near(actual.guide.quality,r.best_metrics_.guide.quality,"incumbent and full score agree");
    }
    static void expensive_tree(NegotiatedRouter& r,size_t index) {
        r.account(index,r.routes_[index],-1);
        NetRoute route;route.net_index=index;
        const auto node=[&](int c,int row,int layer){return r.graph_.node(c,row,layer);};
        auto edge=[&](int a,int b){require(r.graph_.legal_edge(a,b),"fixture edge is legal");
            route.nodes.insert(a);route.nodes.insert(b);route.edges.insert(route_edge(a,b));};
        for(int c=0;c<4;++c) edge(node(c,2,1),node(c+1,2,1));
        for(int row=0;row<2;++row) edge(node(2,row,4),node(2,row+1,4));
        for(int row=2;row<4;++row) edge(node(2,row,8),node(2,row+1,8));
        for(auto [row,top]:std::vector<std::pair<int,int>>{{0,4},{2,8},{4,8}})
            for(int layer=1;layer<top;++layer) edge(node(2,row,layer),node(2,row,layer+1));
        for(auto [c,row]:std::vector<std::pair<int,int>>{{0,2},{4,2},{2,0},{2,4}}) {
            PinAccess access;access.node=node(c,row,1);route.accesses.push_back(access);
        }
        r.encode(index,route);r.routes_[index]=std::move(route);r.account(index,r.routes_[index],1);
        r.best_metrics_=r.measure();exact(r);
    }
    static void layers(NegotiatedRouter& r) {
        require(r.route_all(),"layer fixture initially routes");expensive_tree(r,0);
        const auto before=r.routes_[0];const auto demand=r.demand_;
        const auto cells=r.cell_nets_;const auto edges=r.edge_nets_;const auto score=r.best_metrics_;
        r.options_.joint_layer_recovery=true;
        auto candidates=r.recovery_layer_candidates(0,before);
        require(!candidates.empty() && candidates.size()<=12,"joint layer assignment emits a bounded shortlist");
        int best_vias=before.vias;size_t best_center=99;
        for(const auto& candidate:candidates) {
            best_vias=std::min(best_vias,candidate.vias);
            require(candidate.recovery_layer && candidate.slot==before.slot,"candidate marks layer operation and keeps TSV mapping");
            const auto fresh=measure_official_net_guides(r.def_.nets()[0],candidate.guides,r.graph_,r.lef_,r.options_.official_metrics,r.reference_lengths_[0]);
            require(fresh.demand==candidate.measured.demand && fresh.wirelength==candidate.measured.wirelength,
                "all via and shared cell charges come from encoded geometry");
            // One center ladder joins both vertical branches to the shared
            // horizontal trunk; independent branch ladders would count it twice.
            size_t center_vias=0;
            for(auto e:candidate.edges) {const auto [a,b]=edge_nodes(e);
                if(r.graph_.col(a)==2 && r.graph_.row(a)==2 && r.graph_.layer(a)!=r.graph_.layer(b)) ++center_vias;
            }
            if(candidate.vias==best_vias) best_center=std::min(best_center,center_vias);
        }
        require(best_vias==3 && best_center==1,"both vertical branches share one actual center via and need only three total vias");
        require(r.demand_==demand && r.cell_nets_==cells && r.edge_nets_==edges,
            "candidate enumeration does not mutate resources or owners");
        near(r.best_metrics_.guide.quality,score.guide.quality,"candidate enumeration preserves incumbent");
        require(!r.recovery_transaction(0,[&](const auto& old,const auto&)->std::optional<NetRoute>{
            auto invalid=old;invalid.guides.clear();return invalid;
        }),"invalid joint transaction is rejected");
        require(r.demand_==demand && r.cell_nets_==cells && r.edge_nets_==edges,
            "failed transaction restores all owners and demand");exact(r);
        require(r.recovery_transaction(0,[&](const auto& old,const auto& base)->std::optional<NetRoute>{
            auto choices=r.recovery_layer_candidates(0,old);if(choices.empty()) return std::nullopt;
            auto best=choices.begin();for(auto it=choices.begin();it!=choices.end();++it)
                if(r.official_candidate_key(*it,base)<r.official_candidate_key(*best,base)) best=it;
            return *best;
        }),"complete score accepts real joint layer improvement");
        require(r.routes_[0].measured.wirelength<before.measured.wirelength && r.routes_[0].vias<before.vias,
            "accepted joint tree has lower charged length and fewer ordinary via edges");
        require(r.counters_.v6_layer_accepted>0,"accepted layer transaction is counted");exact(r);
        const auto deadline=r.deadline_;r.deadline_=std::chrono::steady_clock::now();
        const auto attempts=r.counters_.v6_layer_candidates;
        require(r.recovery_layer_candidates(0,r.routes_[0]).empty() && attempts==r.counters_.v6_layer_candidates,
            "expired phase launches no joint layer work");r.deadline_=deadline;exact(r);
    }
    static void global_coverage(NegotiatedRouter& r) {
        require(r.route_all(),"global coverage fixture initially routes");
        for(size_t i=0;i<2;++i) {
            r.account(i,r.routes_[i],-1);auto trial=r.routes_[i];
            int previous=r.graph_.node(0,2,1);trial.nodes.insert(previous);
            for(int layer=2;layer<=9;++layer) {
                const int next=r.graph_.node(0,2,layer);trial.nodes.insert(next);trial.edges.insert(route_edge(previous,next));previous=next;
            }
            for(int c=1;c<5;++c) {
                const int next=r.graph_.node(c,2,9);trial.nodes.insert(next);trial.edges.insert(route_edge(previous,next));previous=next;
            }
            r.encode(i,trial);r.routes_[i]=std::move(trial);r.account(i,r.routes_[i],1);
        }
        r.best_metrics_=r.measure();exact(r);
        const auto before=r.best_metrics_;const auto deadline=r.deadline_;
        r.options_.resource_local_rrr=true;r.options_.tsv_landing_rrr=true;
        r.options_.local_repair=false;r.options_.pin_local=false;r.options_.fast_initial=false;
        r.options_.v7_rrr_astar_net_limit=0;r.options_.v8_rrr_group_size=8;
        r.options_.rrr_net_limit=16;r.options_.v7_rrr_net_limit=16;
        const auto groups=r.counters_.v8_rrr_groups,local=r.counters_.v6_local_rrr_groups;
        r.run_hotspot_rrr(2);
        require(r.counters_.v8_rrr_groups>groups,"combined mode retains all-layer work outside clean TSV landings");
        require(r.counters_.v6_local_rrr_groups==local,"clean landing queue adds no unrelated work");
        require(r.best_metrics_.guide.quality>before.guide.quality &&
            r.best_metrics_.official.squared_normalized_overflow<before.official.squared_normalized_overflow,
            "all-layer group improves real congestion outside TSV landing layers");
        require(r.deadline_==deadline && r.options_.resource_local_rrr && r.options_.tsv_landing_rrr && r.options_.v8_rrr_group_size==8,
            "combined global/local pass restores enclosing deadline and options");exact(r);
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("v6_resource_recovery_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path folder=argv[1];fs::create_directories(folder);
        std::string tech,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 180 180 ) ; GCELLGRID X 0 DO 7 STEP 30 ; GCELLGRID Y 0 DO 7 STEP 30 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            const auto tracks=layer==9?"15 DO 6 STEP 30":"0.5 DO 180 STEP 1";
            design+="TRACKS X "+std::string(tracks)+" LAYER "+name+" ; TRACKS Y "+tracks+" LAYER "+name+" ; ";
        }
        design+="COMPONENTS 0 ; END COMPONENTS PINS 10 ; ";
        const std::vector<std::pair<int,int>> points{{15,75},{135,75},{75,15},{75,135}};
        for(int net=0;net<2;++net) for(size_t pin=0;pin<points.size();++pin)
            design+="- p"+std::to_string(net)+std::to_string(pin)+" + LAYER metal1 ( -1 -1 ) ( 1 1 ) + FIXED ( "+
                std::to_string(points[pin].first)+" "+std::to_string(points[pin].second)+" ) N ; ";
        design+="- lo + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 15 15 ) N ; ";
        design+="- hi + LAYER metal13 ( -1 -1 ) ( 1 1 ) + FIXED ( 165 165 ) N ; ";
        design+="END PINS NETS 3 ; ";
        for(int net=0;net<2;++net) {design+="- tree"+std::to_string(net)+" ";
            for(int pin=0;pin<4;++pin) design+="( PIN p"+std::to_string(net)+std::to_string(pin)+" ) ";
            design+="; ";
        }
        design+="- zz_cross ( PIN lo ) ( PIN hi ) ; END NETS END DESIGN";
        LefParser lef;DefParser def;
        require(lef.parse(write(folder/"fixture.lef",tech))&&def.parse(write(folder/"fixture.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parses");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<6;++y) for(int x=0;x<6;++x) {if(x||y) heat<<',';heat<<'['<<1.5+3*x<<','<<1.5+3*y<<",0,"<<26+x+6*y<<']';}
        heat<<"]}";ThermalParser thermal;
        require(thermal.parse_json(write(folder/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"thermal parses");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];options.time_limit=120;
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        options.official_metrics.score_20261020=true;options.official_metrics.tsv_track_demand=true;
        options.official_metrics.gcell_width_dbu=def.gcell_pitch_x();options.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
        options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;
        options.v7_cell_cost=true;options.v8_rrr=true;
        NegotiatedRouter layer(def,lef,thermal,options);V6ResourceRecoveryTestAccess::layers(layer);
        options.v9_incremental_rrr=true;NegotiatedRouter global(def,lef,thermal,options);V6ResourceRecoveryTestAccess::global_coverage(global);
        std::cout<<"v6_resource_recovery_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}