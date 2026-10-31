#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool v,const char* why){++checks;if(!v)throw std::runtime_error(why);}
std::string write(const fs::path& p,const std::string& s){std::ofstream f(p);f<<s;f.close();require(bool(f),"write fixture");return p.string();}
}
namespace eda {
struct ViaStackTestAccess {
    static void exact(NegotiatedRouter& r) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<int> demand(r.graph_.size());GuideMap guides;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto fresh=measure_official_net_guides(r.def_.nets()[i],r.routes_[i].guides,r.graph_,r.lef_,r.options_.official_metrics,r.reference_lengths_[i]);
            require(fresh.demand==r.routes_[i].measured.demand,"fresh demand matches cache");
            require(fresh.wirelength==r.routes_[i].measured.wirelength,"all overlapping ordinary vias are charged");
            for(auto [n,d]:fresh.demand)demand[n]+=d;
            guides[r.def_.nets()[i].name]=r.routes_[i].guides;
        }
        require(demand==r.demand_,"full demand equals live ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"serialized pins and connectivity valid");
        const auto full=r.measure();r.verify_official_ledger(full);
        require(std::abs(full.guide.quality-r.best_metrics_.guide.quality)<1e-10,"incremental quality equals full score");
    }
    static void run(NegotiatedRouter& r,bool checkpoint) {
        require(r.route_all(),"initial routing succeeds");
        for(size_t i=0;i<r.routes_.size();++i) {
            if(r.def_.nets()[i].is_cross_die) continue;
            r.account(i,r.routes_[i],-1);NetRoute route;route.net_index=i;
            const auto node=[&](int c,int l){return r.graph_.node(c,2,l);};
            const auto add=[&](int a,int b){route.nodes.insert(a);route.nodes.insert(b);route.edges.insert(route_edge(a,b));};
            for(int c=0;c<3;++c)add(node(c,3),node(c+1,3));
            for(int l=3;l<7;++l)add(node(3,l),node(3,l+1));
            for(int c=3;c<6;++c)add(node(c,7),node(c+1,7));
            for(int n:{node(0,3),node(6,7)}){PinAccess a;a.node=n;route.accesses.push_back(a);}
            r.encode(i,route);r.routes_[i]=std::move(route);r.account(i,r.routes_[i],1);
        }
        r.best_metrics_=r.measure();exact(r);
        const auto original=r.routes_;const auto demand=r.demand_;
        const auto owners=r.cell_nets_;const auto edges=r.edge_nets_;
        const auto before=r.best_metrics_;
        const auto choices=r.via_stack_candidates(0,r.routes_[0]);
        require(!choices.empty(),"a congested ordinary via ladder has local alternatives");
        require(std::any_of(choices.begin(),choices.end(),[&](const auto& trial) {
            return trial.nodes.count(r.graph_.node(3,2,4))&&trial.nodes.count(r.graph_.node(3,2,6));
        }),"compatible subladders preserve their external via endpoints");
        const int hotspot=r.graph_.node(3,2,5);
        for(const auto& trial:choices) {
            require(!trial.nodes.count(hotspot),"interior saturated cell is removed");
            require(trial.nodes.count(r.graph_.node(3,2,3))&&trial.nodes.count(r.graph_.node(3,2,7)),"both external endpoints retained");
            require(trial.measured.ordinary_connections==original[0].measured.ordinary_connections,"relocation retains all four real via charges");
            require(trial.measured.tsv_cells==original[0].measured.tsv_cells,"TSV distribution unchanged");
            for(auto edge:trial.edges){const auto [a,b]=edge_nodes(edge);require(r.graph_.legal_edge(a,b),"every new preferred-direction edge is legal");}
        }
        require(r.demand_==demand&&r.cell_nets_==owners&&r.edge_nets_==edges,"candidate enumeration is read only");
        auto branched=original[0];const int neighbor=r.graph_.node(4,2,5);
        branched.nodes.insert(neighbor);branched.edges.insert(route_edge(hotspot,neighbor));
        require(r.via_stack_candidates(0,branched).empty(),"a branch at the congested interior cannot be cut");
        auto pinned=original[0];PinAccess access;access.node=hotspot;pinned.accesses.push_back(access);
        require(r.via_stack_candidates(0,pinned).empty(),"an interior terminal access cannot move");
        auto explicit_route=original[0];explicit_route.explicit_geometry=true;
        require(r.via_stack_candidates(0,explicit_route).empty(),"explicit geometry is preserved");
        r.options_.via_stack_radius=0;
        require(r.via_stack_candidates(0,original[0]).empty(),"default-off starts no work");r.options_.via_stack_radius=3;
        const auto deadline=r.deadline_;r.deadline_=std::chrono::steady_clock::now();
        require(r.via_stack_candidates(0,original[0]).empty(),"shared deadline is respected");r.deadline_=deadline;
        // Force the local proposal path to succeed without relying on a maze.
        r.options_.search_limit=0;r.options_.v7_rrr_astar_net_limit=1;
        const auto maze_before=r.counters_.v7_astar_nets;
        if(checkpoint)r.begin_rrr_checkpoint();
        require(r.rrr_round({0}),"complete transaction accepts a beneficial local shift");
        require(r.counters_.via_stack_selected>0,"local candidate bypasses whole-net reconstruction");
        require(r.counters_.v7_astar_nets==maze_before,"local-only move does not consume the maze quota");
        for(size_t i=1;i<r.routes_.size();++i)require(r.routes_[i].edges==original[i].edges,"unrelated nets and the cross-die route stay unchanged");
        require(r.best_metrics_.official.squared_normalized_overflow<before.official.squared_normalized_overflow,"complete Q falls");
        require(r.best_metrics_.guide.quality>before.guide.quality,"complete W/C/T improves");
        require(r.best_metrics_.official.thermal_score==before.official.thermal_score,"fixed TSV thermal term remains exact");
        require(r.deadline_==deadline,"local proposals do not extend the budget");exact(r);
        if(checkpoint) {
            r.restore_rrr_checkpoint("injected via-stack rollback");
            require(r.routes_[0].edges==original[0].edges&&r.routes_[1].edges==original[1].edges,"rollback restores complete geometry");
            require(r.demand_==demand&&r.cell_nets_==owners&&r.edge_nets_==edges,"rollback restores demand and both ownership maps");exact(r);
        } else {
            const auto stable=r.routes_[0].edges;const auto stable_demand=r.demand_;
            require(!r.rrr_round({0}),"no-relief failed rebuild is rejected");
            require(r.routes_[0].edges==stable&&r.demand_==stable_demand,"rejection preserves the improved route");exact(r);
        }
    }
};
}
int main(int argc,char** argv) {
 try {
    if(argc!=3)throw std::runtime_error("via_stack_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
    fs::path folder=argv[1];fs::create_directories(folder);
    std::string tech,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 700 500 ) ; GCELLGRID X 0 DO 8 STEP 100 ; GCELLGRID Y 0 DO 6 STEP 100 ; ";
    for(int l=1;l<=20;++l) {
        const auto name=get_layer_name(l);
        tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(l%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
        const std::string tracks=l==5?"5 DO 7 STEP 100":"5 DO 70 STEP 10";
        design+="TRACKS X "+tracks+" LAYER "+name+" ; TRACKS Y "+tracks+" LAYER "+name+" ; ";
    }
    design+="COMPONENTS 0 ; END COMPONENTS PINS 6 ; ";
    for(int i=0;i<2;++i)for(int j=0;j<2;++j)
        design+="- p"+std::to_string(i)+std::to_string(j)+" + LAYER metal"+(j?"7":"3")+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+(j?"650":"50")+" 250 ) N ; ";
    design+="- lo + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 50 50 ) N ; - hi + LAYER metal13 ( -1 -1 ) ( 1 1 ) + FIXED ( 650 450 ) N ; ";
    design+="END PINS NETS 3 ; - n0 ( PIN p00 ) ( PIN p01 ) ; - n1 ( PIN p10 ) ( PIN p11 ) ; - zz_cross ( PIN lo ) ( PIN hi ) ; END NETS END DESIGN";
    LefParser lef;DefParser def;
    require(lef.parse(write(folder/"fixture.lef",tech))&&def.parse(write(folder/"fixture.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parses");
    std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
    for(int y=0;y<5;++y)for(int x=0;x<7;++x){if(x||y)heat<<',';heat<<'['<<5+10*x<<','<<5+10*y<<",0,"<<26+x+y<<']';}heat<<"]}";
    ThermalParser thermal;require(thermal.parse_json(write(folder/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"heat parses");
    RouterOptions o;o.official_model=true;o.official_runtime=argv[2];o.time_limit=120;
    o.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};o.graph_options.congestion_overflow_scale=1;
    o.official_metrics.score_20261020=true;o.official_metrics.all_adjacent_overlaps=true;o.official_metrics.snap_reference_coordinates=true;
    o.official_metrics.tsv_track_demand=true;o.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
    o.official_metrics.gcell_width_dbu=def.gcell_pitch_x();o.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
    o.rrr_rounds=o.tsv_rounds=o.wirelength_rounds=0;o.tsv_local=false;o.v7_rrr=true;o.v8_rrr=true;o.via_stack_radius=3;
    for(bool checkpoint:{false,true}){o.v9_incremental_rrr=checkpoint;NegotiatedRouter r(def,lef,thermal,o);ViaStackTestAccess::run(r,checkpoint);}
    std::cout<<"via_stack_tests: "<<checks<<" checks PASS\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
