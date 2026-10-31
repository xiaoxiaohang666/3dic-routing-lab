#include "router/negotiated_router.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace eda::detail {
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch&,
    const RoutingGraph&,const TsvPolicy&,const std::vector<int>&,size_t,int,double,TsvMoveSearchOptions,bool,
    const NetGuideMetrics&,const std::vector<int>&,const OfficialMetricOptions&,bool,bool,TsvMoveSearchDiagnostics*);
}
namespace {
size_t checks=0;
void require(bool value,const std::string& message) {
    ++checks;if(!value) throw std::runtime_error(message);
}
std::string write(const fs::path& path,const std::string& contents) {
    std::ofstream out(path);out<<contents;out.close();require(bool(out),"write fixture");return path.string();
}
struct Fixture {
    LefParser lef;DefParser def;ThermalParser thermal;
    std::unique_ptr<RoutingGraph> graph;
    std::unique_ptr<TsvPolicy> policy;
    Fixture(const fs::path& dir) {
        std::string technology,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
            "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID Y 0 DO 11 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            technology+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+
                " ; WIDTH 0.2 ; END "+name+"\n";
            design+="TRACKS X 5 DO 100 STEP 10 LAYER "+name+" ; TRACKS Y 5 DO 100 STEP 10 LAYER "+name+" ; ";
        }
        design+="COMPONENTS 0 ; END COMPONENTS PINS 2 ; "
            "- bottom + LAYER metal1 ( -1 -1 ) ( 1 1 ) + FIXED ( 50 50 ) N ; "
            "- top + LAYER metal12 ( -1 -1 ) ( 1 1 ) + FIXED ( 50 50 ) N ; "
            "END PINS NETS 1 ; - cross ( PIN bottom ) ( PIN top ) ; END NETS END DESIGN";
        require(lef.parse(write(dir/"technology.lef",technology)),"parse technology");
        require(def.parse(write(dir/"design.def",design),lef,DefGridMapping::OFFICIAL_FULL),"parse design");
        RoutingGraphOptions go{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        graph=std::make_unique<RoutingGraph>(def,lef,go);
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<10;++y) for(int x=0;x<10;++x) {
            if(x||y) heat<<',';
            heat<<'['<<5+10*x<<','<<5+10*y<<",0,30]";
        }
        heat<<"]}";
        require(thermal.parse_json(write(dir/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),
            def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse uniform heat");
        TsvPolicyOptions po;po.resource_mode=TsvResourceMode::GLOBAL_GRID;
        policy=std::make_unique<TsvPolicy>(def,lef,*graph,po);
    }
};
}
namespace eda {
struct DynamicTsvTestAccess {
    static void coverage(NegotiatedRouter& router) {
        require(router.route_all(),"coverage fixture routes completely");
        const auto demand=router.demand_;
        const auto owners=router.cell_nets_;
        const auto baseline=router.best_metrics_.guide.quality;
        router.options_.v8_tsv_dynamic=true;router.options_.tsv_rounds=1;
        router.options_.v9_tsv_coverage=true;router.options_.v9_tsv_stale_sweeps=1;
        router.options_.thermal_coverage=true;
        router.options_.resource_candidate_pool=true;router.options_.tsv_landing_pricing=true;
        router.options_.v7_tsv_nets=1;router.options_.v7_tsv_candidates=2;
        router.options_.v8_tsv_probe_candidates=1;router.options_.v7_tsv_seconds=10;
        router.options_.search_limit=0;
        router.optimize_dynamic_tsv();
        require(router.counters_.thermal_query_unique==1,
            "thermal queue completes its independent query despite an earlier no-gain ordinary sweep");
        require(router.counters_.thermal_positive_unique==0,
            "uniform heat produces no false positive correlation opportunities");
        require(router.counters_.thermal_rerouted_unique==1,
            "actual thermal reroute coverage is distinct from probe and query coverage");
        require(router.counters_.v9_tsv_sweeps>=2,
            "ordinary no-gain sweep alone cannot prematurely terminate independent thermal coverage");
        require(router.best_metrics_.guide.quality+1e-12>=baseline,
            "failed or accepted thermal candidates retain the exact quality floor");
        router.complete_validation_cached_=false;router.validate_complete();
        router.verify_official_ledger(router.measure());
        // One cross-die net and uniform heat provide no shorter slot than its
        // initial median assignment, so all candidate movements must roll back.
        require(router.demand_==demand && router.cell_nets_==owners,
            "unsuccessful thermal trials restore real demand and owners exactly");
        require(router.counters_.thermal_query_seconds>=0 && router.counters_.thermal_route_seconds>=0,
            "independent thermal work has finite nonnegative accounting");
    }
};
}
int main(int argc,char** argv) {
    try {
        require(argc>=2,"v6_thermal_resource_tests OUTPUT_DIRECTORY [OFFICIAL_RUNTIME]");
        const fs::path dir=argv[1];fs::create_directories(dir);Fixture f(dir);
        const auto assignment=f.policy->assign(f.def.nets(),f.thermal);
        require(assignment.size()==1 && assignment[0]>=0,"fixture has one assigned TSV");
        TsvMoveSearch search(f.def,*f.graph,*f.policy,f.thermal,assignment);
        int target=-1;
        for(const auto& slot:f.policy->slots()) if(f.graph->col(slot.lower_node)==8 && f.graph->row(slot.lower_node)==8)
            target=slot.id;
        require(target>=0 && target!=assignment[0],"distant free target exists");
        TsvMoveSearchOptions options;options.max_candidates=1;options.length_price=.001;
        const auto base=search.candidates(0,options);
        require(base.size()==1 && base.front().slot!=target,"geometry top-k excludes the distant target");
        TsvMoveSearchDiagnostics diagnostics;
        options.landing_cost=[&](int slot) {return slot==target || slot==assignment[0]?0.0:1000000.0;};
        options.resource_node_limit=4096;options.diagnostics=&diagnostics;
        const auto resource=search.candidates(0,options);
        require(resource.size()==1 && resource.front().slot==target,
            "resource-aware retrieval rescues a target outside geometry and heat top-k");
        require(diagnostics.resource_queries>0,"resource retrieval records actual price queries");
        require(search.current_slot(0)==assignment[0],"retrieval leaves assignment unchanged");
        const double correlation=search.current_correlation();
        require(search.commit_move(0,target),"accepted occupancy transition updates the index");
        require(search.current_slot(0)==target && search.current_correlation()==correlation,
            "uniform heat stays finite under index update");
        const auto moved=search.candidates(0,options);
        for(const auto& candidate:moved) require(candidate.slot!=target,"occupied slot cannot reappear as destination");
        require(search.commit_move(0,assignment[0]),"old slot is restored in the live index");

        options.landing_cost={};options.diagnostics=nullptr;
        std::vector<int> demand(f.graph->size());
        for(const auto& slot:f.policy->slots())
            for(int node:{slot.lower_node,slot.upper_node}) demand[size_t(node)]=1000000;
        const auto& desired=f.policy->slots()[size_t(target)];
        demand[size_t(desired.lower_node)]=demand[size_t(desired.upper_node)]=0;
        OfficialMetricOptions metrics;metrics.score_20261020=true;metrics.tsv_track_demand=true;
        NetGuideMetrics own;
        own.demand=mandatory_tsv_contribution(*f.graph,f.policy->slots()[size_t(assignment[0])].point,1,true);
        const auto saved=demand;
        const auto quote_only=detail::dynamic_tsv_candidates(search,*f.graph,*f.policy,demand,0,assignment[0],0,
            options,false,own,{},metrics,true,false,nullptr);
        const auto quote_pool=detail::dynamic_tsv_candidates(search,*f.graph,*f.policy,demand,0,assignment[0],0,
            options,false,own,{},metrics,true,true,&diagnostics);
        require(!quote_only.empty() && quote_only.front().slot!=target,
            "correct quote alone keeps the original geometric retrieval breadth");
        require(quote_pool.size()==1 && quote_pool.front().slot==target,
            "independent resource-pool option exposes a physically lower-demand landing");
        require(demand==saved && search.current_slot(0)==assignment[0],
            "both ablation modes are read-only candidate transactions");
        options.landing_cost=[](int) {return -1.0;};bool rejected=false;
        try {search.candidates(0,options);} catch(const std::logic_error&) {rejected=true;}
        require(rejected,"negative resource price cannot invalidate spatial lower bounds");
        if(argc>=3) {
            RouterOptions ro;ro.official_model=true;ro.official_runtime=argv[2];
            ro.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
            ro.official_metrics.score_20261020=true;ro.official_metrics.tsv_track_demand=true;
            ro.official_metrics.thermal_gain=20;
            ro.official_metrics.gcell_width_dbu=f.def.gcell_pitch_x();
            ro.official_metrics.gcell_height_dbu=f.def.gcell_pitch_y();
            ro.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
            ro.rrr_rounds=ro.tsv_rounds=ro.wirelength_rounds=0;ro.tsv_local=false;
            ro.v7_cell_cost=true;ro.time_limit=60;
            NegotiatedRouter router(f.def,f.lef,f.thermal,ro);DynamicTsvTestAccess::coverage(router);
        }
        std::cout<<"v6_thermal_resource_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}