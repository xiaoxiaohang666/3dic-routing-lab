#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool ok,const std::string& message) {++checks;if(!ok) throw std::runtime_error(message);}
void near(double a,double b,const std::string& message) {
    require(std::abs(a-b)<=1e-10*std::max({1.0,std::abs(a),std::abs(b)}),message);
}
std::string write(const fs::path& path,const std::string& contents) {
    std::ofstream out(path);out<<contents;out.close();require(bool(out),"write fixture");return path.string();
}
std::string geometry(const std::vector<NetRoute>& routes) {
    std::ostringstream text;
    for(const auto& route:routes) {
        text<<route.slot<<';';
        for(const auto& g:route.guides) text<<g.layer<<','<<g.x1<<','<<g.y1<<','<<g.x2<<','<<g.y2<<';';
        text<<'\n';
    }
    return text.str();
}
}
namespace eda {
namespace detail {
// Exercise the production pool retrieval and live-Q filter, without exposing
// this implementation detail as part of NegotiatedRouter's public interface.
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch&,
    const RoutingGraph&,const TsvPolicy&,const std::vector<int>&,size_t,int,double,TsvMoveSearchOptions,bool);
bool tsv_region_accessible(const RoutingGraph&,const SlotCandidate&,const std::vector<int>&);
}
struct DynamicTsvTestAccess {
    static void exact(NegotiatedRouter& r,size_t shared=1) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<NetGuideMetrics> measured;std::vector<int> demand(r.graph_.size());GuideMap guides;
        std::map<int,size_t> slots;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto& route=r.routes_[i];
            if(route.slot>=0) require(++slots[route.slot]<=shared,"TSV ownership within the per-landing budget");
            guides[r.def_.nets()[i].name]=route.guides;
            measured.push_back(measure_official_net_guides(r.def_.nets()[i],route.guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]));
            require(measured.back().demand==route.measured.demand,"fresh demand equals route cache");
            require(measured.back().tsv_cells==route.measured.tsv_cells,"fresh TSV cells equal route cache");
            for(auto [n,d]:measured.back().demand) demand[size_t(n)]+=d;
        }
        require(demand==r.demand_,"fresh geometry demand equals aggregate ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"serialized Guides remain legal");
        std::vector<const NetGuideMetrics*> views;for(const auto& net:measured) views.push_back(&net);
        const auto fresh=aggregate_official_metrics(views,demand,r.graph_,r.thermal_,r.options_.official_metrics,false);
        const auto current=r.measure();r.verify_official_ledger(current);
        near(fresh.guide.quality,current.guide.quality,"independent full score equals ledger");
        near(fresh.guide.correlation,current.guide.correlation,"independent Pearson equals ledger");
        near(current.guide.quality,r.best_metrics_.guide.quality,"best checkpoint matches current solution");
    }
    static void risk_order(NegotiatedRouter& r) {
        std::vector<int> assignment;size_t index=r.routes_.size();
        for(size_t i=0;i<r.routes_.size();++i) {
            assignment.push_back(r.routes_[i].slot);
            if(index==r.routes_.size() && r.routes_[i].slot>=0) index=i;
        }
        require(index<r.routes_.size(),"risk test has a movable TSV owner");
        TsvMoveSearch search(r.def_,r.graph_,r.policy_,r.thermal_,assignment);
        TsvMoveSearchOptions options;options.max_candidates=1;
        options.thermal_gain=r.options_.official_metrics.thermal_gain;
        for(bool probe:{false,true}) {
            const auto before=detail::dynamic_tsv_candidates(search,r.graph_,r.policy_,r.demand_,index,
                assignment[index],r.best_metrics_.official.squared_normalized_overflow,options,probe);
            require(before.size()==1,"final candidate cap is respected");
            const int risky=before.front().slot;
            const int node=r.policy_.slots()[size_t(risky)].upper_node;
            require(r.graph_.official_capacity(node)>0,"risk fixture has positive landing capacity");
            auto congestion=r.demand_;congestion[size_t(node)]+=1000000;
            const auto after=detail::dynamic_tsv_candidates(search,r.graph_,r.policy_,congestion,index,
                assignment[index],r.best_metrics_.official.squared_normalized_overflow,options,probe);
            require(after.size()==1 && after.front().slot!=risky,
                "live congestion selects a safer raw-pool candidate before final truncation");
        }
        exact(r);
    }
    static void equivalent_index(NegotiatedRouter& r) {
        require(bool(r.dynamic_tsv_search_),"reusable TSV index exists");
        std::vector<int> assignment;for(const auto& route:r.routes_) assignment.push_back(route.slot);
        TsvMoveSearch fresh(r.def_,r.graph_,r.policy_,r.thermal_,assignment);
        near(fresh.current_correlation(),r.dynamic_tsv_search_->current_correlation(),
                "reused index retains the exact fresh occupancy correlation");
        TsvMoveSearchOptions options;options.max_candidates=8;
        options.thermal_gain=r.options_.official_metrics.thermal_gain;
        options.length_price=1e-5;options.risk_price=0.1;
        for(size_t i=0;i<assignment.size();++i) {
            require(r.dynamic_tsv_search_->current_slot(i)==assignment[i],"reused index assignment matches actual routes");
            const auto actual=r.dynamic_tsv_search_->candidates(i,options),expected=fresh.candidates(i,options);
            require(actual.size()==expected.size(),"reused index yields the fresh candidate count");
            for(size_t k=0;k<actual.size();++k) {
                require(actual[k].slot==expected[k].slot,"reused index yields the fresh ordered destination");
                near(actual[k].correlation_after,expected[k].correlation_after,"reused destination correlation matches fresh");
                near(actual[k].proxy_length_delta_dbu,expected[k].proxy_length_delta_dbu,"reused destination length proxy matches fresh");
                near(actual[k].thermal_score_delta,expected[k].thermal_score_delta,"reused destination thermal gain matches fresh");
                near(actual[k].estimated_score_delta,expected[k].estimated_score_delta,"reused destination ranking matches fresh");
            }
        }
    }
    static void index_lifetime(NegotiatedRouter& r) {
        const auto original_options=r.options_;
        r.options_.v8_tsv_dynamic=true;r.options_.tsv_index_reuse=true;
        r.options_.v9_tsv_coverage=false;r.options_.v9_thermal_regions=false;
        r.options_.tsv_landing_pricing=false;r.options_.resource_candidate_pool=false;
        r.options_.tsv_rounds=1;r.options_.v7_tsv_nets=1;r.options_.v7_tsv_candidates=8;
        r.options_.v7_tsv_seconds=20;r.options_.search_limit=0;
        r.dynamic_tsv_search_.reset();
        const auto before=geometry(r.routes_);const auto demand=r.demand_;
        const auto builds=r.counters_.tsv_index_builds;
        r.optimize_dynamic_tsv();
        require(r.counters_.tsv_index_builds==builds+1,"first reusable slice builds its index");
        require(geometry(r.routes_)==before && r.demand_==demand,"failed trials retain actual occupancy and demand");
        equivalent_index(r);
        const auto hits=r.counters_.tsv_index_hits;
        r.optimize_dynamic_tsv();
        require(r.counters_.tsv_index_builds==builds+1 && r.counters_.tsv_index_hits==hits+1,
                "another unchanged slice reuses the existing index");
        require(geometry(r.routes_)==before && r.demand_==demand,"reused failed trials retain the complete checkpoint");
        equivalent_index(r);

        // Install a real alternative route outside the dynamic phase, as an
        // independent TSV stage could do. The next slice must rebuild.
        size_t index=r.routes_.size();
        for(size_t i=0;i<r.routes_.size();++i) if(r.routes_[i].slot>=0) {index=i;break;}
        require(index<r.routes_.size(),"index invalidation fixture has a TSV net");
        TsvMoveSearchOptions query;query.max_candidates=8;
        const auto candidates=r.dynamic_tsv_search_->candidates(index,query);
        require(!candidates.empty(),"index invalidation fixture has a free destination");
        const auto old=r.routes_[index];r.account(index,old,-1);
        r.options_.search_limit=original_options.search_limit;
        try {r.routes_[index]=r.build_route(index,candidates.front().slot);r.account(index,r.routes_[index],1);}
        catch(...) {r.routes_[index]=old;r.account(index,old,1);throw;}
        r.best_metrics_=r.measure();exact(r);
        r.options_.search_limit=0;
        const auto changed=geometry(r.routes_);const auto changed_demand=r.demand_;
        r.optimize_dynamic_tsv();
        require(r.counters_.tsv_index_builds==builds+2,"external assignment change rebuilds the index");
        require(geometry(r.routes_)==changed && r.demand_==changed_demand,"assignment rebuild preserves real route state");
        equivalent_index(r);

        // A configuration fault stops the complete phase before route mutation.
        // Rejected search contexts must not survive phase rollback.
        require(!r.options_.official_metrics.tsv_track_demand,"fault fixture uses the legacy landing demand option");
        r.options_.tsv_landing_pricing=true;
        const auto restores=r.counters_.v7_checkpoint_restores;
        r.optimize_dynamic_tsv();
        require(r.counters_.v7_checkpoint_restores==restores+1 && !r.dynamic_tsv_search_,
                "phase exception drops the cached index while restoring the complete solution");
        require(geometry(r.routes_)==changed && r.demand_==changed_demand,"phase rollback retains geometry and demand");
        r.options_.tsv_landing_pricing=false;
        r.optimize_dynamic_tsv();
        require(r.counters_.tsv_index_builds==builds+3,"slice after rollback reconstructs its context");
        equivalent_index(r);

        r.account(index,r.routes_[index],-1);r.routes_[index]=old;r.account(index,old,1);
        r.best_metrics_=r.measure();exact(r);
        r.options_=original_options;r.dynamic_tsv_search_.reset();
    }
    // Shared landings: complete routing, exact count-based Pearson and ledger,
    // and unique-occupancy TSV move phases left untouched.
    static void shared_cells(NegotiatedRouter& r,size_t limit) {
        require(r.policy_.shared_cell_limit()==limit,"router keeps the explicit shared-cell budget");
        require(r.route_all(),"shared-cell fixture routes completely");
        std::map<int,size_t> load;size_t cross=0;
        for(const auto& route:r.routes_) if(route.slot>=0) {++load[route.slot];++cross;}
        size_t peak=0;for(const auto& [slot,count]:load) {(void)slot;peak=std::max(peak,count);}
        require(cross==3 && r.best_metrics_.guide.tsv_count==3,"each cross-die net keeps exactly one TSV");
        require(peak==std::min<size_t>(limit,3) && load.size()==(3+limit-1)/limit,"hub loads follow the group budget");
        require(r.policy_.shared_hubs()==load.size() && r.policy_.shared_members()==3 && r.policy_.shared_peak_load()==peak,
                "hub statistics match routed ownership");
        exact(r,limit);
        const auto before=geometry(r.routes_);const auto demand=r.demand_;const auto saved=r.options_;
        r.options_.v7_tsv=true;r.options_.v8_tsv_dynamic=true;r.options_.tsv_rounds=1;r.options_.v7_tsv_seconds=5;
        r.optimize_dynamic_tsv();r.optimize_v7_tsv();
        require(geometry(r.routes_)==before && r.demand_==demand,"unique-occupancy TSV moves leave shared hubs untouched");
        require(r.rejection_reasons_.count("dynamic TSV skipped: shared TSV cells")==1 &&
                r.rejection_reasons_.count("V7 TSV skipped: shared TSV cells")==1,"skips are recorded");
        r.options_=saved;exact(r,limit);
    }
    static void run(NegotiatedRouter& r) {
        require(r.route_all(),"fixture routes completely");exact(r);risk_order(r);
        const auto saved=r.options_;const auto outer_deadline=r.deadline_;
        const auto baseline=geometry(r.routes_);const auto demand=r.demand_;
        const auto owners=r.cell_nets_;const auto edge_owners=r.edge_nets_;
        r.options_.v8_tsv_dynamic=true;r.options_.tsv_rounds=3;
        r.options_.v7_tsv_nets=1;r.options_.v7_tsv_candidates=12;
        r.options_.v8_tsv_probe_candidates=2;r.options_.v7_tsv_seconds=20;
        r.options_.search_limit=0;
        r.optimize_dynamic_tsv();
        require(r.counters_.v8_tsv_probed_nets==3,"initial probe covers every TSV net, not just batch size");
        require(r.counters_.v8_tsv_unique_nets==3,"three one-net batches visit three different nets");
        require(r.counters_.v8_tsv_repeat_visits==0,"visited mask avoids repeating static shortlist");
        for(size_t q=0;q<3;++q) require(r.counters_.v8_tsv_queue_visits[q]>0,"each objective queue receives measured routing work");
        require(r.counters_.v7_tsv_attempted>0 && r.counters_.v7_tsv_rollbacks>0,"failed routed candidates exercise rollback");
        require(geometry(r.routes_)==baseline && r.demand_==demand,"failed candidates restore geometry and demand");
        require(r.cell_nets_==owners && r.edge_nets_==edge_owners,"failed candidates restore reverse ownership");
        require(r.deadline_==outer_deadline && !r.v5_topology_trial_,"phase restores enclosing budget");exact(r);

        // One legacy one-net batch would stop after one query. V9 must visit
        // the whole assigned set and stop only after a full no-gain sweep.
        const auto unique_before=r.counters_.v8_tsv_unique_nets,sweeps_before=r.counters_.v9_tsv_sweeps;
        r.options_.v9_tsv_coverage=true;r.options_.tsv_rounds=1;r.options_.v7_tsv_nets=1;
        r.options_.v9_tsv_stale_sweeps=2;
        const auto explore_before=r.counters_.v9_tsv_explore_visits;
        r.optimize_dynamic_tsv();
        require(r.counters_.v8_tsv_unique_nets-unique_before==3,"V9 visits all nets beyond rounds times batch size");
        require(r.counters_.v9_tsv_sweeps-sweeps_before==2,"two complete no-gain sweeps terminate coverage");
        require(r.counters_.v9_tsv_explore_visits>explore_before,"rotation is attributed independently of objective queues");
        require(r.counters_.v9_tsv_rerouted_unique>0,"actual reroute coverage is counted independently from queries");
        require(geometry(r.routes_)==baseline && r.demand_==demand,"coverage failures retain exact checkpoint");exact(r);
        // Resume a partially visited epoch with a different slice boundary.
        // It must inspect only the two remaining nets before declaring a
        // complete no-gain sweep, retaining its exploration/probe cursors.
        std::vector<size_t> assigned;
        for(size_t i=0;i<r.routes_.size();++i) if(r.routes_[i].slot>=0) assigned.push_back(i);
        r.v9_tsv_state_.visited[assigned.front()]=1;r.v9_tsv_state_.visited_count=1;
        r.v9_tsv_state_.sweep_accepted=0;r.v9_tsv_state_.stale_sweeps=0;
        const auto probe_cursor=r.v9_tsv_state_.probe_cursor,picks=r.v9_tsv_state_.picks;
        const auto repeated=r.counters_.v8_tsv_repeat_visits,sweeps=r.counters_.v9_tsv_sweeps;
        r.options_.v9_tsv_stale_sweeps=1;r.optimize_dynamic_tsv();
        require(r.counters_.v8_tsv_repeat_visits-repeated==2 && r.counters_.v9_tsv_sweeps-sweeps==1,
                "a second slice resumes only the unvisited part of the coverage epoch");
        require(r.v9_tsv_state_.probe_cursor>probe_cursor && r.v9_tsv_state_.picks==picks+2,
                "probe and objective rotation persist across calls");
        std::vector<int> empty_demand(r.graph_.size());
        const SlotCandidate* admissible=nullptr;
        for(const auto& slot:r.policy_.slots()) if(detail::tsv_region_accessible(r.graph_,slot,empty_demand)) {admissible=&slot;break;}
        require(admissible!=nullptr,"region fixture contains a two-die accessible landing");
        auto blocked=empty_demand;
        blocked[size_t(admissible->upper_node)]=int(r.graph_.official_capacity(admissible->upper_node));
        require(!detail::tsv_region_accessible(r.graph_,*admissible,blocked),"region admission rejects exhausted top-die landing capacity");
        blocked=empty_demand;const int lower_access=r.graph_.node(r.graph_.col(admissible->lower_node),r.graph_.row(admissible->lower_node),9);
        blocked[size_t(lower_access)]=int(r.graph_.official_capacity(lower_access));
        require(!detail::tsv_region_accessible(r.graph_,*admissible,blocked),"region admission rejects exhausted bottom-die access capacity");
        r.options_.v9_tsv_stale_sweeps=1;r.options_.v9_thermal_regions=true;
        r.options_.search_limit=saved.search_limit;r.options_.tsv_rounds=3;r.options_.v7_tsv_nets=3;
        const auto before=r.best_metrics_;const auto accepted=r.counters_.v7_tsv_accepted;
        r.optimize_dynamic_tsv();
        require(r.counters_.v7_tsv_accepted>accepted,"dynamic scheduling accepts an actual rerouted thermal move");
        require(r.best_metrics_.guide.quality>before.guide.quality+1e-10,"accepted moves improve exact complete score");
        require(r.counters_.v8_tsv_refreshes>0,"accepted occupancy changes refresh related net probes");exact(r);

        const auto checkpoint=geometry(r.routes_);const auto live_demand=r.demand_;
        const auto stops=r.counters_.v7_tsv_budget_stops;
        r.options_.v7_tsv_seconds=0;r.optimize_dynamic_tsv();
        require(r.counters_.v7_tsv_budget_stops>stops,"zero phase budget is reported");
        require(geometry(r.routes_)==checkpoint && r.demand_==live_demand,"expired phase retains last complete solution");
        require(r.deadline_==outer_deadline && !r.v5_topology_trial_,"timeout restores outer deadline and flags");exact(r);
        r.options_=saved;
        index_lifetime(r);
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("dynamic_tsv_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path dir=argv[1];fs::create_directories(dir);
        std::string tech,def="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 600 600 ) ; GCELLGRID X 0 DO 7 STEP 100 ; GCELLGRID Y 0 DO 7 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            const std::string tracks=layer==9?"5 DO 6 STEP 100":"5 DO 60 STEP 10";
            def+="TRACKS X "+tracks+" LAYER "+name+" ; TRACKS Y "+tracks+" LAYER "+name+" ; ";
        }
        def+="COMPONENTS 0 ; END COMPONENTS PINS 13 ; ";
        const auto pin=[&](const std::string& name,int layer,int x,int y) {
            def+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        };
        for(int i=0;i<3;++i) {pin("a"+std::to_string(i),1,50+i*100,250);pin("b"+std::to_string(i),1,450,350+i*100);pin("c"+std::to_string(i),12,150+i*100,450);}
        pin("local_a",1,50,50);pin("local_b",1,550,550);pin("local_c",1,50,50);pin("local_d",1,550,550);
        def+="END PINS NETS 5 ; ";
        for(int i=0;i<3;++i) def+="- cross"+std::to_string(i)+" ( PIN a"+std::to_string(i)+" ) ( PIN b"+std::to_string(i)+" ) ( PIN c"+std::to_string(i)+" ) ; ";
        def+="- local ( PIN local_a ) ( PIN local_b ) ; - duplicate ( PIN local_c ) ( PIN local_d ) ; END NETS END DESIGN";
        LefParser lef;require(lef.parse(write(dir/"fixture.lef",tech)),"parse technology");
        DefParser parsed;require(parsed.parse(write(dir/"fixture.def",def),lef,DefGridMapping::OFFICIAL_FULL),"parse design");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<6;++y) for(int x=0;x<6;++x) {
            if(x||y) heat<<',';
            heat<<'['<<5+x*10<<','<<5+y*10<<",0,"<<26+x+6*y<<']';
        }
        heat<<"]}";
        ThermalParser thermal;require(thermal.parse_json(write(dir/"heat.json",heat.str()),parsed.gcell_grid(),
            parsed.dbu_per_micron(),parsed.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse heat");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        options.official_metrics.score_20261020=true;options.official_metrics.thermal_gain=3;
        options.official_metrics.gcell_width_dbu=parsed.gcell_pitch_x();options.official_metrics.gcell_height_dbu=parsed.gcell_pitch_y();
        options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;
        options.v7_cell_cost=true;options.time_limit=120;
        NegotiatedRouter router(parsed,lef,thermal,options);DynamicTsvTestAccess::run(router);
        for(size_t limit:{size_t(2),size_t(3)}) {
            auto shared=options;shared.tsv_policy.shared_cell_limit=limit;
            NegotiatedRouter shared_router(parsed,lef,thermal,shared);DynamicTsvTestAccess::shared_cells(shared_router,limit);
        }
        auto invalid=options;invalid.tsv_policy.shared_cell_limit=2;invalid.future_tsv_reservation=true;
        invalid.official_metrics.tsv_track_demand=true;bool rejected=false;
        try {NegotiatedRouter bad(parsed,lef,thermal,invalid);}
        catch(const std::invalid_argument& error) {rejected=std::string(error.what()).find("shared TSV cells")!=std::string::npos;}
        require(rejected,"shared landings reject the untested future-reservation combination");
        std::cout<<"dynamic_tsv_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
