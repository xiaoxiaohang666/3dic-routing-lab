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
void require(bool value,const std::string& reason) {++checks;if(!value) throw std::runtime_error(reason);}
void near(double a,double b,const std::string& reason) {
    require(std::abs(a-b)<=1e-10*std::max({1.0,std::abs(a),std::abs(b)}),reason);
}
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
}
#include "objectives/net_cell_contribution.hpp"
namespace eda {
struct FutureTsvReservationTestAccess {
    static void check_q(NegotiatedRouter& r) {
        long double expected=0;
        for(auto [node,amount]:r.future_tsv_demand_) {
            const auto before=official_resource_cell_contribution(r.graph_,node,r.demand_[node]);
            const auto after=official_resource_cell_contribution(r.graph_,node,r.demand_[node]+amount);
            expected+=after.squared-before.squared;
        }
        near(double(expected),double(r.future_tsv_squared_delta_),"search promise Q equals full recomputation");
    }
    static void run(NegotiatedRouter& r) {
        const auto assignment=r.policy_.assign(r.def_.nets(),r.thermal_);
        r.routes_.resize(assignment.size());r.prepare_tsv_reservations(assignment);
        require(r.counters_.future_tsv_reserved_nets==2,"both nonexempt TSV nets are reserved before search");
        require(!r.future_tsv_demand_.empty(),"pending TSV roots are visible");
        require(r.cell_nets_.empty() && r.edge_nets_.empty(),"promises do not claim real resource owners");
        require(std::all_of(r.demand_.begin(),r.demand_.end(),[](int n){return n==0;}),"promises do not enter final demand");
        check_q(r);
        const auto all=r.future_tsv_demand_;const auto actual=r.demand_;
        r.update_tsv_reservation(0,-1);check_q(r);
        require(r.counters_.future_tsv_remaining==1,"only own promise is released");
        for(auto [node,amount]:r.future_tsv_by_net_[1])
            require(r.search_cell_demand(node)>=amount,"later TSV remains visible to early search");
        bool duplicate=false;try {r.update_tsv_reservation(0,-1);} catch(const std::logic_error&) {duplicate=true;}
        require(duplicate,"duplicate release is rejected without mutation");
        r.update_tsv_reservation(0,1);check_q(r);
        require(r.future_tsv_demand_==all && r.demand_==actual,"release/restore returns exact separate ledgers");
        // A genuinely failed path search must restore the promise and all owners.
        const auto saved=r.options_;
        r.options_.fast_initial=false;r.options_.local_repair=false;r.options_.pin_local=false;r.options_.search_limit=0;
        bool failed=false;try {r.route_initial_net(0,assignment[0]);} catch(const std::exception&) {failed=true;}
        require(failed,"forced initial search failure is exercised");
        require(r.future_tsv_demand_==all && r.demand_==actual && r.cell_nets_.empty() && r.edge_nets_.empty(),
            "failed build restores promises, real demand and owners");check_q(r);r.options_=saved;
        for(size_t i=0;i<assignment.size();++i) {r.route_initial_net(i,assignment[i]);check_q(r);}
        require(r.future_tsv_demand_.empty() && r.counters_.future_tsv_remaining==0,"all promises hand off to actual Guide");
        require(r.future_tsv_squared_delta_==0,"search-only Q is exactly cleared");
        r.initial_build_=false;r.validate_complete();auto measured=r.measure();r.verify_official_ledger(measured);
        std::vector<int> full(r.graph_.size());
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto fresh=measure_official_net_guides(r.def_.nets()[i],r.routes_[i].guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]);
            for(auto [node,amount]:fresh.demand) full[node]+=amount;
        }
        require(full==r.demand_,"final demand equals fresh exported geometry without promises");
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("future_tsv_reservation_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path directory=argv[1];fs::create_directories(directory);
        std::string tech,def="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 180 180 ) ; GCELLGRID X 0 DO 7 STEP 30 ; GCELLGRID Y 0 DO 7 STEP 30 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            const std::string tracks=layer==11?"5 DO 18 STEP 10":"1 DO 180 STEP 1";
            def+="TRACKS X "+tracks+" LAYER "+name+" ; TRACKS Y "+tracks+" LAYER "+name+" ; ";
        }
        def+="COMPONENTS 0 ; END COMPONENTS PINS 4 ; ";
        const auto pin=[&](const std::string& name,int layer,int x,int y) {
            def+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        };
        pin("bottom",1,15,15);pin("top",13,165,165);
        pin("passer_a",1,15,165);pin("passer_b",13,165,15);
        def+="END PINS NETS 2 ; - cross ( PIN bottom ) ( PIN top ) ; - passer ( PIN passer_a ) ( PIN passer_b ) ; END NETS END DESIGN";
        LefParser lef;require(lef.parse(write(directory/"fixture.lef",tech)),"parse LEF");
        DefParser parsed;require(parsed.parse(write(directory/"fixture.def",def),lef,DefGridMapping::OFFICIAL_FULL),"parse DEF");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<6;++y) for(int x=0;x<6;++x) {
            if(x||y) heat<<',';
            heat<<'['<<1.5+x*3<<','<<1.5+y*3<<",0,"<<26+x+6*y<<']';
        }
        heat<<"]}";ThermalParser thermal;
        require(thermal.parse_json(write(directory/"heat.json",heat.str()),parsed.gcell_grid(),parsed.dbu_per_micron(),
                parsed.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse heat");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        options.official_metrics.score_20261020=true;options.official_metrics.tsv_track_demand=true;
        options.official_metrics.thermal_gain=3;options.official_metrics.gcell_width_dbu=parsed.gcell_pitch_x();
        options.official_metrics.gcell_height_dbu=parsed.gcell_pitch_y();options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;
        options.v7_cell_cost=true;options.v8_rrr=true;options.time_limit=120;
        options.future_tsv_reservation=true;
        NegotiatedRouter router(parsed,lef,thermal,options);FutureTsvReservationTestAccess::run(router);
        NegotiatedRouter complete(parsed,lef,thermal,options);
        require(complete.route_all(),"complete reserved initial routing succeeds");
        require(complete.counters().future_tsv_remaining==0,"successful route_all exports no promises");
        std::cout<<"future_tsv_reservation_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
