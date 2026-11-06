#include "router/negotiated_router.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool value,const std::string& reason) {++checks;if(!value) throw std::runtime_error(reason);}
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
}
namespace eda {
// Rotation must only reorder which nets budgeted recovery attempts: a net
// attempted earlier in the sweep is skipped until every other qualifying net
// has been attempted, and an exhausted sweep starts again from the full list.
struct RecoveryRotationTestAccess {
    static void run(NegotiatedRouter& r) {
        // The global-W shortlist admits every routed net, so the fixture does
        // not depend on whether a net ends above its own reference.
        r.options_.recovery_global_wirelength=true;r.options_.recovery_rotate=false;
        const auto full=r.recovery_selection();
        require(full.size()==r.routes_.size(),"global shortlist holds every routed net ("+std::to_string(full.size())+")");
        r.options_.recovery_rotate=true;
        require(r.recovery_selection()==full,"rotation without any attempt keeps the historical order");
        r.recovery_tried_.assign(r.routes_.size(),0);
        require(r.recovery_selection()==full,"empty sweep keeps the historical order");
        r.recovery_tried_[full.front()]=1;
        auto rest=full;rest.erase(rest.begin());
        require(r.recovery_selection()==rest,"attempted net waits and the others keep their order");
        for(size_t i:full) r.recovery_tried_[i]=1;
        require(r.recovery_selection().empty(),"complete sweep leaves nothing to select");
        // The per-net excess shortlist skips the same marks.
        r.options_.recovery_global_wirelength=false;
        require(r.recovery_selection().empty(),"excess shortlist also skips attempted nets");
        r.recovery_tried_.assign(r.routes_.size(),0);
        const auto excess=r.recovery_selection();
        if(!excess.empty()) {
            r.recovery_tried_[excess.front()]=1;
            auto tail=excess;tail.erase(tail.begin());
            require(r.recovery_selection()==tail,"excess shortlist keeps its order without the attempted net");
        }
        // The recovery phase itself begins a new sweep instead of idling.
        r.options_.recovery_global_wirelength=true;
        for(size_t i:full) r.recovery_tried_[i]=1;
        r.options_.large_recovery_seconds=30;r.options_.large_recovery_rounds=1;
        const auto sweeps=r.recovery_rotation_sweeps_,marked=r.recovery_distinct_attempted_;
        r.recover_budgeted_wirelength();
        require(r.recovery_rotation_sweeps_==sweeps+1,"exhausted sweep restarts ("+std::to_string(r.recovery_rotation_sweeps_-sweeps)+")");
        require(r.recovery_distinct_attempted_>marked,"new sweep attempts nets again");
        const auto tried=size_t(std::count(r.recovery_tried_.begin(),r.recovery_tried_.end(),uint8_t{1}));
        require(tried==r.recovery_distinct_attempted_-marked,"each attempted net is marked once per sweep");
        r.options_.recovery_rotate=false;
        require(r.recovery_selection().size()==r.routes_.size(),"rotation off ignores the marks");
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("recovery_rotation_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path directory=argv[1];fs::create_directories(directory);
        std::string tech,def="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 180 180 ) ; GCELLGRID X 0 DO 7 STEP 30 ; GCELLGRID Y 0 DO 7 STEP 30 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            def+="TRACKS X 1 DO 180 STEP 1 LAYER "+name+" ; TRACKS Y 1 DO 180 STEP 1 LAYER "+name+" ; ";
        }
        def+="COMPONENTS 0 ; END COMPONENTS PINS 10 ; ";
        const auto pin=[&](const std::string& name,int layer,int x,int y) {
            def+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        };
        // The second pin of each net sits on metal5, so every route needs a via
        // climb. One cross-die net gives the design a TSV, as in production.
        pin("a0",1,15,15);pin("a1",5,135,105);pin("b0",1,45,165);pin("b1",5,165,45);
        pin("c0",2,25,75);pin("c1",5,155,140);pin("d0",1,75,25);pin("d1",5,105,155);
        pin("x0",1,45,45);pin("x1",13,135,135);
        def+="END PINS NETS 5 ; - a ( PIN a0 ) ( PIN a1 ) ; - b ( PIN b0 ) ( PIN b1 ) ; - c ( PIN c0 ) ( PIN c1 ) ; - d ( PIN d0 ) ( PIN d1 ) ; "
             "- x ( PIN x0 ) ( PIN x1 ) ; END NETS END DESIGN";
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
        options.v7_cell_cost=true;options.v8_rrr=true;options.time_limit=120;options.large_recovery=true;
        NegotiatedRouter router(parsed,lef,thermal,options);
        require(router.route_all(),"fixture routes");
        RecoveryRotationTestAccess::run(router);
        std::cout<<"recovery_rotation_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
