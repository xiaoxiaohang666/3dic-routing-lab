#include "router/negotiated_router.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool condition,const std::string& reason) {
    ++checks;if(!condition) throw std::runtime_error(reason);
}
std::string write(const fs::path& file,const std::string& text) {
    std::ofstream out(file);out<<text;out.close();
    if(!out) throw std::runtime_error("could not write route-order fixture");
    return file.string();
}
std::string pin(const std::string& name,const std::string& layer,int x,int y) {
    return "- "+name+" + LAYER "+layer+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
}
}
namespace eda {
struct PinLocalTestAccess {
    static std::vector<size_t> order(NegotiatedRouter& router,const std::vector<int>& assignment) { return router.initial_route_order(assignment); }
    static RouteOrderTerminal terminal(const NegotiatedRouter& router,size_t net,int col,int row,int layer) {
        return router.route_order_terminal(net,col,row,layer,layer>10?11:1,layer>10?20:10);
    }
    static int slot_at(const NegotiatedRouter& router,const RoutingGraph& graph,int col,int row) {
        for(const auto& slot:router.policy_.slots())
            if(graph.grid().get_col(slot.point.x)==col && graph.grid().get_row(slot.point.y)==row) return slot.id;
        throw std::runtime_error("no TSV slot in that column");
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("fixture directory and official runtime required");
        const fs::path folder=argv[1],runtime=argv[2];fs::create_directories(folder);
        for(const auto* asset:{"libflute.so","tse_framework_pybind_flute.cpython-310-x86_64-linux-gnu.so","POWV9.dat","POST9.dat"})
            if(!fs::is_regular_file(runtime/asset)) {std::cout<<"SKIP route order: native runtime unavailable at "<<runtime<<'\n';return 0;}
        std::string technology;
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            technology+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.1 ; END "+name+"\n";
        }
        // 90 x 90 um, 9 GCells each way. "wall" leaves only M3 open at the low
        // layers, so a pin in its middle can run along its row but cannot turn.
        // "deep" blocks the layers under M10, so a TSV landing there cannot leave.
        technology+="MACRO wall CLASS BLOCK ; SIZE 90 BY 90 ; "
            "PIN c PORT LAYER metal3 ; RECT 44 44 46 46 ; END END c "
            "PIN e PORT LAYER metal3 ; RECT 0.5 44 1.5 46 ; END END e "
            "OBS LAYER metal1 ; RECT 0 0 90 90 ; LAYER metal2 ; RECT 0 0 90 90 ; LAYER metal4 ; RECT 0 0 90 90 ; LAYER metal5 ; RECT 0 0 90 90 ; END END wall\n";
        technology+="MACRO deep CLASS BLOCK ; SIZE 90 BY 90 ; "
            "OBS LAYER metal6 ; RECT 0 0 90 90 ; LAYER metal7 ; RECT 0 0 90 90 ; LAYER metal8 ; RECT 0 0 90 90 ; LAYER metal9 ; RECT 0 0 90 90 ; END END deep\n";
        std::string design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 3000 3000 ) ; "
            "GCELLGRID X 0 DO 31 STEP 100 ; GCELLGRID Y 0 DO 31 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            design+="TRACKS X 5 DO 300 STEP 10 LAYER "+name+" ; TRACKS Y 5 DO 300 STEP 10 LAYER "+name+" ; ";
        }
        design+="COMPONENTS 2 ; - u0 wall + PLACED ( 500 500 ) N ; - u1 deep + PLACED ( 1800 500 ) N ; END COMPONENTS PINS 10 ; "+
            pin("far0","metal3",1450,950)+pin("far1","metal3",350,950)+
            pin("p1","metal3",100,2500)+pin("p2","metal3",2900,2500)+
            pin("p3","metal3",100,2000)+pin("top","metal12",2900,2000)+
            pin("p4","metal3",100,2800)+pin("p5","metal3",300,2800)+pin("p6","metal3",500,2800)+
            pin("p7","metal3",1000,2800)+"END PINS NETS 5 ; "
            "- hard ( u0 c ) ( PIN far0 ) ; "
            "- edge ( u0 e ) ( PIN far1 ) ; "
            "- plain ( PIN p1 ) ( PIN p2 ) ; "
            "- cross ( PIN p3 ) ( PIN top ) ; "
            "- fan ( PIN p4 ) ( PIN p5 ) ( PIN p6 ) ; "
            "END NETS END DESIGN";
        LefParser lef;DefParser def;ThermalParser thermal;
        require(lef.parse(write(folder/"order.lef",technology)) &&
                def.parse(write(folder/"order.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parse failed");
        RouterOptions base;base.official_model=true;base.official_runtime=runtime.string();
        base.graph_options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;base.graph_options.allow_nonpreferred=false;
        base.graph_options.score_20261020=true;base.graph_options.documented_rules=true;
        base.official_metrics.score_20261020=true;base.official_metrics.gcell_width_dbu=def.gcell_pitch_x();
        base.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
        base.score_unit_cost=true;base.fast_initial=true;base.fast_quality=true;
        base.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        RoutingGraph graph(def,lef,base.graph_options);
        std::map<std::string,size_t> index;
        for(size_t i=0;i<def.nets().size();++i) index[def.nets()[i].name]=i;
        const auto names=[&](const std::vector<size_t>& order) {
            std::string text;for(size_t i:order) text+=def.nets()[i].name+" ";
            return text;
        };
        const auto run=[&](int mode,const std::vector<int>& assignment,RouterCounters* counters=nullptr) {
            RouterOptions options=base;options.route_order_mode=mode;
            NegotiatedRouter router(def,lef,thermal,options);
            auto order=PinLocalTestAccess::order(router,assignment);
            if(counters) *counters=router.counters();
            return names(order);
        };
        const std::vector<int> no_slots(def.nets().size(),-1);

        // The fixture really has full cells where the formula expects them.
        const int over_centre=graph.node(9,9,4);
        require(graph.official_obstacle_demand(over_centre)>=graph.official_capacity(over_centre),"the macro must fill M4 over its pin");
        require(graph.official_obstacle_demand(graph.node(9,9,3))<graph.official_capacity(graph.node(9,9,3)),"the macro must leave M3 open");

        RouterOptions radius_three=base;radius_three.climb_escape_radius=3;
        {
            NegotiatedRouter router(def,lef,thermal,radius_three);
            // Middle of the macro: M4 and M5 full before M6/M7, nowhere to turn nearby.
            const auto middle=PinLocalTestAccess::terminal(router,index.at("hard"),9,9,3);
            require(middle.straight_full==2,"two full cells above the middle pin");
            require(middle.usable_columns==0 && middle.window_columns==49,"no usable column around the middle pin");
            // Edge of the macro: the same climb, but open columns are one step away.
            const auto edge=PinLocalTestAccess::terminal(router,index.at("edge"),5,9,3);
            require(edge.straight_full==2,"two full cells above the edge pin");
            require(edge.usable_columns>=21,"columns outside the macro are usable from its edge");
            // Open area: no flood at all, and the die boundary does not matter.
            const auto open=PinLocalTestAccess::terminal(router,index.at("plain"),1,25,3);
            require(open.straight_full==0,"a free column is easy");
        }

        // Mode 0 is today's order and does no difficulty work.
        RouterCounters counters;
        require(run(0,no_slots,&counters)=="cross fan plain hard edge ","mode 0 must keep cross-die, pins, span");
        require(counters.route_order_hard_nets==0 && counters.route_order_blocked_terminals==0,"mode 0 must not compute difficulty");

        // Mode 1 moves only the blocked nets, the one with the fewest open columns
        // first. The others keep today's relative order.
        require(run(1,no_slots,&counters)=="hard edge cross fan plain ","mode 1 must put blocked nets first and keep cross-die first");
        require(counters.route_order_hard_nets==2 && counters.route_order_promoted_nets==2,"two nets are blocked and moved earlier");
        require(counters.route_order_blocked_terminals==2,"two pins are blocked");

        // Mode 2 drops the cross-die key: the three-pin net now passes the cross-die net.
        require(run(2,no_slots,&counters)=="hard edge fan cross plain ","mode 2 must rank the rest by pins and span only");
        require(counters.route_order_hard_nets==2,"mode 2 finds the same blocked nets");

        // A cross-die net with open pins can still be hard because of its landing:
        // M9..M6 are full below it, so the climb crosses four cells with no way out.
        std::vector<int> landing=no_slots;
        {
            NegotiatedRouter router(def,lef,thermal,base);
            landing[index.at("cross")]=PinLocalTestAccess::slot_at(router,graph,22,9);
        }
        require(run(1,landing,&counters)=="cross hard edge fan plain ","a blocked TSV landing must count, and the harder net goes first");
        require(counters.route_order_hard_nets==3,"the cross-die net and both wall nets are hard");
        require(counters.route_order_promoted_nets==2,"the cross-die net was first already, so two nets moved earlier");

        // Out-of-range modes are refused instead of silently clamped.
        RouterOptions wrong=base;wrong.route_order_mode=3;bool refused=false;
        try {NegotiatedRouter router(def,lef,thermal,wrong);} catch(const std::invalid_argument&) {refused=true;}
        require(refused,"mode above the limit must be refused");

        std::cout<<"PASS "<<checks<<" route-order checks\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
