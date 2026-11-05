#include "router/negotiated_router.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

using namespace eda;
namespace fs=std::filesystem;
namespace {
using Wall=std::array<int,4>; // column, row, lowest layer, highest layer
void require(bool condition,const std::string& reason) {
    if(!condition) throw std::runtime_error(reason);
}
std::string write(const fs::path& file,const std::string& text) {
    std::ofstream out(file);out<<text;out.close();
    require(bool(out),"could not write climb-escape fixture");return file.string();
}
void check_path(const PathResult& path,const RoutingGraph& graph,int source,int target) {
    require(path.status==PathResult::Status::FOUND && !path.nodes.empty(),"no path found");
    require(path.nodes.front()==source && path.nodes.back()==target,"path lost a terminal");
    for(size_t i=1;i<path.nodes.size();++i) require(graph.legal_edge(path.nodes[i-1],path.nodes[i]),"illegal edge in path");
    auto sorted=path.nodes;std::sort(sorted.begin(),sorted.end());
    require(std::adjacent_find(sorted.begin(),sorted.end())==sorted.end(),"path visits a node twice");
}
bool crosses_column(const PathResult& path,const RoutingGraph& graph,int col,int row,int low,int high) {
    for(int n:path.nodes) if(graph.col(n)==col && graph.row(n)==row && graph.layer(n)>=low && graph.layer(n)<=high) return true;
    return false;
}
// Every layer of a column except the pin layer M3 the terminal sits on.
bool crosses_wall(const PathResult& path,const RoutingGraph& graph,int col,int row) {
    return crosses_column(path,graph,col,row,1,2) || crosses_column(path,graph,col,row,4,10);
}
size_t nodes_on_layer(const PathResult& path,const RoutingGraph& graph,int layer) {
    return size_t(std::count_if(path.nodes.begin(),path.nodes.end(),[&](int n){return graph.layer(n)==layer;}));
}
}
namespace eda {
struct PinLocalTestAccess {
    static void begin_refinement(NegotiatedRouter& router) { router.initial_build_=false; }
    static void fill(NegotiatedRouter& router,const RoutingGraph& graph,const Wall& wall) {
        for(int layer=wall[2];layer<=wall[3];++layer) {
            const int node=graph.node(wall[0],wall[1],layer);
            router.demand_.at(size_t(node))=int(graph.official_capacity(node));
        }
    }
};
}
int main(int argc,char** argv) {
    try {
        require(argc==3,"fixture directory and official runtime required");
        const fs::path folder=argv[1],runtime=argv[2];fs::create_directories(folder);
        for(const auto* asset:{"libflute.so","tse_framework_pybind_flute.cpython-310-x86_64-linux-gnu.so","POWV9.dat","POST9.dat"})
            if(!fs::is_regular_file(runtime/asset)) {std::cout<<"SKIP climb escape: native runtime unavailable at "<<runtime<<'\n';return 0;}
        std::string technology;
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            technology+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.1 ; END "+name+"\n";
        }
        std::string design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 2000 2000 ) ; "
            "GCELLGRID X 0 DO 21 STEP 100 ; GCELLGRID Y 0 DO 21 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            design+="TRACKS X 5 DO 200 STEP 10 LAYER "+name+" ; TRACKS Y 5 DO 200 STEP 10 LAYER "+name+" ; ";
        }
        design+="COMPONENTS 0 ; END COMPONENTS PINS 2 ; "
            "- a + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 250 250 ) N ; "
            "- b + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 1450 1450 ) N ; END PINS NETS 1001 ; ";
        for(int i=0;i<1001;++i) design+="- n"+std::to_string(i)+" ( PIN a ) ( PIN b ) ; ";
        design+="END NETS END DESIGN";
        LefParser lef;DefParser def;ThermalParser thermal;
        require(lef.parse(write(folder/"climb.lef",technology)) &&
                def.parse(write(folder/"climb.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parse failed");
        RouterOptions off;off.official_model=true;off.official_runtime=runtime.string();
        off.graph_options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;off.graph_options.allow_nonpreferred=false;
        off.graph_options.score_20261020=true;off.graph_options.documented_rules=true;
        off.official_metrics.score_20261020=true;off.official_metrics.gcell_width_dbu=def.gcell_pitch_x();
        off.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
        off.score_unit_cost=true;off.fast_initial=true;off.fast_quality=true;off.search_limit=50000;
        RouterOptions on=off;on.climb_escape_radius=3;
        RoutingGraph graph(def,lef,off.graph_options);
        const int source=graph.node(2,2,3),target=graph.node(14,14,3);
        struct Case {int from,to,high;bool refining;};
        const auto route_case=[&](const RouterOptions& options,const std::vector<Wall>& walls,const Case& c,size_t* searches=nullptr) {
            NegotiatedRouter router(def,lef,thermal,options);
            if(c.refining) PinLocalTestAccess::begin_refinement(router);
            for(const auto& wall:walls) PinLocalTestAccess::fill(router,graph,wall);
            PinAccess access;access.node=c.from;
            NetRoute own;own.nodes.insert(c.to);
            auto path=router.search({access},{c.to},own,1,c.high,false);
            if(searches) *searches=router.counters().climb_escape_searches;
            return path;
        };
        // A wall alone would not force a climb: an L can stay on the pin layer or
        // run under the wall on M1/M2. So the column is full above and below the
        // pin, and the pin-layer run is full too, except the two cells next to the
        // terminal where the net can step aside. Every template then pays for at
        // least one full cell, as with a pin under a macro.
        std::vector<Wall> over_source{{2,2,1,2},{2,2,4,10}},over_target{{14,14,1,2},{14,14,4,10}};
        for(int c=5;c<=14;++c) over_source.push_back({c,2,3,3});
        for(int c=2;c<=11;++c) over_target.push_back({c,14,3,3});
        auto over_both=over_source;over_both.insert(over_both.end(),over_target.begin(),over_target.end());
        const Case refine{source,target,10,true},initial{source,target,10,false};
        size_t checks=0;

        // A free column keeps today's template and never builds an escape field.
        for(const auto& mode:{refine,initial}) {
            size_t searches=9;
            const auto quiet_off=route_case(off,{},mode);
            const auto quiet_on=route_case(on,{},mode,&searches);
            require(quiet_on.nodes==quiet_off.nodes && quiet_on.cost==quiet_off.cost,"free columns must keep the same path");++checks;
            require(searches==0,"free columns must not trigger the escape search");++checks;
        }

        // A full column over the source: without the option the template climbs
        // through it; with the option the path steps aside. Initial routing and
        // RRR share the search, and both must get the new candidate.
        for(const auto& mode:{refine,initial}) {
            size_t searches=0;
            const auto blocked_off=route_case(off,over_source,mode);
            check_path(blocked_off,graph,source,target);
            require(crosses_wall(blocked_off,graph,2,2),"fixture must force today's template through the full column");++checks;
            const auto blocked_on=route_case(on,over_source,mode,&searches);
            check_path(blocked_on,graph,source,target);
            require(searches>0,"a full column must trigger the escape search");++checks;
            require(!crosses_wall(blocked_on,graph,2,2),"the climb must leave the full source column");++checks;
            require(blocked_on.cost<blocked_off.cost,"the escape must be cheaper than crossing full cells");++checks;
        }

        // The same wall over the target needs the descent to shift as well.
        const auto far_off=route_case(off,over_target,refine);
        check_path(far_off,graph,source,target);
        require(crosses_wall(far_off,graph,14,14),"fixture must force today's descent through the full column");++checks;
        const auto far_on=route_case(on,over_target,refine);
        check_path(far_on,graph,source,target);
        require(!crosses_wall(far_on,graph,14,14),"the descent must leave the full target column");++checks;

        // Both ends at once.
        const auto both_on=route_case(on,over_both,refine);
        check_path(both_on,graph,source,target);
        require(!crosses_wall(both_on,graph,2,2) && !crosses_wall(both_on,graph,14,14),"both climbs must leave their columns");++checks;

        // A TSV landing sits on M10 while the lower die routes only up to M9.
        // The landing may be either end of the connection. It is reached only
        // through M9 of its own column, so that cell stays on the path, but the
        // rest of the full column can be avoided and no other M10 cell is used.
        const int landing=graph.node(14,14,10);
        const Wall under_landing{14,14,4,9};
        for(const auto& mode:{Case{source,landing,9,true},Case{landing,source,9,true}}) {
            size_t searches=0;
            const auto path=route_case(on,{under_landing},mode,&searches);
            check_path(path,graph,mode.from,mode.to);
            require(searches>0,"a full column under the landing must trigger the escape search");++checks;
            require(!crosses_column(path,graph,14,14,4,8),"the route must leave the column under the landing");++checks;
            require(nodes_on_layer(path,graph,10)==1,"M10 is closed to everything except the landing");++checks;
        }

        // Nowhere to go inside the radius: the option may not make the path worse.
        std::vector<Wall> region;
        for(int r=0;r<=5;++r) for(int c=0;c<=5;++c) region.push_back({c,r,1,10});
        const auto region_off=route_case(off,region,refine);
        const auto region_on=route_case(on,region,refine);
        check_path(region_on,graph,source,target);
        require(region_on.cost<=region_off.cost,"the escape candidate may only replace a more expensive path");++checks;

        // Out-of-range radii are refused instead of silently clamped.
        RouterOptions wide=on;wide.climb_escape_radius=9;bool refused=false;
        try {NegotiatedRouter router(def,lef,thermal,wide);} catch(const std::invalid_argument&) {refused=true;}
        require(refused,"radius above the limit must be refused");++checks;

        std::cout<<"PASS "<<checks<<" climb-escape checks\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
