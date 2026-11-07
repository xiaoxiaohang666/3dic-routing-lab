#include "router/negotiated_router.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
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
}
namespace eda {
struct PinLocalTestAccess {
    static void begin_refinement(NegotiatedRouter& router) { router.initial_build_=false; }
    static void fill(NegotiatedRouter& router,const RoutingGraph& graph,int col,int row,int low,int high) {
        for(int layer=low;layer<=high;++layer) {
            const int node=graph.node(col,row,layer);
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
        PinAccess access;access.node=source;
        const auto route=[&](const RouterOptions& options,bool wall_source,bool wall_target,size_t* searches=nullptr) {
            NegotiatedRouter router(def,lef,thermal,options);
            PinLocalTestAccess::begin_refinement(router);
            if(wall_source) PinLocalTestAccess::fill(router,graph,2,2,4,10);
            if(wall_target) PinLocalTestAccess::fill(router,graph,14,14,4,10);
            NetRoute own;own.nodes.insert(target);
            auto path=router.search({access},{target},own,1,10,false);
            if(searches) *searches=router.counters().climb_escape_searches;
            return path;
        };
        size_t checks=0;

        // A free column keeps today's template and never builds an escape field.
        size_t searches=9;
        const auto quiet_off=route(off,false,false);
        const auto quiet_on=route(on,false,false,&searches);
        require(quiet_on.nodes==quiet_off.nodes && quiet_on.cost==quiet_off.cost,"free columns must keep the same path");++checks;
        require(searches==0,"free columns must not trigger the escape search");++checks;

        // A full column over the source: without the option the template climbs
        // through it; with the option the path steps aside on the pin layer.
        const auto blocked_off=route(off,true,false);
        check_path(blocked_off,graph,source,target);
        require(crosses_column(blocked_off,graph,2,2,4,10),"fixture must force today's template through the full column");++checks;
        const auto blocked_on=route(on,true,false,&searches);
        check_path(blocked_on,graph,source,target);
        require(searches>0,"a full column must trigger the escape search");++checks;
        require(!crosses_column(blocked_on,graph,2,2,4,10),"the climb must leave the full source column");++checks;
        require(blocked_on.cost<blocked_off.cost,"the escape must be cheaper than crossing full cells");++checks;

        // The same wall over the target needs the descent to shift as well.
        const auto far_off=route(off,false,true);
        check_path(far_off,graph,source,target);
        require(crosses_column(far_off,graph,14,14,4,10),"fixture must force today's descent through the full column");++checks;
        const auto far_on=route(on,false,true);
        check_path(far_on,graph,source,target);
        require(!crosses_column(far_on,graph,14,14,4,10),"the descent must leave the full target column");++checks;

        // Both ends at once.
        const auto both_on=route(on,true,true);
        check_path(both_on,graph,source,target);
        require(!crosses_column(both_on,graph,2,2,4,10) && !crosses_column(both_on,graph,14,14,4,10),"both climbs must leave their columns");++checks;

        // Out-of-range radii are refused instead of silently clamped.
        RouterOptions wide=on;wide.climb_escape_radius=9;bool refused=false;
        try {NegotiatedRouter router(def,lef,thermal,wide);} catch(const std::invalid_argument&) {refused=true;}
        require(refused,"radius above the limit must be refused");++checks;

        std::cout<<"PASS "<<checks<<" climb-escape checks\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
