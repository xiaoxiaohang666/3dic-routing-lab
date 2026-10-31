#include "router/tsv_policy.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool value,const std::string& message) {
    ++checks;if(!value) throw std::runtime_error(message);
}
template<class F> void rejects_with(F&& action,const std::string& expected) {
    std::string reason;try {action();} catch(const std::exception& error) {reason=error.what();}
    require(reason.find(expected)!=std::string::npos,"expected rejection: "+expected+"; actual: "+reason);
}
std::string write(const fs::path& file,const std::string& text) {
    std::ofstream out(file);out<<text;out.close();if(!out) throw std::runtime_error("write fixture");return file.string();
}
std::string technology() {
    std::ostringstream out;
    for(int layer=1;layer<=20;++layer) {
        const auto name=get_layer_name(layer);
        out<<"LAYER "<<name<<" TYPE ROUTING ; DIRECTION "<<(layer%2?"HORIZONTAL":"VERTICAL")
           <<" ; WIDTH 0.2 ; END "<<name<<'\n';
    }
    return out.str();
}
std::string tracks() {
    std::ostringstream out;
    for(int layer=1;layer<=20;++layer)
        out<<"TRACKS "<<(layer%2?'Y':'X')<<" 25 DO 10 STEP 100 LAYER "<<get_layer_name(layer)<<" ; ";
    return out.str();
}
RoutingGraphOptions graph_options() {
    RoutingGraphOptions options;options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;
    options.score_20261020=true;options.documented_rules=true;return options;
}
TsvPolicyOptions policy_options(TsvObstacleScope scope) {
    TsvPolicyOptions options;options.resource_mode=TsvResourceMode::GLOBAL_GRID;options.tsv_obstacle_scope=scope;return options;
}
std::vector<GuideRect> witness(const SlotCandidate& slot) {
    const auto& r=slot.region;
    return {{r.x1,r.y1,r.x2,r.y2,"metal10"},{r.x1,r.y1,r.x2,r.y2,"metal11"}};
}
void geometry_and_sources(const fs::path& dir) {
    const std::string macro=
        "MACRO fixed CLASS BLOCK ; SIZE 50 BY 50 ; "
        "PIN UNUSED PORT LAYER metal10 ; RECT 1 1 1.1 1.1 ; END END UNUSED "
        "PIN VDD USE POWER ; PORT LAYER metal2 ; RECT 11 1 11.1 1.1 ; END END VDD "
        "PIN MULTI PORT LAYER metal3 ; RECT 21 1 21.1 1.1 ; LAYER metal11 ; RECT 31 1 31.1 1.1 ; END END MULTI "
        "OBS LAYER metal10 ; RECT 1 11 1.1 11.1 ; LAYER metal11 ; RECT 10 20 20 21 ; END END fixed ";
    LefParser lef;require(lef.parse(write(dir/"sources.lef",technology()+macro)),"parse mask source masters");
    const std::string design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 500 500 ) ; "
        "GCELLGRID X 0 DO 6 STEP 100 ; GCELLGRID Y 0 DO 6 STEP 100 ; "+tracks()+
        "COMPONENTS 1 ; - u fixed + PLACED ( 0 0 ) N ; END COMPONENTS "
        "PINS 4 ; - rotated + LAYER metal11 ( 0 0 ) ( 120 20 ) + FIXED ( 250 250 ) W ; "
        "- unused_io + LAYER metal14 ( -1 -1 ) ( 1 1 ) + FIXED ( 410 410 ) N ; "
        "- bottom + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 30 430 ) N ; "
        "- top + LAYER metal14 ( -1 -1 ) ( 1 1 ) + FIXED ( 430 30 ) N ; END PINS "
        "NETS 1 ; - n ( PIN bottom ) ( PIN top ) ; END NETS END DESIGN";
    DefParser def;require(def.parse(write(dir/"sources.def",design),lef,DefGridMapping::OFFICIAL_FULL),"parse mask source placement");
    RoutingGraph graph(def,lef,graph_options());
    require(graph.official_obstacle_demand(graph.node(0,0,10))==0,"tiny unconnected PIN covers zero TRACKS");
    require(graph.tsv_obstacle_sources(0,0,TsvObstacleScope::INTERFACE)==TSV_MACRO_PIN,
        "unconnected PIN blocks whole GCell despite zero fixed-track demand and distance from its center");
    require(graph.tsv_obstacle_layers(1,0,TSV_MACRO_PIN)==(uint32_t(1)<<1),"power PIN remains on its actual layer");
    require(graph.tsv_landing_legal(1,0,TsvObstacleScope::INTERFACE) &&
        !graph.tsv_landing_legal(1,0,TsvObstacleScope::ALL_LAYERS),"scope assumption is explicit");
    require(graph.tsv_obstacle_layers(2,0,TSV_MACRO_PIN)==(uint32_t(1)<<2) &&
        graph.tsv_obstacle_layers(3,0,TSV_MACRO_PIN)==(uint32_t(1)<<10),"each multilayer PORT rectangle retains its layer");
    require(graph.tsv_obstacle_sources(0,1,TsvObstacleScope::INTERFACE)==TSV_OBS,"zero-track OBS blocks whole GCell");
    require(graph.tsv_obstacle_sources(1,2,TsvObstacleScope::INTERFACE)==TSV_OBS &&
        graph.tsv_obstacle_layers(2,2,TSV_OBS)==0 && graph.tsv_obstacle_layers(0,2,TSV_OBS)==0,
        "boundary-aligned rectangle uses positive-area GCell coverage");
    require(graph.tsv_obstacle_sources(2,2,TsvObstacleScope::INTERFACE)==TSV_IO_PIN &&
        graph.tsv_obstacle_sources(2,3,TsvObstacleScope::INTERFACE)==TSV_IO_PIN,
        "rotated IO rectangle blocks every covered GCell");
    require(graph.tsv_obstacle_sources(4,4,TsvObstacleScope::ALL_LAYERS)==TSV_IO_PIN,"unconnected IO is included");
    const auto& stats=graph.tsv_obstacle_stats();
    require(stats.shape_counts==std::array<uint64_t,3>{4,2,4},"all source rectangles counted without NETS filtering");
    require(stats.interface_blocked_cells==6 && stats.all_layer_blocked_cells==11,"independent XY masks retain both scope supplies");
    auto interface_options=policy_options(TsvObstacleScope::INTERFACE),all_options=policy_options(TsvObstacleScope::ALL_LAYERS);
    TsvPolicy interface_policy(def,lef,graph,interface_options),all_policy(def,lef,graph,all_options);
    require(interface_policy.slots().size()==19 && all_policy.slots().size()==14,"hard exclusion is applied before candidate allocation");
    require(interface_policy.obstacle_rejected_cells()==6 && all_policy.obstacle_rejected_cells()==11,"candidate supply rejection counters");
    for(const auto& slot:all_policy.slots()) {
        require(all_policy.compatible(def.nets().front(),slot.id),"legal candidate remains compatible");
        require(graph.tsv_landing_legal(graph.col(slot.lower_node),graph.row(slot.lower_node),TsvObstacleScope::ALL_LAYERS),
            "no blocked candidate can reach any matching/movement index");
        std::string error;require(all_policy.validate_mapping(def.nets().front(),witness(slot),slot.id,error),"legal output mapping: "+error);
    }
    require(graph.tsv_landing_legal(0,0,TsvObstacleScope::DISABLED),"historical fixture mode is explicitly isolated");
    require(!graph.tsv_landing_legal(-1,0,TsvObstacleScope::ALL_LAYERS),"outside grid is never a legal landing");
}
void orientations_and_origin(const fs::path& dir) {
    LefParser lef;require(lef.parse(write(dir/"oriented.lef",technology()+
        "MACRO oriented CLASS BLOCK ; ORIGIN -1 -2 ; SIZE 10 BY 20 ; "
        "PIN hidden PORT LAYER metal10 ; RECT 1 2 3 4 ; END END hidden END oriented ")),
        "parse nonzero LEF ORIGIN");
    // Expected rectangles follow the independently documented DEF orientation
    // transforms, rather than querying transform_macro_rect to build the oracle.
    struct Expected {const char* orientation;int col,row;};
    const Expected expected[]={{"N",5,10},{"S",9,19},{"W",14,10},{"E",5,14},
        {"FN",9,10},{"FS",5,19},{"FW",5,10},{"FE",14,14}};
    for(const auto& e:expected) {
        const std::string design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
            "GCELLGRID X 0 DO 51 STEP 20 ; GCELLGRID Y 0 DO 51 STEP 20 ; "+tracks()+
            "COMPONENTS 1 ; - u oriented + PLACED ( 100 200 ) "+e.orientation+
            " ; END COMPONENTS PINS 1 ; - dummy + LAYER metal1 ( 0 0 ) ( 1 1 ) + FIXED ( 900 900 ) N ; END PINS NETS 1 ; - dummy_net ( PIN dummy ) ; END NETS END DESIGN";
        DefParser def;require(def.parse(write(dir/(std::string(e.orientation)+".def"),design),lef,DefGridMapping::OFFICIAL_FULL),"parse orientation");
        RoutingGraph graph(def,lef,graph_options());
        require(graph.tsv_obstacle_layers(e.col,e.row,TSV_MACRO_PIN)==(uint32_t(1)<<9),
            std::string("PIN orientation and LEF ORIGIN: ")+e.orientation);
        require(graph.tsv_obstacle_stats().interface_blocked_cells==1,"aligned transformed rectangle does not block touching neighbors");
    }
}
void infeasible_fixed_and_supply(const fs::path& dir) {
    LefParser lef;require(lef.parse(write(dir/"fixed.lef",technology())),"parse fixed-crossing technology");
    const auto prefix="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 200 200 ) ; "
        "GCELLGRID X 0 DO 3 STEP 100 ; GCELLGRID Y 0 DO 3 STEP 100 ; "+tracks()+
        "COMPONENTS 0 ; END COMPONENTS ";
    const std::string fixed=prefix+
        "PINS 2 ; - a + LAYER metal10 ( -1 -1 ) ( 1 1 ) + FIXED ( 150 150 ) N ; "
        "- b + LAYER metal11 ( -1 -1 ) ( 1 1 ) + FIXED ( 150 150 ) N ; END PINS "
        "NETS 1 ; - fixed ( PIN a ) ( PIN b ) ; END NETS END DESIGN";
    DefParser def;require(def.parse(write(dir/"fixed.def",fixed),lef,DefGridMapping::OFFICIAL_FULL),"parse fixed PIN crossing");
    RoutingGraph graph(def,lef,graph_options());auto options=policy_options(TsvObstacleScope::ALL_LAYERS);
    rejects_with([&]{TsvPolicy invalid(def,lef,graph,options);},"A150_FIXED_CROSSING_FORBIDDEN");
    options.supply_audit_only=true;TsvPolicy audit(def,lef,graph,options);ThermalParser thermal;
    require(audit.slots().size()==3,"readback audit still builds the legal candidate index for infeasible fixed input");
    rejects_with([&]{audit.assign(def.nets(),thermal);},"audit-only TSV policy");
    std::string dense=prefix+
        "PINS 2 ; - a + LAYER metal3 ( 0 0 ) ( 200 200 ) + FIXED ( 0 0 ) N ; "
        "- b + LAYER metal14 ( -1 -1 ) ( 1 1 ) + FIXED ( 50 50 ) N ; END PINS "
        "NETS 1 ; - dense ( PIN a ) ( PIN b ) ; END NETS END DESIGN";
    DefParser dense_def;require(dense_def.parse(write(dir/"dense.def",dense),lef,DefGridMapping::OFFICIAL_FULL),"parse zero-supply fixture");
    RoutingGraph dense_graph(dense_def,lef,graph_options());options.supply_audit_only=false;
    rejects_with([&]{TsvPolicy invalid(dense_def,lef,dense_graph,options);},"A150_legal_cells=0");
    options.supply_audit_only=true;TsvPolicy dense_audit(dense_def,lef,dense_graph,options);
    require(dense_audit.slots().empty() && dense_audit.obstacle_rejected_cells()==4,"zero supply is reviewable without routing or fallback");
}
}
int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("usage: a150_tsv_obstacle_tests artifact-directory");
        const fs::path dir=argv[1];fs::create_directories(dir);
        geometry_and_sources(dir);orientations_and_origin(dir);infeasible_fixed_and_supply(dir);
        std::cout<<"A150 geometric TSV obstacle checks: "<<checks<<'\n';return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
