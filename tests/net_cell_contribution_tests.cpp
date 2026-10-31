#include "objectives/net_cell_contribution.hpp"
#include "objectives/official_metrics.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks{};
void require(bool condition,const std::string& message) {
    ++checks;if(!condition)throw std::runtime_error(message);
}
template<class Action> void rejects(Action action,const std::string& message) {
    bool rejected=false;try {action();}catch(const std::exception&){rejected=true;}
    require(rejected,message);
}
bool close(long double a,long double b) {return std::abs(a-b)<1e-14L;}
std::string write(const fs::path& path,const std::string& content) {
    std::ofstream stream(path);stream<<content;stream.close();
    if(!stream)throw std::runtime_error("cannot write contribution fixture");
    return path.string();
}
struct Fixture {
    LefParser lef;DefParser def;
    Fixture(const fs::path& directory) {
        fs::create_directories(directory);
        std::ostringstream technology,design;
        design<<"UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 300 300 ) ; "
              <<"GCELLGRID X 0 DO 4 STEP 100 ; GCELLGRID Y 0 DO 4 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            technology<<"LAYER "<<name<<" TYPE ROUTING ; DIRECTION "
                      <<(layer%2?"HORIZONTAL":"VERTICAL")<<" ; WIDTH 0.2 ; END "<<name<<'\n';
            design<<"TRACKS X 3 DO 100 STEP 10 LAYER "<<name<<" ; "
                  <<"TRACKS Y 3 DO 100 STEP 20 LAYER "<<name<<" ; ";
        }
        design<<"COMPONENTS 0 ; END COMPONENTS PINS 2 ; "
              <<"- a + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 50 50 ) N ; "
              <<"- b + LAYER metal14 ( -1 -1 ) ( 1 1 ) + FIXED ( 250 250 ) N ; "
              <<"END PINS NETS 1 ; - n ( PIN a ) ( PIN b ) ; END NETS END DESIGN";
        require(lef.parse(write(directory/"tech.lef",technology.str())),"parse contribution technology");
        require(def.parse(write(directory/"design.def",design.str()),lef,DefGridMapping::OFFICIAL_FULL),
                "parse contribution design");
    }
};
RoutingGraphOptions graph_options() {
    RoutingGraphOptions options;options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;
    options.score_20261020=true;options.documented_rules=true;return options;
}
void landing_and_measurement(const Fixture& fixture,const RoutingGraph& graph) {
    const int bottom=graph.node(1,1,10),top=graph.node(1,1,11);
    const Point point{150,150};
    const NetCellContribution expected{{bottom,5},{top,2}};
    require(mandatory_tsv_contribution(graph,point,1,true)==expected,
            "shared mandatory reservation follows each physical preferred track axis");
    require(mandatory_tsv_contribution(graph,point,0,true)==NetCellContribution{{bottom,1},{top,1}},
            "zero ratio retains the required landing Guide");
    require(mandatory_tsv_contribution(graph,point,1,false)==NetCellContribution{{bottom,2},{top,2}},
            "legacy mode keeps Guide plus constant TSV demand");
    const auto& net=fixture.def.nets().front();
    const auto pins=graph.own_pin_nodes(net);
    require(mandatory_tsv_contribution(graph,point,1,true,pins)==expected,
            "remote own PIN does not exempt the landing");
    const auto cell=graph.grid().get_cell_rect(1,1);
    const std::vector<GuideRect> guides{{cell.x1,cell.y1,cell.x2,cell.y2,"metal10"},
                                         {cell.x1,cell.y1,cell.x2,cell.y2,"metal11"}};
    OfficialMetricOptions options;options.score_20261020=true;options.tsv_track_demand=true;
    options.gcell_width_dbu=100;options.gcell_height_dbu=100;
    const auto measured=measure_official_net_guides(net,guides,graph,fixture.lef,options,1000);
    require(measured.demand==expected,"full encoded measurement and search reservation have identical roots");
    auto with_pin=net;Pin pin;pin.name="own";pin.layer="metal10";pin.layer_idx=10;
    pin.coord=point;pin.port_rects={{10,{145,145,155,155}}};with_pin.pins.push_back(pin);
    const auto own=graph.own_pin_nodes(with_pin);
    require(mandatory_tsv_contribution(graph,point,1,true,own)==NetCellContribution{{top,2}},
            "only the true own PIN layer is exempt");
    const auto exempt=measure_official_net_guides(with_pin,guides,graph,fixture.lef,options,1000);
    require(exempt.demand==mandatory_tsv_contribution(graph,point,1,true,own),
            "encoded measurement applies the same PIN exemption as the reservation");
    require(mandatory_tsv_contribution(graph,point,1,true,own,true)==expected,
            "A150 TSV reservation ignores own PIN credit; placement legality is checked separately");
    auto a150=options;a150.a150_rules=true;
    require(measure_official_net_guides(with_pin,guides,graph,fixture.lef,a150,1000).demand==expected,
            "A150 measurement never removes TSV demand at an own PIN node");
    require(merge_net_cell_contribution({{bottom,1},{top,1}},{{bottom,0},{top,2}},{},true)==
                NetCellContribution{{top,2}},"zero-track A150 marker replaces rather than retains ordinary Guide demand");
    require(merge_net_cell_contribution({{top,1},{bottom,1}},{{top,2},{bottom,0}},{},true)==
                NetCellContribution{{top,2}},"unsorted A150 merge agrees with canonical zero-track replacement");
    require(merge_net_cell_contribution({{bottom,1},{top,1},{top+1,1}},expected,{bottom,top+1},true)==expected,
            "A150 TSV is never PIN-exempt while ordinary Guide PIN credit remains separate");
    rejects([&]{mandatory_tsv_contribution(graph,point,0,true,{},true);},
            "A150 direct track demand rejects a zero diagnostic multiplier");
    rejects([&]{mandatory_tsv_contribution(graph,point,1,false,{},true);},
            "A150 cannot silently use the historical constant-demand mode");
    const NetCellContribution ordinary{{bottom,1},{bottom,1},{top,1},{top+1,1}};
    require(merge_net_cell_contribution(ordinary,expected)==
                NetCellContribution{{bottom,5},{top,2},{top+1,1}},
            "same-net overlap and repeated Guide coverage are max unions");
    require(merge_net_cell_contribution(ordinary,expected,{bottom})==
                NetCellContribution{{top,2},{top+1,1}},"PIN mask applies to all same-net components");
    rejects([&]{mandatory_tsv_contribution(graph,point,-1,true);},"negative TSV ratio rejected");
    rejects([&]{mandatory_tsv_contribution(graph,point,std::numeric_limits<int>::max(),true);},
            "physical reservation overflow rejected before ledger mutation");
    rejects([&]{mandatory_tsv_contribution(graph,{-1,150},1,true);},"outside landing rejected");
    rejects([&]{merge_net_cell_contribution({{bottom,0}},{});},"zero demand sparse entry rejected");
}
void grouped_nonlinear_delta(const RoutingGraph& graph) {
    const int node=graph.node(1,1,10),other=graph.node(1,1,11);
    require(graph.official_capacity(node)==10,"nonlinear fixture capacity is ten physical tracks");
    require(graph.official_obstacle_demand(node)==0,"nonlinear fixture landing has no fixed demand");
    std::vector<int> aggregate(graph.size());aggregate[node]=12;aggregate[other]=1;
    const auto original=aggregate;
    const std::vector<NetCellContribution> old_nets{{{node,6},{other,1}},{{node,6}}};
    const std::vector<NetCellContribution> new_nets{{{node,2},{other,1}},{{node,3}}};
    const auto group=compare_net_cell_replacement(graph,aggregate,old_nets,new_nets);
    require(group.demand_delta==NetCellContribution{{node,-7}},
            "different owners add separate contributions to the same cell");
    require(close(group.squared_delta,-0.0004L),"group Q uses combined nonlinear change once");
    const auto first=compare_net_cell_replacement(graph,aggregate,{old_nets[0]},{new_nets[0]});
    const auto second=compare_net_cell_replacement(graph,aggregate,{old_nets[1]},{new_nets[1]});
    require(!close(first.squared_delta+second.squared_delta,group.squared_delta),
            "independent stale-ledger move deltas are not a group objective");
    require(aggregate==original,"group quotation never mutates the actual demand ledger");
    long double before=0;
    for(size_t id=0;id<aggregate.size();++id)
        before+=official_resource_cell_contribution(graph,int(id),aggregate[id]).squared;
    for(auto [id,change]:group.demand_delta)aggregate[id]+=change;
    require(aggregate[node]==5,"committed cross-net demand uses sum rather than same-net max");
    long double after=0;
    for(size_t id=0;id<aggregate.size();++id)
        after+=official_resource_cell_contribution(graph,int(id),aggregate[id]).squared;
    require(close(after-before,group.squared_delta),"local group delta equals full graph recomputation");
    const auto undo=compare_net_cell_replacement(graph,aggregate,new_nets,old_nets);
    require(close(group.squared_delta+undo.squared_delta,0),"reverse transaction restores Q");
    require(group.overflow_delta+undo.overflow_delta==0&&group.zero_capacity_delta+undo.zero_capacity_delta==0,
            "reverse transaction restores overflow and zero-capacity totals");
    for(auto [id,change]:undo.demand_delta)aggregate[id]+=change;
    require(aggregate==original,"reverse transaction restores every cell demand");
    rejects([&]{compare_net_cell_replacement(graph,aggregate,{{{node,13}}},{});},
            "group cannot remove more demand than the actual ledger");
    rejects([&]{compare_net_cell_replacement(graph,aggregate,{{{node,2},{node,3}}},{});},
            "group rejects noncanonical duplicated owner contribution");
    require(aggregate==original,"rejected group leaves all cell demands intact");
    const auto shared=official_resource_cell_contribution(graph,node,12);
    require(shared.overflow==2&&close(shared.squared,0.0004L)&&!shared.zero_capacity,
            "shared cell contract preserves documented normalized overflow");
}
}
int main(int argc,char** argv) {
    try {
        if(argc!=2)throw std::runtime_error("usage: net_cell_contribution_tests artifact-directory");
        Fixture fixture(fs::path(argv[1])/"contribution");
        RoutingGraph graph(fixture.def,fixture.lef,graph_options());
        landing_and_measurement(fixture,graph);grouped_nonlinear_delta(graph);
        Net local;local.name="local";
        const std::vector<GuideRect> branches{{100,0,200,300,"metal8"},{0,0,300,100,"metal9"},{0,200,300,300,"metal9"}};
        OfficialMetricOptions old;old.score_20261020=true;old.gcell_width_dbu=100;old.gcell_height_dbu=100;
        auto full=old;full.all_adjacent_overlaps=true;
        const auto first=measure_official_net_guides(local,branches,graph,fixture.lef,old,1000);
        const auto all=measure_official_net_guides(local,branches,graph,fixture.lef,full,1000);
        require(first.ordinary_connections==1&&all.ordinary_connections==2,"all real positive-area adjacent connections are charged");
        require(close(all.wirelength-first.wirelength,300),"second physical connection adds its full via price");
        auto reordered=branches;std::reverse(reordered.begin(),reordered.end());
        require(close(measure_official_net_guides(local,reordered,graph,fixture.lef,full,1000).wirelength,all.wirelength),
                "connection charging does not depend on rectangle order");
        auto scaled_options=graph_options();scaled_options.congestion_overflow_scale=1;
        RoutingGraph scaled(fixture.def,fixture.lef,scaled_options);
        const int cell=scaled.node(1,1,10);
        require(close(official_resource_cell_contribution(scaled,cell,12).squared,0.04L),
                "observed positive-capacity penalty is not attenuated by 0.1");
        std::vector<int> d(scaled.size());d[cell]=12;
        auto delta=compare_net_cell_replacement(scaled,d,{{{cell,12}}},{{{cell,5}}});
        require(close(delta.squared_delta,-0.04L),"new scale enters grouped transaction delta");
        require(delta.overflow_delta==-2,"penalty scale does not change physical overflow");
        require(close(congestion_cell_delta(12,1,10,1),0.05L),"search one-unit delta uses same penalty");
        require(close(congestion_cell_delta(12,1,10,0.1),0.0005L),"old search scale stays available");
        require(close(congestion_cell_penalty(1,0,1),10000),"zero-capacity fallback remains explicit conservative policy");
        rejects([]{congestion_cell_penalty(-1,10,1);},"negative demand is rejected");
        rejects([]{congestion_cell_penalty(1,10,0);},"zero penalty scale is rejected");
        std::cout<<"Net-cell contribution checks: "<<checks<<'\n';return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
