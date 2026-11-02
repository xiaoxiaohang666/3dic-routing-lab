#include "router/shared_hub_optimizer.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
using namespace eda;
namespace fs=std::filesystem;
namespace {
int checks=0;
void require(bool b,const std::string& why) {++checks;if(!b) throw std::runtime_error(why);}
std::string save(const fs::path& p,const std::string& text) {std::ofstream out(p);out<<text;out.close();if(!out) throw std::runtime_error("fixture write");return p.string();}
std::string geometry(const GuideMap& g) {
    std::ostringstream out;for(const auto& [name,rects]:g) {out<<name;for(const auto& r:rects) out<<r.layer<<r.x1<<','<<r.y1<<','<<r.x2<<','<<r.y2<<';';}return out.str();
}
std::vector<GuideRect> tree(int c,int r) {
    return {{200,200,double((c+1)*100),300,"metal9"},{double(c*100),200,double((c+1)*100),double((r+1)*100),"metal10"},
        {200,double(r*100),double((c+1)*100),double((r+1)*100),"metal11"},{200,300,300,double((r+1)*100),"metal12"}};
}
}
int main(int argc,char** argv) {
 try {
    if(argc!=2) throw std::runtime_error("artifact directory required");const fs::path dir=argv[1];fs::create_directories(dir);
    std::ostringstream tech;for(int l=1;l<=20;++l) tech<<"LAYER metal"<<l<<" TYPE ROUTING ; DIRECTION "<<(l%2?"HORIZONTAL":"VERTICAL")<<" ; WIDTH 0.2 ; END metal"<<l<<"\n";
    LefParser lef;require(lef.parse(save(dir/"fixture.lef",tech.str())),"parse LEF");
    std::ostringstream design;design<<"UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1200 1200 ) ; GCELLGRID X 0 DO 13 STEP 100 ; GCELLGRID Y 0 DO 13 STEP 100 ; ";
    for(int l=1;l<=20;++l) design<<"TRACKS X 5 DO 120 STEP 10 LAYER metal"<<l<<" ; TRACKS Y 5 DO 120 STEP 10 LAYER metal"<<l<<" ; ";
    design<<"COMPONENTS 0 ; END COMPONENTS PINS 8 ; ";
    for(int n=0;n<4;++n) for(int side=0;side<2;++side) design<<"- p"<<n<<side<<" + LAYER metal"<<(side?12:9)<<" ( -1 -1 ) ( 1 1 ) + FIXED ( 250 "<<(side?350:250)<<" ) N ; ";
    design<<"END PINS NETS 4 ; ";for(int n=0;n<4;++n) design<<"- n"<<n<<" ( PIN p"<<n<<"0 ) ( PIN p"<<n<<"1 ) ; ";design<<"END NETS END DESIGN";
    DefParser def;require(def.parse(save(dir/"fixture.def",design.str()),lef,DefGridMapping::OFFICIAL_FULL),"parse DEF");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});ThermalParser thermal;
    std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
    for(int r=0;r<12;++r) for(int c=0;c<12;++c) {if(r||c) heat<<',';heat<<'['<<5+10*c<<','<<5+10*r<<",0,26]";}
    heat<<"]}";require(thermal.parse_json(save(dir/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse uniform positive heat");
    OfficialMetricOptions metric;metric.score_20261020=true;metric.tsv_track_demand=true;metric.a150_rules=true;
    metric.all_adjacent_overlaps=true;metric.snap_reference_coordinates=true;metric.thermal_gain=3;metric.gcell_width_dbu=metric.gcell_height_dbu=100;
    GuideMap original;original["n0"]=tree(8,8);original["n1"]=tree(8,8);original["n2"]=tree(10,10);original["n3"]=tree(10,10);
    const std::vector<double> references(4,100);const auto frozen=geometry(original);
    const int old_cell=8*graph.grid().cols+8;
    require(shared_hub_cell_geometry(tree(8,8),old_cell,graph),"one complete landing passes interface geometry");
    std::vector<GuideRect> edge={{800,800,900,1000,"metal10"},{800,800,900,900,"metal11"},{900,900,1000,1000,"metal11"}};
    require(!shared_hub_cell_geometry(edge,old_cell,graph),"positive TSV plus an external edge touch is rejected");
    std::vector<GuideRect> second={{800,800,900,1000,"metal10"},{800,800,900,1000,"metal11"}};
    require(!shared_hub_cell_geometry(second,old_cell,graph),"multi-cell interface overlap is rejected");
    auto run=[&](const SharedHubOptions& opt,GuideMap& g) {return optimize_shared_hubs(def,lef,graph,thermal,metric,references,g,opt);};
    SharedHubOptions opt;opt.max_groups=1;opt.max_moves=1;opt.seconds=30;
    auto g=original;const auto success=run(opt,g);
    require(success.moves==1,"whole group actually moves");
    require(success.after.L<success.before.L&&success.after.S>success.before.S,"real attachment length and full score improve");
    require(success.groups.size()==2&&success.trials.size()>0,"two frozen groups and bounded candidates recorded");
    require(success.before.rho==0&&success.after.rho==0,"zero-variance heat remains finite");
    require(success.after.ordinary_vias<=success.before.ordinary_vias,"all ordinary vias included");
    size_t changed=0;int new_cell=-1;
    for(size_t i=0;i<def.nets().size();++i) {
        const auto& net=def.nets()[i];const auto a=measure_official_net_guides(net,original.at(net.name),graph,lef,metric,100);
        const auto b=measure_official_net_guides(net,g.at(net.name),graph,lef,metric,100);
        require(b.tsv_connections==1&&b.tsv_cells.size()==1,"one physical TSV per net");
        if(a.tsv_cells!=b.tsv_cells) {++changed;if(new_cell<0) new_cell=b.tsv_cells[0];require(new_cell==b.tsv_cells[0],"all original owners move together");}
        else require(geometry({{net.name,original.at(net.name)}})==geometry({{net.name,g.at(net.name)}}),"unselected groups retain exact geometry");
    }
    require(changed==2,"only one fixed two-owner group changes");
    auto replay=original;const auto repeat=run(opt,replay);require(geometry(g)==geometry(replay)&&repeat.after.S==success.after.S,"deterministic repeat");
    auto control=original;auto disabled=opt;disabled.max_moves=0;require(run(disabled,control).moves==0&&geometry(control)==frozen,"analysis/control cannot mutate input");
    auto timeout=original;auto expired=opt;expired.seconds=0;require(run(expired,timeout).timed_out&&geometry(timeout)==frozen,"expired search preserves complete incumbent");
    auto bounded=original;auto tiny=opt;tiny.search_nodes=1;require(run(tiny,bounded).moves==0&&geometry(bounded)==frozen,"search exhaustion rolls back whole group");
    auto tail=original;auto limited=opt;limited.max_tail_nodes=1;require(run(limited,tail).moves==0&&geometry(tail)==frozen,"tail budget preserves input");
    auto failure=original;auto injected=opt;injected.before_commit=[] {throw std::runtime_error("injected staged failure");};bool threw=false;
    try {run(injected,failure);} catch(const std::runtime_error& e) {threw=std::string(e.what())=="injected staged failure";}
    require(threw&&geometry(failure)==frozen,"exception before atomic commit preserves every owner");
    auto interrupted=original;auto delayed=opt;delayed.seconds=1;bool staged=false;
    delayed.before_commit=[&] {staged=true;std::this_thread::sleep_for(std::chrono::milliseconds(1100));};
    const auto cancelled=run(delayed,interrupted);
    require(staged&&cancelled.timed_out&&geometry(interrupted)==frozen,"deadline after candidate evaluation rolls back every staged owner");
    auto invalid=original;invalid["n0"].push_back({800,800,900,900,"metal1"});bool rejected=false;
    try {run(opt,invalid);} catch(const std::runtime_error&) {rejected=true;}
    require(rejected,"disconnected malformed baseline is rejected");
    // Nonuniform heat exercises exact whole-group Pearson changes and the
    // independent full aggregate assertion, rather than only the zero case.
    std::ostringstream varied;varied<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
    for(int r=0;r<12;++r) for(int c=0;c<12;++c) {if(r||c) varied<<',';varied<<'['<<5+10*c<<','<<5+10*r<<",0,"<<26+c+12*r<<']';}
    varied<<"]}";require(thermal.parse_json(save(dir/"varied.json",varied.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse varied heat");
    metric.thermal_gain=.01;auto nonuniform=original;const auto weighted=run(opt,nonuniform);
    require(weighted.moves==1&&weighted.after.S>weighted.before.S&&weighted.after.rho!=weighted.before.rho,"full-score acceptance includes actual thermal change");
    // Every trial includes a same-position reattachment control. Position
    // gains must survive this comparator, not just the original detoured tree.
    size_t controls=0;for(const auto& t:weighted.trials) if(t.from==t.to&&t.reason=="current-hub-reattachment-control") ++controls;
    require(controls==1,"explicit current-hub control for the selected group");
    // A partial split changes count variance; the optimizer's independent
    // aggregate assertion must still agree with exact count-based Pearson.
    metric.thermal_gain=.0001;
    for(const std::string mode:{"outlier","gap"}) {
        auto hetero=opt;hetero.heterogeneous=mode;auto split=original;const auto result=run(hetero,split);
        require(result.moves==1&&result.after.S>result.before.S,"bounded heterogeneous split improves full fixture score");
        std::map<int,size_t> load;size_t contacts=0;
        for(const auto& net:def.nets()) {
            const auto m=measure_official_net_guides(net,split.at(net.name),graph,lef,metric,100);
            require(m.tsv_connections==1,"split preserves one TSV per real owner");++load[m.tsv_cells.front()];contacts+=m.tsv_connections;
        }
        require(load.size()==3&&contacts==4,"split adds a landing but no extra physical TSV owners");
        auto repeat=original;run(hetero,repeat);require(geometry(split)==geometry(repeat),"heterogeneous repeat deterministic");
        auto failing=original;hetero.before_commit=[] {throw std::runtime_error("heterogeneous failure");};bool aborted=false;
        try {run(hetero,failing);} catch(const std::runtime_error& e) {aborted=std::string(e.what())=="heterogeneous failure";}
        require(aborted&&geometry(failing)==frozen,"partial-owner staged failure preserves all nets");
    }
    auto branches=original;branches["n0"].push_back({700,800,900,900,"metal9"});
    std::ostringstream census;write_shared_hub_diagnostics(census,def,lef,graph,thermal,metric,references,branches);
    require(census.str().find("\"root_degree\":2")!=std::string::npos,"diagnostic retains and analyzes branch-root nets");
    require(census.str().find("\"root_to_nearest_covered_pin_L\":null")==std::string::npos,"every die side receives actual covered root-to-PIN path");
    save(dir/"diagnostic.json",census.str());
    auto multi=opt;multi.heterogeneous="outlier";multi.max_groups=2;multi.max_moves=2;
    auto two=original;const auto two_report=run(multi,two);
    require(two_report.moves==2,"successive partial splits update the shared global Pearson state");
    auto cap=multi;cap.max_groups=9;auto unchanged=original;bool capped=false;
    try {run(cap,unchanged);} catch(const std::invalid_argument&) {capped=true;}
    require(capped&&geometry(unchanged)==frozen,"hard eight-original-group cap checked before mutation");
    std::cout<<"shared_hub_optimizer "<<checks<<" checks passed\n";return 0;
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
