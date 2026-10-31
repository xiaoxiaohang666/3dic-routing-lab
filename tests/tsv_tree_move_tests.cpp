#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool ok,const std::string& why) {++checks;if(!ok) throw std::runtime_error("check "+std::to_string(checks)+": "+why);}
void near(double a,double b,const std::string& why) {
    require(std::abs(a-b)<=1e-10*std::max({1.0,std::abs(a),std::abs(b)}),why);
}
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
std::string access_key(const PinAccess& access) {
    std::ostringstream out;out<<std::setprecision(17)<<access.node<<':'<<access.length<<':'<<access.vias;
    for(const auto& g:access.guides) out<<':'<<g.layer<<','<<g.x1<<','<<g.y1<<','<<g.x2<<','<<g.y2;
    return out.str();
}
std::string route_key(const NetRoute& route) {
    std::ostringstream out;out<<std::setprecision(17)<<route.slot<<':'<<route.length<<':'<<route.vias;
    for(int node:route.nodes) out<<'n'<<node;
    for(auto edge:route.edges) out<<'e'<<edge;
    for(const auto& access:route.accesses) out<<'a'<<access_key(access);
    for(const auto& g:route.guides) out<<'g'<<g.layer<<','<<g.x1<<','<<g.y1<<','<<g.x2<<','<<g.y2;
    for(auto [node,amount]:route.measured.demand) out<<'d'<<node<<','<<amount;
    return out.str();
}
}
namespace eda {
struct TsvTreeMoveTestAccess {
    static int slot_at(const NegotiatedRouter& r,int col,int row) {
        for(size_t k=0;k<r.policy_.slots().size();++k) {
            const auto& slot=r.policy_.slots()[k];
            if(r.graph_.col(slot.lower_node)==col && r.graph_.row(slot.lower_node)==row) return int(k);
        }
        return -1;
    }
    static NetRoute install(NegotiatedRouter& r) {
        NetRoute old;old.net_index=0;old.slot=slot_at(r,0,2);require(old.slot>=0,"old TSV cell is a legal empty cell");
        const auto edge=[&](int a,int b) {
            require(r.graph_.legal_edge(a,b),"fixture obeys preferred directions and adjacent-layer vias");
            old.nodes.insert(a);old.nodes.insert(b);old.edges.insert(route_edge(a,b));
        };
        for(int layer:{9,11}) for(int col=0;col<6;++col)
            edge(r.graph_.node(col,2,layer),r.graph_.node(col+1,2,layer));
        for(int layer=3;layer<9;++layer) edge(r.graph_.node(3,2,layer),r.graph_.node(3,2,layer+1));
        for(int layer=11;layer<13;++layer) edge(r.graph_.node(3,2,layer),r.graph_.node(3,2,layer+1));
        for(const auto& pin:r.def_.nets()[0].pins) {
            const bool top=pin.layer_idx>10;const auto choices=r.accesses(pin,top?11:1,top?20:9);
            require(choices.size()==1,"fixture real PIN has a direct native access");
            old.accesses.push_back(choices.front());old.nodes.insert(choices.front().node);
        }
        const auto& slot=r.policy_.slots()[size_t(old.slot)];
        for(int side=0;side<2;++side) {
            const auto choices=r.accesses(slot.point,side?11:10,side?11:1,side?20:9);
            old.accesses.push_back(choices.front());old.nodes.insert(choices.front().node);
        }
        r.encode(0,old);r.routes_.resize(1);r.routes_[0]=old;r.account(0,old,1);
        r.initial_build_=false;r.complete_=true;r.best_metrics_=r.measure();exact(r);
        return old;
    }
    static void exact(NegotiatedRouter& r) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<int> demand(r.graph_.size());GuideMap guides;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto fresh=measure_official_net_guides(r.def_.nets()[i],r.routes_[i].guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]);
            require(fresh.demand==r.routes_[i].measured.demand,"full emitted Guide demand matches route cache");
            near(fresh.wirelength,r.routes_[i].measured.wirelength,"full emitted Guide fee includes every ordinary via");
            require(fresh.tsv_cells==r.routes_[i].measured.tsv_cells,"serialized TSV cell matches route cache");
            for(auto [node,amount]:fresh.demand) demand[size_t(node)]+=amount;
            guides[r.def_.nets()[i].name]=r.routes_[i].guides;
        }
        require(demand==r.demand_,"full emitted Guide demand matches live ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"actual real PIN geometry remains connected");
        const auto measured=r.measure();r.verify_official_ledger(measured);
        near(measured.guide.quality,r.best_metrics_.guide.quality,"complete and incremental resource/length/thermal score agree");
    }
    static void run(NegotiatedRouter& r) {
        const auto old=install(r);const int target=slot_at(r,8,2);require(target>=0,"new TSV cell is legal and empty");
        const auto fixed_floor=r.graph_.official_pin_zero_track_floor_total();
        require(fixed_floor==5,"four connected plus one unconnected tiny PIN each retain a fixed floor unit");
        require(!r.graph_.tsv_landing_legal(3,2,TsvObstacleScope::ALL_LAYERS),"real multilayer PIN cell is forbidden");
        require(!r.graph_.tsv_landing_legal(10,5,TsvObstacleScope::ALL_LAYERS),"unconnected tiny zero-track PIN cell is forbidden");
        // The fixture includes one more unconnected IO PIN floor unit.
        const auto outer_deadline=r.deadline_;const auto outer_limit=r.options_.search_limit;
        const auto outer_refinement=r.v7_refinement_active_;const auto outer_trial=r.v5_topology_trial_;
        r.options_.tsv_tree_move_seconds=1;r.options_.tsv_tree_move_expansions=20000;
        r.account(0,old,-1);
        const auto residual=r.demand_;const auto owners=r.cell_nets_;const auto edge_owners=r.edge_nets_;const auto usage=r.usage_;
        const auto incumbent=route_key(r.routes_[0]);
        const auto nonempty=[](auto values) {
            for(auto it=values.begin();it!=values.end();) {
                if(it->second.empty()) it=values.erase(it);else ++it;
            }
            return values;
        };
        const auto positive=[](auto values) {
            for(auto it=values.begin();it!=values.end();) {
                if(it->second==0) it=values.erase(it);else ++it;
            }
            return values;
        };
        const auto unchanged=[&] {
            // account() retains zero-valued sparse storage after a separately
            // installed/rolled-back route; ownership contents must still match.
            require(r.demand_==residual && nonempty(r.cell_nets_)==nonempty(owners) &&
                    nonempty(r.edge_nets_)==nonempty(edge_owners) && positive(r.usage_)==positive(usage),
                    "candidate construction leaves resource ledger and reverse ownership unchanged");
            require(route_key(r.routes_[0])==incumbent,"candidate construction leaves incumbent route and slot unchanged");
            require(r.deadline_==outer_deadline && r.options_.search_limit==outer_limit &&
                    r.v7_refinement_active_==outer_refinement && r.v5_topology_trial_==outer_trial,
                    "candidate construction restores outer search deadline/options");
            require(r.graph_.official_pin_zero_track_floor_total()==fixed_floor,"candidate never removes fixed PIN floor demand");
        };
        const auto trial=r.build_tree_tsv_move(0,target,old);require(trial.has_value(),"legal tree-preserving TSV migration constructed");unchanged();
        require(trial->slot==target && trial->measured.tsv_cells.size()==1 && trial->measured.tsv_cells!=old.measured.tsv_cells,
                "single physical TSV moves to the requested cell");
        for(size_t k=0;k<r.def_.nets()[0].pins.size();++k)
            require(access_key(old.accesses[k])==access_key(trial->accesses[k]),"every original real PIN access is retained byte-for-byte");
        for(int layer:{9,11}) {
            for(int col=0;col<3;++col) require(!trial->nodes.count(r.graph_.node(col,2,layer)),"old TSV-only planar leaf stem removed");
            for(int col=3;col<6;++col) require(trial->edges.count(route_edge(r.graph_.node(col,2,layer),r.graph_.node(col+1,2,layer))),
                                               "shared branch and original PIN trunk retained");
        }
        for(auto edge:old.edges) {
            const auto [a,b]=edge_nodes(edge);
            if(r.graph_.layer(a)!=r.graph_.layer(b)) require(trial->edges.count(edge),"existing shared via column retained with all intermediate metals");
        }
        const int old_interface=r.graph_.node(0,2,10);
        require(std::none_of(trial->measured.demand.begin(),trial->measured.demand.end(),
                            [&](const auto& entry){return entry.first==old_interface;}),"old M10 TSV witness and its fee are removed");
        for(int layer:{10,11}) {
            const int node=r.graph_.node(8,2,layer);const int tracks=r.graph_.tsv_landing_tracks(r.policy_.slots()[size_t(target)].point,layer);
            const auto entry=std::find_if(trial->measured.demand.begin(),trial->measured.demand.end(),[&](const auto& e){return e.first==node;});
            require(tracks>1 && entry!=trial->measured.demand.end() && entry->second==tracks,
                    "new TSV billed by actual tracks without an extra ordinary Guide unit");
        }
        r.routes_[0]=*trial;r.account(0,*trial,1);r.best_metrics_=r.measure();exact(r);
        r.account(0,*trial,-1);r.routes_[0]=old;r.account(0,old,1);r.best_metrics_=r.measure();exact(r);
        r.account(0,old,-1);unchanged();

        auto explicit_old=old;explicit_old.explicit_geometry=true;
        require(!r.build_tree_tsv_move(0,target,explicit_old),"explicit geometry uses whole-net fallback");unchanged();
        auto missing=old;missing.accesses.pop_back();
        require(!r.build_tree_tsv_move(0,target,missing),"missing TSV access is not guessed from coordinates");unchanged();
        auto duplicate=old;duplicate.accesses.push_back(old.accesses.back());
        require(!r.build_tree_tsv_move(0,target,duplicate),"ambiguous TSV access is rejected rather than deleting a possible real access");unchanged();
        require(!r.build_tree_tsv_move(0,-1,old) && !r.build_tree_tsv_move(0,old.slot,old),"invalid/current landing starts no trial");unchanged();
        r.options_.tsv_tree_move_expansions=0;
        require(!r.build_tree_tsv_move(0,target,old),"exhausted expansion budget rejects without mutation");unchanged();
        r.options_.tsv_tree_move_expansions=20000;r.options_.tsv_tree_move_seconds=0;
        require(!r.build_tree_tsv_move(0,target,old),"exhausted local time budget rejects without mutation");unchanged();
        r.options_.tsv_tree_move_seconds=1;
        r.deadline_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
        require(!r.build_tree_tsv_move(0,target,old),"expired global deadline starts no trial");
        r.deadline_=outer_deadline;unchanged();
        r.routes_.push_back(NetRoute{});r.routes_.back().slot=target;
        const int occupied_node=r.policy_.slots()[size_t(target)].upper_node;
        r.cell_nets_[occupied_node].push_back(1);
        require(!r.build_tree_tsv_move(0,target,old),"occupied landing is not reassigned by a one-net helper");
        r.cell_nets_.erase(occupied_node);r.routes_.pop_back();unchanged();
        r.account(0,old,1);r.best_metrics_=r.measure();exact(r);
    }
    static void dynamic(NegotiatedRouter& r) {
        install(r);
        r.options_.tsv_tree_move=true;r.options_.v8_tsv_dynamic=true;
        r.options_.tsv_rounds=1;r.options_.v7_tsv_nets=1;r.options_.v7_tsv_candidates=16;
        r.options_.v7_tsv_seconds=10;r.options_.tsv_tree_move_seconds=0.1;
        r.options_.tsv_tree_move_expansions=20000;
        r.optimize_dynamic_tsv();exact(r);
        require(r.counters_.tsv_tree_move_attempted>0,"dynamic loop executes optional tree construction");
        require(r.counters_.tsv_tree_move_built>0,"dynamic loop builds valid tree candidates");
        // Force both candidate families to fail before installation; the whole
        // phase must preserve the accepted route, Pearson occupancy and ledger.
        const auto geometry=route_key(r.routes_[0]);const auto demand=r.demand_;
        const auto before=r.measure();r.options_.search_limit=0;
        r.optimize_dynamic_tsv();exact(r);
        require(route_key(r.routes_[0])==geometry && r.demand_==demand,"failed dynamic candidates restore complete state");
        near(r.measure().guide.correlation,before.guide.correlation,"failed dynamic candidates preserve Pearson occupancy");
    }
    static void forbidden_scope(NegotiatedRouter& r) {
        const auto old=install(r);const int pin_slot=slot_at(r,3,2);
        require(pin_slot>=0 && r.policy_.compatible(r.def_.nets()[0],pin_slot),"interface-only fixture exposes a noninterface PIN cell");
        r.account(0,old,-1);const auto demand=r.demand_;const auto route=route_key(r.routes_[0]);
        require(!r.build_tree_tsv_move(0,pin_slot,old),"helper independently rejects all-layer PIN geometry at target");
        require(r.demand_==demand && route_key(r.routes_[0])==route,"forbidden target rejection preserves incumbent and residual ledger");
        r.account(0,old,1);r.best_metrics_=r.measure();exact(r);
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("tsv_tree_move_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path dir=argv[1];fs::create_directories(dir);
        std::string tech,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1200 800 ) ; GCELLGRID X 0 DO 13 STEP 100 ; GCELLGRID Y 0 DO 9 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            design+="TRACKS X 5 DO 120 STEP 10 LAYER "+name+" ; TRACKS Y 5 DO 80 STEP 10 LAYER "+name+" ; ";
        }
        design+="COMPONENTS 0 ; END COMPONENTS PINS 5 ; ";
        for(const auto& [name,layer,x,y]:std::vector<std::tuple<std::string,int,int,int>>{
                {"b0",9,650,250},{"b1",3,350,250},{"t0",11,650,250},{"t1",13,350,250},{"unused",2,1050,550}})
            design+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        design+="END PINS NETS 1 ; - tree ( PIN b0 ) ( PIN b1 ) ( PIN t0 ) ( PIN t1 ) ; END NETS END DESIGN";
        LefParser lef;DefParser def;
        require(lef.parse(write(dir/"fixture.lef",tech)) && def.parse(write(dir/"fixture.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parses");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int row=0;row<8;++row) for(int col=0;col<12;++col) {
            if(row||col) heat<<',';
            heat<<'['<<5+10*col<<','<<5+10*row<<",0,"<<26+col+12*row<<']';
        }
        heat<<"]}";ThermalParser thermal;
        require(thermal.parse_json(write(dir/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"heat parses");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];options.time_limit=120;
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true};
        options.official_metrics.score_20261020=true;options.official_metrics.a150_rules=true;
        options.official_metrics.tsv_track_demand=true;options.official_metrics.tsv_congestion_ratio=1;
        options.official_metrics.gcell_width_dbu=def.gcell_pitch_x();options.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
        options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;options.tsv_policy.tsv_obstacle_scope=TsvObstacleScope::ALL_LAYERS;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;options.v7_cell_cost=true;
        NegotiatedRouter router(def,lef,thermal,options);TsvTreeMoveTestAccess::run(router);
        NegotiatedRouter dynamic_router(def,lef,thermal,options);TsvTreeMoveTestAccess::dynamic(dynamic_router);
        options.tsv_policy.tsv_obstacle_scope=TsvObstacleScope::INTERFACE;
        NegotiatedRouter narrower(def,lef,thermal,options);TsvTreeMoveTestAccess::forbidden_scope(narrower);
        std::cout<<"tsv_tree_move_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
