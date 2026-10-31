#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <cmath>
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
    require(std::abs(a-b)<=2e-10*std::max({1.0,std::abs(a),std::abs(b)}),reason);
}
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
}
namespace eda {
struct LowLayerRepairTestAccess {
    static void exact(NegotiatedRouter& r) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<int> demand(r.graph_.size());GuideMap guides;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto fresh=measure_official_net_guides(r.def_.nets()[i],r.routes_[i].guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]);
            require(fresh.demand==r.routes_[i].measured.demand && fresh.wirelength==r.routes_[i].measured.wirelength,
                "serialized complete via and cell charges equal cached measurement");
            for(auto [node,count]:fresh.demand) demand[size_t(node)]+=count;
            guides[r.def_.nets()[i].name]=r.routes_[i].guides;
        }
        require(demand==r.demand_,"serialized resources equal live ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"all real pins remain connected");
        const auto actual=r.measure();r.verify_official_ledger(actual);
        near(actual.guide.quality,r.best_metrics_.guide.quality,"full and incremental quality agree");
    }
    static void forced_chains(NegotiatedRouter& r) {
        require(r.route_all(),"initial routing succeeds");
        for(size_t index=0;index<r.routes_.size();++index) {
            const auto& net=r.def_.nets()[index];if(net.name=="zz_cross") continue;
            const int layer=net.name.front()=='b'?1:11;
            r.account(index,r.routes_[index],-1);NetRoute route;route.net_index=index;
            for(int c=0;c<20;++c) {
                const int n=r.graph_.node(c,2,layer);route.nodes.insert(n);
                if(c) route.edges.insert(route_edge(r.graph_.node(c-1,2,layer),n));
            }
            for(int col:{0,19}) {PinAccess access;access.node=r.graph_.node(col,2,layer);route.accesses.push_back(access);}
            r.encode(index,route);r.routes_[index]=std::move(route);r.account(index,r.routes_[index],1);
        }
        r.best_metrics_=r.measure();exact(r);
    }
    static size_t index_of(NegotiatedRouter& r,const std::string& name) {
        for(size_t i=0;i<r.def_.nets().size();++i) if(r.def_.nets()[i].name==name) return i;
        throw std::runtime_error("fixture missing net");
    }
    static void exercise(NegotiatedRouter& r) {
        forced_chains(r);
        const auto bottom=index_of(r,"bottom0"),top=index_of(r,"top0");
        const auto before=r.best_metrics_;const auto demand=r.demand_;
        const auto owners=r.cell_nets_;
        const auto edges=r.edge_nets_;
        r.options_.low_layer_mask=1;
        const auto bottom_candidates=r.low_layer_candidates(bottom,r.routes_[bottom]);
        require(!bottom_candidates.empty(),"congested bottom chain yields legal candidates");
        require(r.low_layer_candidates(top,r.routes_[top]).empty(),"bottom mask excludes top die");
        for(const auto& candidate:bottom_candidates) {
            require(candidate.slot==r.routes_[bottom].slot,"slot unchanged");
            require(candidate.vias>=4,"lifting to same-direction layer includes four or more ordinary vias");
            require(candidate.measured.wirelength>r.routes_[bottom].measured.wirelength,"all via ladders increase charged length");
            for(int col:{0,19}) require(candidate.nodes.count(r.graph_.node(col,2,1)),"fixed endpoints retained");
            bool interior_removed=false;
            for(int col=1;col<19;++col) interior_removed|=!candidate.nodes.count(r.graph_.node(col,2,1));
            require(interior_removed,"chain interior actually leaves lowest layer");
            for(auto edge:candidate.edges) {
                const auto [a,b]=edge_nodes(edge);require(r.graph_.legal_edge(a,b),"every planar/via edge is legal");
                if(r.graph_.layer(a)!=r.graph_.layer(b)) require(std::abs(r.graph_.layer(a)-r.graph_.layer(b))==1,
                    "via ladder never skips an intermediate layer");
            }
        }
        require(r.demand_==demand&&r.cell_nets_==owners&&r.edge_nets_==edges,"enumeration is read only");
        auto protected_route=r.routes_[bottom];
        PinAccess middle;middle.node=r.graph_.node(9,2,1);protected_route.accesses.push_back(middle);
        const auto protected_candidates=r.low_layer_candidates(bottom,protected_route);
        require(!protected_candidates.empty(),"protected interior splits into separate removable chains");
        for(const auto& candidate:protected_candidates)
            require(candidate.nodes.count(middle.node),"an interior access never disappears");
        auto branched=r.routes_[bottom];const int branch=r.graph_.node(9,2,2);
        branched.nodes.insert(branch);branched.edges.insert(route_edge(middle.node,branch));r.encode(bottom,branched);
        const auto branch_candidates=r.low_layer_candidates(bottom,branched);
        require(!branch_candidates.empty(),"branch boundary still allows neighboring chain lift");
        for(const auto& candidate:branch_candidates)
            require(candidate.nodes.count(middle.node)&&candidate.edges.count(route_edge(middle.node,branch)),
                "external branch and its connection endpoint remain intact");
        r.options_.low_layer_mask=2;
        require(!r.low_layer_candidates(top,r.routes_[top]).empty(),"top mask generates upper-die lifts");
        require(r.low_layer_candidates(bottom,r.routes_[bottom]).empty(),"top mask excludes bottom die");
        r.options_.low_layer_mask=3;
        r.begin_rrr_checkpoint();
        require(r.low_layer_transaction(bottom),"complete documented quality accepts beneficial lift despite added via cost");
        require(r.best_metrics_.official.squared_normalized_overflow<before.official.squared_normalized_overflow,
            "accepted lift reduces whole-graph Q");
        require(r.best_metrics_.guide.quality>before.guide.quality,"accepted lift improves complete quality");
        r.restore_rrr_checkpoint("injected recovery test");
        require(r.demand_==demand&&r.cell_nets_==owners&&r.edge_nets_==edges,"checkpoint rollback restores all resources and owners");
        near(r.best_metrics_.guide.quality,before.guide.quality,"checkpoint rollback restores quality");exact(r);
        // Clear test-only speculative counters; production stage performs the
        // same restoration automatically when checkpoint validation fails.
        r.counters_.low_layer_accepted=0;r.counters_.low_layer_accepted_by_die={};
        r.counters_.low_layer_delta_q=r.counters_.low_layer_delta_length=r.counters_.low_layer_delta_score=0;
        r.options_.low_layer_seconds=30;r.options_.low_layer_max_attempts=100;
        r.run_low_layer_repair();
        require(r.counters_.low_layer_accepted>0,"bounded production phase retains useful work");
        require(r.counters_.low_layer_delta_score>0 && r.counters_.low_layer_delta_q<0,"retained score and Q deltas reported");exact(r);
        const auto old_deadline=r.deadline_;r.deadline_=std::chrono::steady_clock::now();
        const auto attempts=r.counters_.low_layer_attempted;
        r.run_low_layer_repair();require(attempts==r.counters_.low_layer_attempted,"expired budget starts no work");
        r.deadline_=old_deadline;
        const auto saved_mask=r.options_.low_layer_mask;r.options_.low_layer_mask=0;
        r.run_low_layer_repair();require(attempts==r.counters_.low_layer_attempted,"disabled operator starts no work");
        r.options_.low_layer_mask=saved_mask;
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("low_layer_repair_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path folder=argv[1];fs::create_directories(folder);
        std::string tech,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 660 180 ) ; GCELLGRID X 0 DO 23 STEP 30 ; GCELLGRID Y 0 DO 7 STEP 30 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            const auto tracks=(layer==1||layer==11)?"15 DO 22 STEP 30":"0.5 DO 660 STEP 1";
            design+="TRACKS X "+std::string(tracks)+" LAYER "+name+" ; TRACKS Y "+tracks+" LAYER "+name+" ; ";
        }
        design+="COMPONENTS 0 ; END COMPONENTS PINS 14 ; ";
        for(int side=0;side<2;++side) for(int net=0;net<3;++net) for(int pin=0;pin<2;++pin)
            design+="- p"+std::to_string(side)+std::to_string(net)+std::to_string(pin)+" + LAYER metal"+std::to_string(side?11:1)+
                " ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(pin?585:15)+" 75 ) N ; ";
        design+="- lo + LAYER metal3 ( -1 -1 ) ( 1 1 ) + FIXED ( 15 15 ) N ; ";
        design+="- hi + LAYER metal13 ( -1 -1 ) ( 1 1 ) + FIXED ( 645 165 ) N ; END PINS NETS 7 ; ";
        for(int side=0;side<2;++side) for(int net=0;net<3;++net) {
            design+="- "+std::string(side?"top":"bottom")+std::to_string(net);
            for(int pin=0;pin<2;++pin) design+=" ( PIN p"+std::to_string(side)+std::to_string(net)+std::to_string(pin)+" )";
            design+=" ; ";
        }
        design+="- zz_cross ( PIN lo ) ( PIN hi ) ; END NETS END DESIGN";
        LefParser lef;DefParser def;
        require(lef.parse(write(folder/"fixture.lef",tech))&&def.parse(write(folder/"fixture.def",design),lef,DefGridMapping::OFFICIAL_FULL),"fixture parses");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<6;++y) for(int x=0;x<22;++x) {if(x||y) heat<<',';heat<<'['<<1.5+3*x<<','<<1.5+3*y<<",0,"<<26+x+6*y<<']';}
        heat<<"]}";ThermalParser thermal;
        require(thermal.parse_json(write(folder/"heat.json",heat.str()),def.gcell_grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"thermal parses");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];options.time_limit=120;
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        options.official_metrics.score_20261020=true;options.official_metrics.tsv_track_demand=true;
        options.official_metrics.gcell_width_dbu=def.gcell_pitch_x();options.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
        options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;
        options.v7_cell_cost=true;options.v9_incremental_rrr=true;
        NegotiatedRouter router(def,lef,thermal,options);LowLayerRepairTestAccess::exercise(router);
        std::cout<<"low_layer_repair_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
