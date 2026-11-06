#include "router/negotiated_router.hpp"
#include <algorithm>
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
std::string write(const fs::path& path,const std::string& text) {
    std::ofstream out(path);out<<text;out.close();require(bool(out),"write fixture");return path.string();
}
bool same(double a,double b) {return std::abs(a-b)<=2e-10*std::max({1.0,std::abs(a),std::abs(b)});}
}
namespace eda {
// A layer swap is one all-or-nothing transaction over several nets. Whatever
// its outcome, the incremental score ledger must equal a full re-measurement,
// a rejected swap must leave every route exactly as it was, and an accepted
// one must raise the score without adding wirelength.
struct LayerSwapTestAccess {
    static std::vector<std::vector<GuideRect>> guides(const NegotiatedRouter& r) {
        std::vector<std::vector<GuideRect>> out;for(const auto& route:r.routes_) out.push_back(route.guides);return out;
    }
    static bool equal(const std::vector<std::vector<GuideRect>>& a,const std::vector<std::vector<GuideRect>>& b) {
        if(a.size()!=b.size()) return false;
        for(size_t i=0;i<a.size();++i) {
            if(a[i].size()!=b[i].size()) return false;
            for(size_t k=0;k<a[i].size();++k) {
                const auto& x=a[i][k];const auto& y=b[i][k];
                if(x.layer!=y.layer||x.x1!=y.x1||x.y1!=y.y1||x.x2!=y.x2||x.y2!=y.y2) return false;
            }
        }
        return true;
    }
    static void ledger(NegotiatedRouter& r,const std::string& when) {
        r.complete_validation_cached_=false;r.validate_complete();
        const auto predicted=r.recovery_metrics(r.best_metrics_);const auto full=r.measure();
        require(same(predicted.guide.wirelength,full.guide.wirelength),when+": wirelength ledger");
        require(same(predicted.guide.quality,full.guide.quality),when+": score ledger");
        require(same(predicted.official.squared_normalized_overflow,full.official.squared_normalized_overflow),when+": overflow ledger");
        require(predicted.official.zero_capacity_cells==full.official.zero_capacity_cells,when+": zero-capacity ledger");
    }
    static void outcomes(const NegotiatedRouter& r) {
        const auto& s=r.swap_stats_;
        require(s.attempted==s.accepted+s.no_gain+s.no_blocker+s.rejected+s.failures,"every swap attempt has one outcome");
    }
    static void run(NegotiatedRouter& r) {
        r.options_.layer_swap=true;
        r.complete_validation_cached_=false;r.validate_complete();r.best_metrics_=r.measure();
        const auto targets=r.layer_swap_targets();
        std::cout<<"targets="<<targets.size()<<'\n';
        for(size_t i:targets) require(r.routes_[i].slot<0 && !r.def_.nets()[i].is_cross_die,"targets are same-die nets");
        // One transaction on every net, target or not, one at a time.
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto before=guides(r);const auto score=r.best_metrics_;
            std::vector<std::pair<size_t,NetRoute>> journal;size_t pending=0;
            const auto accepted=r.swap_stats_.accepted;
            const bool kept=r.layer_swap_transaction(i,journal,pending);
            outcomes(r);
            ledger(r,"after swap of net "+std::to_string(i));
            if(kept) {
                require(r.swap_stats_.accepted==accepted+1 && pending==1,"accepted swap is counted once");
                require(!journal.empty() && journal.front().first==i,"journal starts with the swapped net");
                require(r.best_metrics_.guide.quality>score.guide.quality,"accepted swap raises the score");
                require(r.best_metrics_.guide.wirelength<=score.guide.wirelength,"accepted swap adds no wirelength");
                CompleteMetrics checkpoint=score;
                require(r.recovery_checkpoint(journal,checkpoint),"accepted swap passes the exact checkpoint");
            } else {
                require(journal.empty() && pending==0,"rejected swap leaves no journal");
                require(equal(before,guides(r)),"rejected swap restores every route exactly");
                require(same(r.best_metrics_.guide.quality,score.guide.quality),"rejected swap restores the score");
            }
        }
        // The whole stage, as the schedule runs it.
        const auto start=r.measure();
        r.swap_tried_.clear();r.deadline_=std::chrono::steady_clock::now()+std::chrono::seconds(30);
        r.recover_layer_swaps();
        outcomes(r);
        ledger(r,"after stage");
        const auto end=r.measure();
        require(end.guide.quality+1e-12>=start.guide.quality,"stage never lowers the score");
        require(end.guide.wirelength<=start.guide.wirelength,"stage never adds wirelength");
        for(size_t i=0;i<r.swap_tried_.size();++i)
            require(!r.swap_tried_[i] || (r.routes_[i].slot<0 && !r.def_.nets()[i].is_cross_die),"stage only tries same-die nets");
        std::cout<<"attempted="<<r.swap_stats_.attempted<<" accepted="<<r.swap_stats_.accepted<<" no_gain="<<r.swap_stats_.no_gain
                 <<" no_blocker="<<r.swap_stats_.no_blocker<<" rejected="<<r.swap_stats_.rejected<<" failures="<<r.swap_stats_.failures<<'\n';
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("layer_swap_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path directory=argv[1];fs::create_directories(directory);
        // One track per GCell on every layer: nets sharing a row compete for
        // it, so later nets either detour or climb to higher layers.
        std::string tech,def="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 180 180 ) ; GCELLGRID X 0 DO 7 STEP 30 ; GCELLGRID Y 0 DO 7 STEP 30 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            def+="TRACKS X 15 DO 6 STEP 30 LAYER "+name+" ; TRACKS Y 15 DO 6 STEP 30 LAYER "+name+" ; ";
        }
        def+="COMPONENTS 0 ; END COMPONENTS PINS 12 ; ";
        const auto pin=[&](const std::string& name,int x,int y,int layer=1) {
            def+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        };
        // Two long nets along row 2 and three short ones inside it; one
        // cross-die net away from that row gives the design a TSV.
        pin("l0",15,65);pin("l1",165,65);pin("m0",15,85);pin("m1",165,85);
        pin("s0",75,70);pin("s1",105,70);pin("t0",45,80);pin("t1",105,80);pin("u0",75,75);pin("u1",135,75);
        pin("x0",15,165);pin("x1",165,15,13);
        def+="END PINS NETS 6 ; - l ( PIN l0 ) ( PIN l1 ) ; - m ( PIN m0 ) ( PIN m1 ) ; - s ( PIN s0 ) ( PIN s1 ) ; "
             "- t ( PIN t0 ) ( PIN t1 ) ; - u ( PIN u0 ) ( PIN u1 ) ; - x ( PIN x0 ) ( PIN x1 ) ; END NETS END DESIGN";
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
        NegotiatedRouter router(parsed,lef,thermal,options);
        require(router.route_all(),"fixture routes");
        LayerSwapTestAccess::run(router);
        std::cout<<"layer_swap_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
