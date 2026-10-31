#include "router/negotiated_router.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace eda;
namespace fs=std::filesystem;
namespace eda::detail {
double tsv_landing_q_delta(const RoutingGraph&,const SlotCandidate&,const SlotCandidate&,
    const std::vector<int>&,const NetGuideMetrics&,const std::vector<int>&,const OfficialMetricOptions&);
bool tsv_region_accessible(const RoutingGraph&,const SlotCandidate&,const std::vector<int>&);
bool tsv_region_accessible(const RoutingGraph&,const SlotCandidate&,const std::vector<int>&,bool);
double tsv_landing_marginal_q(const RoutingGraph&,const SlotCandidate&,const std::vector<int>&,
    const NetGuideMetrics&,const std::vector<int>&,const OfficialMetricOptions&);
long double tsv_quote_base_q(const RoutingGraph&,const std::vector<int>&,const NetGuideMetrics&,long double);
double finite_congestion_quote(long double,long double);
}
namespace {
size_t checks=0;
void require(bool value,const std::string& message) {
    ++checks;if(!value) throw std::runtime_error(message);
}
void near(double actual,double expected,const std::string& message) {
    require(std::abs(actual-expected)<=1e-10*std::max({1.0,std::abs(actual),std::abs(expected)}),message);
}
std::string write(const fs::path& path,const std::string& value) {
    std::ofstream stream(path);stream<<value;stream.close();require(bool(stream),"write fixture");return path.string();
}
struct Fixture {
    LefParser lef;
    DefParser def;
    std::unique_ptr<RoutingGraph> graph;
    Fixture(const fs::path& dir,bool zero_lower=false,double scale=0.1) {
        std::string technology,design="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 400 400 ) ; GCELLGRID X 0 DO 5 STEP 100 ; GCELLGRID Y 0 DO 5 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            technology+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            if(zero_lower && layer==10) continue;
            design+="TRACKS X 2 DO 40 STEP 10 LAYER "+name+" ; TRACKS Y 2 DO 40 STEP 10 LAYER "+name+" ; ";
        }
        // The parser requires a real NETS record. Keep this ordinary net on
        // M1, away from the M10/M11 landing cells used by the quote tests.
        design+="COMPONENTS 0 ; END COMPONENTS PINS 2 ; "
            "- a + LAYER metal1 ( -1 -1 ) ( 1 1 ) + FIXED ( 350 350 ) N ; "
            "- b + LAYER metal1 ( -1 -1 ) ( 1 1 ) + FIXED ( 350 250 ) N ; "
            "END PINS NETS 1 ; - ordinary ( PIN a ) ( PIN b ) ; END NETS END DESIGN";
        require(lef.parse(write(dir/"technology.lef",technology)),"parse technology");
        require(def.parse(write(dir/"design.def",design),lef,DefGridMapping::OFFICIAL_FULL),"parse design");
        RoutingGraphOptions options;
        options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;options.allow_nonpreferred=false;
        options.score_20261020=true;options.documented_rules=true;
        options.congestion_overflow_scale=scale;
        graph=std::make_unique<RoutingGraph>(def,lef,options);
    }
    SlotCandidate landing(int col,int row) const {
        SlotCandidate slot;
        slot.lower_node=graph->node(col,row,10);slot.upper_node=graph->node(col,row,11);
        slot.point=graph->point(slot.lower_node);return slot;
    }
};
NetGuideMetrics own(std::initializer_list<std::pair<int,int>> entries) {
    NetGuideMetrics route;route.demand=entries;
    std::sort(route.demand.begin(),route.demand.end());return route;
}
// Brute-force whole-graph Q, independent of every incremental quote helper.
long double full_q(const RoutingGraph& graph,const std::vector<int>& demand) {
    long double q=0;
    for(size_t node=0;node<graph.size();++node)
        q+=official_resource_cell_contribution(graph,int(node),demand[node]).squared;
    return q;
}
double weighted_c(long double q) {return double(30/std::sqrt(1+q));}
// Finite quote: one common base with the moving net removed, exact C change.
void finite_quote_checks(const fs::path& dir,double scale) {
    fs::create_directories(dir);Fixture fixture(dir,false,scale);const auto& graph=*fixture.graph;
    const auto previous=fixture.landing(0,0),target=fixture.landing(1,0),third=fixture.landing(2,0);
    OfficialMetricOptions metrics;metrics.score_20261020=true;metrics.tsv_track_demand=true;
    // The moving net owns its old TSV plus ordinary cells, one shared with the target.
    const auto old=own({{previous.lower_node,5},{previous.upper_node,5},
                        {graph.node(0,0,9),1},{target.upper_node,1}});
    std::vector<int> demand(graph.size());
    demand[size_t(previous.lower_node)]=demand[size_t(previous.upper_node)]=8;
    demand[size_t(target.lower_node)]=7;demand[size_t(target.upper_node)]=6;
    demand[size_t(third.lower_node)]=demand[size_t(third.upper_node)]=12;
    demand[size_t(graph.node(0,0,9))]=11;
    for(const auto& [node,amount]:old.demand) demand[size_t(node)]+=amount;
    const auto saved=demand;
    const long double live=full_q(graph,demand);
    auto residual=demand;for(const auto& [node,amount]:old.demand) residual[size_t(node)]-=amount;
    const long double base=full_q(graph,residual);
    near(double(detail::tsv_quote_base_q(graph,demand,old,live)),double(base),
         "common base equals a whole-graph recount with the moving net removed");
    require(base<live,"the fixture's own route contributes positive Q");
    std::vector<double> quotes;
    for(const auto* slot:{&previous,&target,&third}) {
        auto with=residual;
        for(const auto& [node,amount]:mandatory_tsv_contribution(graph,slot->point,1,true)) with[size_t(node)]+=amount;
        const long double after=full_q(graph,with);
        const double m=detail::tsv_landing_marginal_q(graph,*slot,demand,old,{},metrics);
        near(m,double(after-base),"landing Delta Q equals the whole-graph recount on the common base");
        const double quote=detail::finite_congestion_quote(base,m);
        near(quote,weighted_c(base)-weighted_c(after),"finite quote equals the exact weighted C loss");
        require(quote>=0 && quote<=weighted_c(base),"finite quote is nonnegative and bounded by the base C");
        quotes.push_back(quote);
    }
    {
        auto at_previous=residual,at_target=residual;
        for(const auto& [node,amount]:mandatory_tsv_contribution(graph,previous.point,1,true)) at_previous[size_t(node)]+=amount;
        for(const auto& [node,amount]:mandatory_tsv_contribution(graph,target.point,1,true)) at_target[size_t(node)]+=amount;
        near(quotes[1]-quotes[0],weighted_c(full_q(graph,at_previous))-weighted_c(full_q(graph,at_target)),
             "old and new landings are compared on the same base");
    }
    require(demand==saved,"finite quotes leave the live ledger unchanged");
    require(detail::finite_congestion_quote(7,0)==0,"zero added Q is free");
    require(detail::finite_congestion_quote(7,1)<detail::finite_congestion_quote(7,2),"finite quote increases with Delta Q");
    require(detail::finite_congestion_quote(7,1e12)<weighted_c(7),"finite quote saturates below the remaining C");
    near(detail::finite_congestion_quote(254,1000),weighted_c(254)-weighted_c(1254),
         "plan example: Q 254 plus 1000 loses the exact C difference");
    require(15/std::pow(255.0,1.5)*1000>3*detail::finite_congestion_quote(254,1000),
            "the live derivative overstates a large finite increase more than threefold");
    bool rejected=false;
    try {detail::finite_congestion_quote(-1,1);} catch(const std::logic_error&) {rejected=true;}
    require(rejected,"negative base Q is rejected");
    rejected=false;
    try {detail::finite_congestion_quote(1,-1);} catch(const std::logic_error&) {rejected=true;}
    require(rejected,"negative added Q is rejected");
    rejected=false;
    try {detail::tsv_quote_base_q(graph,demand,old,0);} catch(const std::logic_error&) {rejected=true;}
    require(rejected,"a live Q below the net's own contribution is inconsistent");
    auto absent=demand;absent[size_t(graph.node(0,0,9))]=0;rejected=false;
    try {detail::tsv_quote_base_q(graph,absent,old,live);} catch(const std::logic_error&) {rejected=true;}
    require(rejected,"withdrawing absent own demand is rejected");
}
}
int main(int argc,char** argv) {
    try {
        if(argc!=2) throw std::runtime_error("tsv_landing_candidates_tests OUTPUT_DIRECTORY");
        const fs::path output=argv[1];fs::create_directories(output/"positive");
        Fixture fixture(output/"positive");const auto& graph=*fixture.graph;
        const auto previous=fixture.landing(0,0),target=fixture.landing(1,0);
        require(graph.official_capacity(previous.lower_node)==10 &&
                graph.official_capacity(previous.upper_node)==10,"fixture provides ten real tracks per interface");
        require(graph.tsv_landing_tracks(previous.point,10)==5 &&
                graph.tsv_landing_tracks(previous.point,11)==5,"five-micron landing reserves five real tracks");
        OfficialMetricOptions metrics;metrics.score_20261020=true;metrics.tsv_track_demand=true;
        auto old=own({{previous.lower_node,5},{previous.upper_node,5},
                      {target.lower_node,1},{target.upper_node,1}});
        std::vector<int> demand(graph.size());
        demand[size_t(previous.lower_node)]=demand[size_t(previous.upper_node)]=15;
        demand[size_t(target.lower_node)]=demand[size_t(target.upper_node)]=9;
        const auto saved=demand;
        // Old landing: 2*(0.1*5/10)^2. New landing after removing this
        // network's old crossing: 2*(0.1*3/10)^2. Other owners remain present.
        near(detail::tsv_landing_q_delta(graph,previous,target,demand,old,{},metrics),-.0032,
             "landing relocation removes own old demand before pricing both positions");
        require(demand==saved,"candidate quote does not mutate the live ledger");
        auto other=demand;other[size_t(target.lower_node)]+=2;other[size_t(target.upper_node)]+=2;
        near(detail::tsv_landing_q_delta(graph,previous,target,other,old,{},metrics),0,
             "another network's target occupancy is retained rather than credited to this net");
        std::vector<int> exempt{target.lower_node,target.upper_node};std::sort(exempt.begin(),exempt.end());
        near(detail::tsv_landing_q_delta(graph,previous,target,demand,old,exempt,metrics),-.005,
             "own-PIN interface cells receive only their own exemption");
        exempt={target.lower_node};
        near(detail::tsv_landing_q_delta(graph,previous,target,demand,old,exempt,metrics),-.0041,
             "PIN exemption remains layer-specific");
        auto absent=demand;absent[size_t(previous.lower_node)]=4;bool rejected=false;
        try { detail::tsv_landing_q_delta(graph,previous,target,absent,old,{},metrics); }
        catch(const std::logic_error&) {rejected=true;}
        require(rejected,"withdrawal of absent own demand is rejected");
        auto inactive=metrics;inactive.tsv_track_demand=false;rejected=false;
        try { detail::tsv_landing_q_delta(graph,previous,target,demand,old,{},inactive); }
        catch(const std::invalid_argument&) {rejected=true;}
        require(rejected,"track quote requires explicit resource opt-in");

        std::vector<int> empty(graph.size());
        require(detail::tsv_region_accessible(graph,target,empty),"legacy admission accepts an uncongested fixture");
        auto blocked=empty;blocked[size_t(target.upper_node)]=10;
        require(!detail::tsv_region_accessible(graph,target,blocked),"legacy admission preserves its strict capacity gate");
        require(!detail::tsv_region_accessible(graph,target,blocked,false),"explicit legacy admission remains unchanged");
        require(detail::tsv_region_accessible(graph,target,blocked,true),"soft admission retains a legal exhausted interface candidate");
        blocked=empty;blocked[size_t(graph.node(1,0,9))]=10;
        require(!detail::tsv_region_accessible(graph,target,blocked),"legacy optional access layer can block admission");
        require(detail::tsv_region_accessible(graph,target,blocked,true),"soft admission permits other actual access-layer choices");
        auto invalid=target;invalid.lower_node=-1;
        require(!detail::tsv_region_accessible(graph,invalid,empty,true),"soft admission still rejects invalid geometry");

        fs::create_directories(output/"zero");Fixture zero(output/"zero",true);
        const auto& zgraph=*zero.graph;const auto zprevious=zero.landing(0,0),ztarget=zero.landing(1,0);
        require(zgraph.official_capacity(zprevious.lower_node)==0,"zero-capacity fixture has no lower tracks");
        auto zold=own({{zprevious.lower_node,1},{zprevious.upper_node,5}});
        std::vector<int> zdemand(zgraph.size());
        zdemand[size_t(zprevious.lower_node)]=1;zdemand[size_t(zprevious.upper_node)]=5;
        zdemand[size_t(ztarget.lower_node)]=1;zdemand[size_t(ztarget.upper_node)]=8;
        near(detail::tsv_landing_q_delta(zgraph,zprevious,ztarget,zdemand,zold,{},metrics),-9999.9991,
             "already occupied zero-capacity target does not receive a second fixed event penalty");
        finite_quote_checks(output/"finite-scale-0.1",0.1);
        finite_quote_checks(output/"finite-scale-1",1.0);
        std::cout<<"tsv_landing_candidates_tests: "<<checks<<" checks PASS\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
