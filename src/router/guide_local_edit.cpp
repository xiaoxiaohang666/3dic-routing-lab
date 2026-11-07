#include "router/guide_local_edit.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "router/guide_subtree_reconnector.hpp"
#include "router/guide_via_stack_relocator.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace eda {
namespace {
using Clock=std::chrono::steady_clock;
struct Totals {double length{},q{};};

bool pin_cell(const RoutingGraph& graph,int node) {
    return graph.tsv_obstacle_layers(graph.col(node),graph.row(node),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(graph.layer(node)-1));
}
// One overflow unit a net could still give back; TSV-mandated demand and PIN
// cells are not movable, so they carry no rank.
std::vector<double> movable_ranks(const RoutingGraph& graph,const std::vector<int>& demand,const std::vector<int>& mandatory) {
    std::vector<double> ranks(graph.size());
    for(size_t n=0;n<graph.size();++n) {
        if(demand[n]<=mandatory[n] || pin_cell(graph,int(n))) continue;
        ranks[n]=double(official_resource_cell_contribution(graph,int(n),demand[n]).squared-
                        official_resource_cell_contribution(graph,int(n),demand[n]-1).squared);
    }
    return ranks;
}
// Rank nets by the overflow they could release. A subtree edit costs more on a
// big net, which the size discount reflects; a ladder move does not.
std::vector<size_t> ranked_nets(const std::vector<NetGuideMetrics>& measured,const std::vector<double>& ranks,
    const RoutingGraph& graph,size_t limit,bool size_discount) {
    std::vector<std::pair<double,size_t>> order;
    const int plane=graph.grid().rows*graph.grid().cols;
    for(size_t i=0;i<measured.size();++i) {
        double rank=0;
        for(auto [n,d]:measured[i].demand) {
            if((graph.layer(n)==10||graph.layer(n)==11) &&
               std::find(measured[i].tsv_cells.begin(),measured[i].tsv_cells.end(),n%plane)!=measured[i].tsv_cells.end()) continue;
            rank+=ranks[n]*d;
        }
        if(rank>0) order.emplace_back(size_discount?rank/std::sqrt(1+.05*measured[i].demand.size()):rank,i);
    }
    std::sort(order.begin(),order.end(),[](const auto& a,const auto& b){return a.first!=b.first?a.first>b.first:a.second<b.second;});
    if(order.size()>limit) order.resize(limit);
    std::vector<size_t> nets;nets.reserve(order.size());
    for(const auto& entry:order) nets.push_back(entry.second);
    return nets;
}
// Re-measure the accepted Guide and commit it, refusing a candidate whose
// reported deltas disagree with the exact recomputation.
void commit(size_t i,const Net& net,std::vector<GuideRect> guides,double delta_length,double delta_q,const RoutingGraph& graph,
    const LefParser& lef,const OfficialMetricOptions& mo,std::vector<std::vector<GuideRect>>& all,
    std::vector<NetGuideMetrics>& measured,std::vector<int>& demand,Totals& totals) {
    auto next=measure_official_net_guides(net,guides,graph,lef,mo,measured[i].reference);
    std::map<int,int> delta;
    for(auto [n,d]:measured[i].demand) delta[n]-=d;
    for(auto [n,d]:next.demand) delta[n]+=d;
    long double dq=0;
    for(auto [n,d]:delta) if(d)
        dq+=official_resource_cell_contribution(graph,n,demand[n]+d).squared-official_resource_cell_contribution(graph,n,demand[n]).squared;
    if(std::abs(double(dq)-delta_q)>1e-10 || std::abs(next.wirelength-measured[i].wirelength-delta_length)>1e-7)
        throw std::runtime_error("local Guide edit disagrees with its own measurement");
    for(auto [n,d]:delta) demand[n]+=d;
    totals.length+=delta_length;totals.q+=double(dq);
    measured[i]=std::move(next);all[i]=std::move(guides);
}
}

GuideLocalEditStats edit_guides_locally(const DefParser& def,const RoutingGraph& graph,const LefParser& lef,const OfficialMetricOptions& mo,
    std::vector<std::vector<GuideRect>>& guides,std::vector<NetGuideMetrics>& measured,std::vector<int>& demand,
    double reference,const GuideLocalEditOptions& o) {
    GuideLocalEditStats stats;const auto started=Clock::now();
    const auto& nets=def.nets();
    if(guides.size()!=nets.size() || measured.size()!=nets.size() || demand.size()!=graph.size())
        throw std::invalid_argument("local Guide edit needs one Guide and measurement per net");
    std::vector<int> mandatory(graph.size());
    Totals totals;
    for(const auto& m:measured) {
        totals.length+=m.wirelength;
        for(auto p:m.tsv_points) for(auto [n,d]:mandatory_tsv_contribution(graph,p,1,true,{},true)) mandatory[n]+=d;
    }
    for(size_t n=0;n<graph.size();++n)
        totals.q+=double(official_resource_cell_contribution(graph,int(n),demand[n]).squared);
    stats.length_before=totals.length;stats.q_before=totals.q;
    const double max_length=totals.length*(1+o.max_length_growth);
    const auto expired=[&]{return Clock::now()>=o.deadline;};
    for(size_t pass=0;pass<o.passes && !stats.timed_out;++pass) {
        size_t pass_accepted=0;++stats.passes;
        for(size_t round=0;round<o.stack_rounds && !stats.timed_out;++round) {
            if(expired()) {stats.timed_out=true;break;}
            const auto ranks=movable_ranks(graph,demand,mandatory);size_t accepted=0;
            for(size_t i:ranked_nets(measured,ranks,graph,o.stack_nets,false)) {
                if(expired()) {stats.timed_out=true;break;}
                ++stats.attempted;
                GuideViaStackOptions opt;opt.deadline=o.deadline;opt.max_trials=o.stack_trials;opt.radius=o.stack_radius;
                opt.total_L=totals.length;opt.total_Q=totals.q;opt.reference=reference;opt.max_total_L=max_length;
                auto result=relocate_guide_via_stack(nets[i],guides[i],graph,lef,mo,opt,demand,[&](int n){return ranks[n];});
                stats.trials+=result.trials;stats.invalid+=result.invalid;stats.pin_rejections+=result.pin_rejections;
                stats.canonical_rejections+=result.canonical_rejections;
                if(result.timed_out) {stats.timed_out=true;break;}
                if(result.removed.empty()) continue;
                commit(i,nets[i],std::move(result.guides),result.delta_L,result.delta_Q,graph,lef,mo,guides,measured,demand,totals);
                ++accepted;
            }
            stats.stack_accepted+=accepted;pass_accepted+=accepted;
            if(!accepted) break;
        }
        for(size_t round=0;round<o.subtree_rounds && !stats.timed_out;++round) {
            if(expired()) {stats.timed_out=true;break;}
            const auto ranks=movable_ranks(graph,demand,mandatory);size_t accepted=0;
            for(size_t i:ranked_nets(measured,ranks,graph,o.subtree_nets,true)) {
                if(expired()) {stats.timed_out=true;break;}
                ++stats.attempted;
                GuideSubtreeOptions opt;opt.deadline=o.deadline;opt.max_trials=o.subtree_trials;opt.radius=o.subtree_radius;
                opt.max_windows=o.subtree_windows;opt.max_expansions=o.subtree_expansions;opt.net_seconds=o.subtree_net_seconds;
                opt.total_L=totals.length;opt.total_Q=totals.q;opt.reference=reference;opt.max_total_L=max_length;
                auto result=reconnect_guide_subtree(nets[i],guides[i],graph,lef,mo,opt,demand,[&](int n){return ranks[n];});
                stats.trials+=result.trials;stats.invalid+=result.invalid;stats.pin_rejections+=result.pin_rejections;
                stats.canonical_rejections+=result.canonical_rejections;stats.search_bound_hits+=result.search_bound_hits;
                if(result.timed_out) {stats.timed_out=true;break;}
                if(result.removed.empty()) continue;
                commit(i,nets[i],std::move(result.guides),result.delta_L,result.delta_Q,graph,lef,mo,guides,measured,demand,totals);
                ++accepted;
            }
            stats.subtree_accepted+=accepted;pass_accepted+=accepted;
            if(!accepted) break;
        }
        if(!pass_accepted) break;
    }
    stats.length_after=totals.length;stats.q_after=totals.q;
    stats.seconds=std::chrono::duration<double>(Clock::now()-started).count();
    return stats;
}
}
