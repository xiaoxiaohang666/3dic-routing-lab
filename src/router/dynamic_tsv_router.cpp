#include "router/negotiated_router.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <iostream>
#include <queue>
#include <unordered_set>

namespace eda {
namespace detail {
// The broad raw pool must survive until live demand has been priced. Asking
// the geometry/thermal index for only the final count can permanently hide a
// slightly weaker geometric candidate with much better actual landing access.
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double squared_overflow,TsvMoveSearchOptions options,bool probe) {
    const size_t final_limit=options.max_candidates;
    options.max_candidates=probe?std::min(size_t(8),2*final_limit):std::min(size_t(64),4*final_limit);
    if(probe) options.max_pins=32;
    auto candidates=proposals.candidates(index,options);
    const auto landing_q=[&](int slot) {
        const auto& s=policy.slots().at(size_t(slot));
        double risk=0;
        for(int node:{s.lower_node,s.upper_node,graph.node(graph.col(s.lower_node),graph.row(s.lower_node),9)}) {
            const double capacity=graph.official_capacity(node);
            if(capacity<=0) {risk+=10000;continue;}
            const double total=demand.at(size_t(node))+graph.official_obstacle_demand(node);
            const double before=graph.congestion_overflow_scale()*std::max(0.0,total-int(capacity))/capacity;
            const double after=graph.congestion_overflow_scale()*std::max(0.0,total+1-int(capacity))/capacity;
            risk+=after*after-before*before;
        }
        return risk;
    };
    const double price=15/std::pow(1+squared_overflow,1.5),old=landing_q(old_slot);
    for(auto& candidate:candidates)
        candidate.estimated_score_delta-=price*(landing_q(candidate.slot)-old);
    std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b) {
        if(a.estimated_score_delta!=b.estimated_score_delta) return a.estimated_score_delta>b.estimated_score_delta;
        return a.slot<b.slot;
    });
    if(candidates.size()>final_limit) candidates.resize(final_limit);
    return candidates;
 }
// Price only the mandatory interface occupancy on a common graph with this
// network removed. Comparing both landing positions against that same residual
// avoids charging its old path as a competing net, and avoids attributing the
// removal of the entire path to a landing-only proposal.
// Nonnegative marginal occupancy price on the common other-net ledger.
// This is also passed into retrieval, before geometry/heat top-k truncation.
double tsv_landing_marginal_q(const RoutingGraph& graph,const SlotCandidate& slot,
        const std::vector<int>& demand,const NetGuideMetrics& old_route,
        const std::vector<int>& own_pin_nodes,const OfficialMetricOptions& metrics) {
    const auto mandatory=mandatory_tsv_contribution(graph,slot.point,
        metrics.tsv_congestion_ratio,metrics.tsv_track_demand,own_pin_nodes,metrics.a150_rules);
    long double value=0;
    for(const auto& [node,added]:mandatory) {
        if(node<0 || size_t(node)>=demand.size())
            throw std::logic_error("TSV landing quote has an invalid interface node");
        const auto found=std::lower_bound(old_route.demand.begin(),old_route.demand.end(),node,
            [](const auto& entry,int key){return entry.first<key;});
        const int owned=found!=old_route.demand.end() && found->first==node?found->second:0;
        const int remaining=demand[size_t(node)]-owned;
        if(remaining<0) throw std::logic_error("TSV landing quote cannot remove absent own demand");
        value+=official_resource_cell_contribution(graph,node,remaining+added).squared-
            official_resource_cell_contribution(graph,node,remaining).squared;
    }
    return double(value);
}
double tsv_landing_q_delta(const RoutingGraph& graph,const SlotCandidate& previous,
        const SlotCandidate& target,const std::vector<int>& demand,
        const NetGuideMetrics& old_route,const std::vector<int>& own_pin_nodes,
        const OfficialMetricOptions& metrics) {
    if(!metrics.tsv_track_demand)
        throw std::invalid_argument("track-aware landing quote requires TSV track demand");
    return tsv_landing_marginal_q(graph,target,demand,old_route,own_pin_nodes,metrics)-
        tsv_landing_marginal_q(graph,previous,demand,old_route,own_pin_nodes,metrics);
}
// Q of the common other-net ledger: the live ledger with this net's complete
// cached contribution withdrawn. Old and new landings are priced on this one
// base, so neither inherits the other's routing or the removal of the path.
long double tsv_quote_base_q(const RoutingGraph& graph,const std::vector<int>& demand,
        const NetGuideMetrics& old_route,long double live_q) {
    if(!std::isfinite(live_q) || live_q<0) throw std::logic_error("TSV quote base needs a finite live Q");
    long double removed=0;
    for(const auto& [node,owned]:old_route.demand) {
        if(node<0 || size_t(node)>=demand.size() || owned<=0)
            throw std::logic_error("TSV quote base has an invalid own cell");
        const int remaining=demand[size_t(node)]-owned;
        if(remaining<0) throw std::logic_error("TSV quote base cannot remove absent own demand");
        removed+=official_resource_cell_contribution(graph,node,demand[size_t(node)]).squared-
            official_resource_cell_contribution(graph,node,remaining).squared;
    }
    if(removed>live_q+1e-9L*std::max(1.0L,live_q))
        throw std::logic_error("TSV quote base: own contribution exceeds the live ledger Q");
    return std::max(0.0L,live_q-removed);
}
// Exact weighted C loss 30*(1/sqrt(1+Q_b)-1/sqrt(1+Q_b+m)) of adding m on Q_b.
// Rationalized against cancellation for tiny m. It is nonnegative and at most
// 30/sqrt(1+Q_b), so retrieval lower bounds that omit it remain valid.
double finite_congestion_quote(long double base_q,long double added_q) {
    if(!std::isfinite(base_q) || base_q<0 || !std::isfinite(added_q) || added_q<0)
        throw std::logic_error("finite TSV quote needs nonnegative finite Q values");
    if(added_q==0) return 0;
    const long double a=std::sqrt(1+base_q),b=std::sqrt(1+base_q+added_q);
    return double(30*added_q/(a*b*(a+b)));
}
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double squared_overflow,TsvMoveSearchOptions options,bool probe,
        const NetGuideMetrics& old_route,const std::vector<int>& own_pin_nodes,
        const OfficialMetricOptions& metrics,bool correct_pricing,bool resource_pool,
        TsvMoveSearchDiagnostics* diagnostics,const TsvQuoteSettings& quote) {
    if(quote.finite && !correct_pricing)
        throw std::invalid_argument("finite TSV quote requires track-aware landing pricing");
    if(!correct_pricing && !resource_pool)
        return dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,
            squared_overflow,options,probe);
    if(correct_pricing && !metrics.tsv_track_demand)
        throw std::invalid_argument("track-aware landing quote requires TSV track demand");
    const size_t final_limit=options.max_candidates;
    // Quote accuracy and pool expansion are separate experiments. Quote-only
    // uses the original raw-pool size and the original strict region admission.
    options.max_candidates=resource_pool?
        (probe?std::min(size_t(32),std::max(size_t(8),4*final_limit)):
            std::min(size_t(128),std::max(size_t(32),8*final_limit))):
        (probe?std::min(size_t(8),2*final_limit):std::min(size_t(64),4*final_limit));
    if(probe) options.max_pins=32;
    const double price=15/std::pow(1+squared_overflow,1.5);
    // One common base per query; the cache below is valid only while this
    // query's ledger snapshot is unchanged.
    const long double base_q=quote.finite?tsv_quote_base_q(graph,demand,old_route,squared_overflow):0.0L;
    if(quote.diagnostics) {
        ++quote.diagnostics->queries;
        if(quote.finite) quote.diagnostics->base_q_drop+=double(squared_overflow-base_q);
    }
    // charge ranks the candidates; derivative is the historical quote, kept
    // for the audit even when the finite charge is selected.
    struct Quote {double charge{},derivative{};};
    std::unordered_map<int,Quote> quote_cache;
    const auto evaluate=[&](int id) {
        const auto cached=quote_cache.find(id);
        if(cached!=quote_cache.end()) return cached->second;
        const auto& slot=policy.slots().at(size_t(id));
        Quote value;
        if(correct_pricing) {
            const double m=tsv_landing_marginal_q(graph,slot,demand,old_route,own_pin_nodes,metrics);
            value.derivative=price*m;
            value.charge=quote.finite?finite_congestion_quote(base_q,m):value.derivative;
        } else {
            double q=0;
            for(int node:{slot.lower_node,slot.upper_node,
                    graph.node(graph.col(slot.lower_node),graph.row(slot.lower_node),9)}) {
                const double capacity=graph.official_capacity(node);
                if(capacity<=0) {q+=10000;continue;}
                const double total=demand.at(size_t(node))+graph.official_obstacle_demand(node);
                const double before =graph.congestion_overflow_scale()*std::max(0.0,total-int(capacity))/capacity;
                const double after =graph.congestion_overflow_scale()*std::max(0.0,total+1-int(capacity))/capacity;
                q+=after*after-before*before;
            }
            value.charge=value.derivative=price*q;
        }
        if(quote.diagnostics) ++quote.diagnostics->quotes;
        quote_cache.emplace(id,value);return value;
    };
    if(resource_pool) {
        options.landing_cost=[&](int id) {return evaluate(id).charge;};options.diagnostics=diagnostics;
        options.resource_node_limit=probe?256:1024;
    }
    auto candidates=proposals.candidates(index,options);
    const auto by_estimate=[](const auto& a,const auto& b) {
        if(a.estimated_score_delta!=b.estimated_score_delta) return a.estimated_score_delta>b.estimated_score_delta;
        return a.slot<b.slot;
    };
    // Without resource-aware retrieval both quotes see the identical raw pool,
    // so the derivative shortlist is an exact counterfactual for this query.
    const bool compare=quote.finite && !resource_pool;
    std::vector<TsvMoveCandidate> derivative_list;
    if(!resource_pool) {
        const auto old=evaluate(old_slot);
        if(compare) {
            derivative_list=candidates;
            for(auto& candidate:derivative_list)
                candidate.estimated_score_delta-=evaluate(candidate.slot).derivative-old.derivative;
            std::stable_sort(derivative_list.begin(),derivative_list.end(),by_estimate);
            if(derivative_list.size()>final_limit) derivative_list.resize(final_limit);
        }
        for(auto& candidate:candidates) candidate.estimated_score_delta-=evaluate(candidate.slot).charge-old.charge;
    }
    std::stable_sort(candidates.begin(),candidates.end(),by_estimate);
    if(candidates.size()>final_limit) candidates.resize(final_limit);
    if(quote.finite && quote.diagnostics) {
        auto& audit=*quote.diagnostics;
        const auto old=evaluate(old_slot);
        for(const auto& candidate:candidates) {
            const auto value=evaluate(candidate.slot);
            const double difference=(value.charge-old.charge)-(value.derivative-old.derivative);
            audit.finite_minus_derivative+=difference;
            audit.abs_finite_minus_derivative+=std::abs(difference);
            audit.finite_lower+=difference<0;
        }
        if(compare) {
            ++audit.compared_queries;audit.shortlisted+=candidates.size();
            for(const auto& candidate:candidates)
                if(std::none_of(derivative_list.begin(),derivative_list.end(),
                        [&](const auto& other) {return other.slot==candidate.slot;})) {
                    ++audit.newly_admitted;
                    if(quote.newly_admitted) quote.newly_admitted->insert(candidate.slot);
                }
        }
    }
    return candidates;
}
// Historical signature: derivative quote, no audit.
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double squared_overflow,TsvMoveSearchOptions options,bool probe,
        const NetGuideMetrics& old_route,const std::vector<int>& own_pin_nodes,
        const OfficialMetricOptions& metrics,bool correct_pricing,bool resource_pool,
        TsvMoveSearchDiagnostics* diagnostics) {
    return dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,squared_overflow,options,probe,
        old_route,own_pin_nodes,metrics,correct_pricing,resource_pool,diagnostics,TsvQuoteSettings{});
}
std::vector<TsvMoveCandidate> dynamic_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double squared_overflow,TsvMoveSearchOptions options,bool probe,
        const NetGuideMetrics& old_route,const std::vector<int>& own_pin_nodes,
        const OfficialMetricOptions& metrics) {
    if(!metrics.tsv_track_demand)
        return dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,
            squared_overflow,options,probe);
    return dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,squared_overflow,
        options,probe,old_route,own_pin_nodes,metrics,true,false,nullptr);
}
// A coarse 8x8 region is only a candidate/admission unit, not a claimed global
// thermal optimum. A landing needs residual capacity on both die interfaces
// and a positive-capacity local exit on each side before expensive rerouting.
bool tsv_region_accessible(const RoutingGraph& graph,const SlotCandidate& slot,
                           const std::vector<int>& demand) {
    const int c=graph.col(slot.lower_node),r=graph.row(slot.lower_node);
    auto available=[&](int node) {
        return node>=0 && graph.legal_node(node) &&
            graph.official_capacity(node)>graph.official_obstacle_demand(node)+demand.at(size_t(node));
    };
    for(int l:{9,10,11,12}) if(!available(graph.node(c,r,l))) return false;
    for(int l:{9,11}) {
        const int start=graph.node(c,r,l);bool exit=false;
        for(int next:graph.neighbors(start)) {
            if((graph.layer(next)>10)!=(l>10) || !available(next)) continue;
            if(graph.col(next)!=c || graph.row(next)!=r) {exit=true;break;}
            for(int farther:graph.neighbors(next))
                if((graph.layer(farther)>10)==(l>10) && available(farther) &&
                   (graph.col(farther)!=c || graph.row(farther)!=r)) {exit=true;break;}
            if(exit) break;
        }
        if(!exit) return false;
    }
    return true;
}
// Resource exhaustion is a cost, not a geometry violation. In the new
// shortlist mode optional access layers do not all need positive residual
// capacity; the subsequent real two-sided reroute proves reachability.
bool tsv_region_accessible(const RoutingGraph& graph,const SlotCandidate& slot,
                           const std::vector<int>& demand,bool soft_admission) {
    if(!soft_admission) return tsv_region_accessible(graph,slot,demand);
    return graph.legal_node(slot.lower_node) && graph.legal_node(slot.upper_node);
}
std::vector<TsvMoveCandidate> regional_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double q,TsvMoveSearchOptions options) {
    const size_t limit=options.max_candidates;
    options.max_candidates=std::min(size_t(64),std::max(size_t(16),4*limit));
    auto pool=dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,q,options,false);
    std::map<std::pair<int,int>,std::vector<TsvMoveCandidate>> regions;
    for(const auto& candidate:pool) {
        const auto& slot=policy.slots().at(size_t(candidate.slot));
        if(candidate.thermal_score_delta<=0 || !tsv_region_accessible(graph,slot,demand)) continue;
        regions[{graph.col(slot.lower_node)/8,graph.row(slot.lower_node)/8}].push_back(candidate);
    }
    // Each viable site contributes at most one TSV capacity. Take one best
    // proposal per region before allocating a second, retaining sparse diversity.
    std::vector<TsvMoveCandidate> result;
    for(size_t depth=0;result.size()<limit;++depth) {
        std::vector<TsvMoveCandidate> layer;
        for(const auto& [region,candidates]:regions) {
            (void)region;if(depth<candidates.size()) layer.push_back(candidates[depth]);
        }
        if(layer.empty()) break;
        std::sort(layer.begin(),layer.end(),[](const auto& a,const auto& b) {
            return a.estimated_score_delta!=b.estimated_score_delta?
                a.estimated_score_delta>b.estimated_score_delta:a.slot<b.slot;
        });
        for(const auto& candidate:layer) {
            result.push_back(candidate);if(result.size()==limit) break;
        }
    }
    return result;
}
std::vector<TsvMoveCandidate> regional_tsv_candidates(const TsvMoveSearch& proposals,
        const RoutingGraph& graph,const TsvPolicy& policy,const std::vector<int>& demand,
        size_t index,int old_slot,double q,TsvMoveSearchOptions options,
        const NetGuideMetrics& old_route,const std::vector<int>& own_pin_nodes,
        const OfficialMetricOptions& metrics,bool correct_pricing,bool resource_pool,
        TsvMoveSearchDiagnostics* diagnostics,const TsvQuoteSettings& quote) {
    if(quote.finite && !correct_pricing)
        throw std::invalid_argument("finite TSV quote requires track-aware landing pricing");
    if(!correct_pricing && !resource_pool)
        return regional_tsv_candidates(proposals,graph,policy,demand,index,old_slot,q,options);
    const size_t limit=options.max_candidates;
    options.max_candidates=std::min(resource_pool?size_t(128):size_t(64),std::max(size_t(16),4*limit));
    // Regional pools use the same quote model but stay out of the audit, which
    // covers the main and probe queries only.
    TsvQuoteSettings pool_quote=quote;pool_quote.diagnostics=nullptr;pool_quote.newly_admitted=nullptr;
    auto pool=dynamic_tsv_candidates(proposals,graph,policy,demand,index,old_slot,q,options,false,
        old_route,own_pin_nodes,metrics,correct_pricing,resource_pool,diagnostics,pool_quote);
    std::map<std::pair<int,int>,std::vector<TsvMoveCandidate>> regions;
    for(const auto& candidate:pool) {
        const auto& slot=policy.slots().at(size_t(candidate.slot));
        if(candidate.thermal_score_delta<=0 || !tsv_region_accessible(graph,slot,demand,resource_pool)) continue;
        regions[{graph.col(slot.lower_node)/8,graph.row(slot.lower_node)/8}].push_back(candidate);
    }
    std::vector<TsvMoveCandidate> result;
    for(size_t depth=0;result.size()<limit;++depth) {
        std::vector<TsvMoveCandidate> layer;
        for(const auto& [region,candidates]:regions) {
            (void)region;if(depth<candidates.size()) layer.push_back(candidates[depth]);
        }
        if(layer.empty()) break;
        std::sort(layer.begin(),layer.end(),[](const auto& a,const auto& b) {
            return a.estimated_score_delta!=b.estimated_score_delta?
                a.estimated_score_delta>b.estimated_score_delta:a.slot<b.slot;
        });
        for(const auto& candidate:layer) {
            result.push_back(candidate);if(result.size()==limit) break;
        }
    }
    return result;
}
}
void NegotiatedRouter::optimize_dynamic_tsv() {
    if(!options_.v8_tsv_dynamic || options_.tsv_rounds<=0 || timed_out()) return;
    // The move index, owner map and Pearson shortcuts assume one net per landing.
    if(shared_landings_active_) {++rejection_reasons_["dynamic TSV skipped: shared TSV cells"];return;}
    if(!options_.official_model || !options_.official_metrics.score_20261020 ||
       options_.tsv_policy.resource_mode!=TsvResourceMode::GLOBAL_GRID)
        throw std::invalid_argument("dynamic TSV scheduling requires official 20261020 global-grid routing");
    using Clock=std::chrono::steady_clock;
    const auto started=Clock::now(),outer_deadline=deadline_;
    const bool outer_trial=v5_topology_trial_;
    const auto baseline=measure();
    std::map<size_t,NetRoute> journal;
    const size_t accepted_start=counters_.v7_tsv_accepted;
    const size_t quote_admission_accepted_start=counters_.tsv_quote_new_admission_accepted;
    const size_t positive_accepted_start=counters_.thermal_positive_accepted;
    const size_t tree_accepted_start=counters_.tsv_tree_move_accepted;
    const auto tree_charge_start=counters_.tsv_tree_move_accepted_charge_delta;
    const double tree_length_start=counters_.tsv_tree_move_accepted_length_delta,
        tree_q_start=counters_.tsv_tree_move_accepted_q_delta,tree_rho_start=counters_.tsv_tree_move_accepted_rho_delta;
    const auto queue_gain_start=counters_.v8_tsv_queue_gain;
    const auto queue_seconds_start=counters_.v8_tsv_queue_seconds;
    const auto queue_visits_start=counters_.v8_tsv_queue_visits;
    const auto explore_seconds_start=counters_.v9_tsv_explore_seconds;
    const auto explore_visits_start=counters_.v9_tsv_explore_visits;
    const auto moved_start=counters_.v9_tsv_moved_unique,region_accepted_start=counters_.v9_region_accepted;
    const auto explore_gain_start=counters_.v9_tsv_explore_gain;
    std::set<size_t> phase_new_moved;
    struct Restore {
        std::function<void()> action;
        ~Restore(){action();}
    } restore{[&] {
        deadline_=outer_deadline;v5_topology_trial_=outer_trial;
        counters_.v7_tsv_seconds+=std::chrono::duration<double>(Clock::now()-started).count();
    }};
    deadline_=std::min(outer_deadline,started+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.v7_tsv_seconds)));
    v5_topology_trial_=true;
    if(timed_out()) {++counters_.v7_tsv_budget_stops;best_metrics_=baseline;return;}
    try {
        std::vector<int> assignment;
        std::vector<size_t> scope;
        for(size_t i=0;i<routes_.size();++i) {
            assignment.push_back(routes_[i].slot);
            if(routes_[i].slot>=0) scope.push_back(i);
        }
        const auto index_started=Clock::now();
        std::unique_ptr<TsvMoveSearch> local_search;
        bool reusable=options_.tsv_index_reuse && bool(dynamic_tsv_search_);
        if(reusable) for(size_t i=0;i<assignment.size();++i)
            if(dynamic_tsv_search_->current_slot(i)!=assignment[i]) {reusable=false;break;}
        if(!reusable) {
            auto fresh=std::make_unique<TsvMoveSearch>(def_,graph_,policy_,thermal_,assignment);
            if(options_.tsv_index_reuse) dynamic_tsv_search_=std::move(fresh);
            else local_search=std::move(fresh);
            ++counters_.tsv_index_builds;
        } else ++counters_.tsv_index_hits;
        auto& proposals=*(options_.tsv_index_reuse?dynamic_tsv_search_:local_search);
        counters_.tsv_index_seconds+=std::chrono::duration<double>(Clock::now()-index_started).count();
        if(std::abs(proposals.current_correlation()-baseline.guide.correlation)>1e-9)
            throw std::logic_error("dynamic TSV initial occupancy differs from encoded Guide");
        if(scope.empty()) {best_metrics_=baseline;return;}

        // All estimates below use weighted 0..100 quality units. The landing
        // proxy sees live normalized Q, including fixed obstacles. It is only
        // a shortlist filter: complete encoded W/C/T decides every commit.
        const auto settings=[&](size_t candidates) {
            const auto current=recovery_metrics(best_metrics_);
            TsvMoveSearchOptions o;o.max_candidates=candidates;
            o.thermal_gain=options_.official_metrics.thermal_gain;
            o.length_price=current.guide.wirelength>current.guide.reference?
                45.0*current.guide.reference/(current.guide.wirelength*current.guide.wirelength):0;
            o.risk_price=0.15/std::pow(1+current.official.squared_normalized_overflow,1.5);
            return o;
        };
        const auto query=[&](size_t i,TsvMoveSearchOptions options,bool cheap,std::set<int>* newly_admitted) {
            const auto current=recovery_metrics(best_metrics_);
            if(options_.tsv_landing_pricing || options_.resource_candidate_pool) {
                const TsvQuoteSettings quote{options_.tsv_finite_quote,&counters_.tsv_quote,newly_admitted};
                return detail::dynamic_tsv_candidates(proposals,graph_,policy_,demand_,i,routes_[i].slot,
                    current.official.squared_normalized_overflow,options,cheap,routes_[i].measured,
                    own_pin_nodes_[i],options_.official_metrics,options_.tsv_landing_pricing,
                    options_.resource_candidate_pool,options.diagnostics,quote);
            }
            return detail::dynamic_tsv_candidates(proposals,graph_,policy_,demand_,i,routes_[i].slot,
                current.official.squared_normalized_overflow,options,cheap);
        };
        const bool attributed_work=options_.v9_tsv_coverage;
        DynamicTsvState local_state;
        auto& state=attributed_work?v9_tsv_state_:local_state;
        if(state.estimates.size()!=routes_.size()) {
            state=DynamicTsvState{};
            state.estimates.resize(routes_.size());state.measured_work.resize(routes_.size());
            state.visited.resize(routes_.size());state.ever_visited.resize(routes_.size());
        }
        auto& estimates=state.estimates;auto& measured_work=state.measured_work;
        auto& visited=state.visited;auto& ever_visited=state.ever_visited;
        auto& recent_gain=state.recent_gain;auto& recent_seconds=state.recent_seconds;
        auto& local_visits=state.local_visits;
        auto& visited_count=state.visited_count;auto& explore_cursor=state.explore_cursor;
        auto& refresh_cursor=state.refresh_cursor;auto& picks=state.picks;
        auto& sweep_accepted=state.sweep_accepted;auto& stale_sweeps=state.stale_sweeps;
        // Per-call probe costs are fully attributed before returning. Retaining
        // them in the persistent estimates would charge the same work twice.
        std::vector<double> probe_work(routes_.size(),0);
        double pending_overhead=0;
        const auto probe=[&](size_t i,bool initial) {
            const auto begin=Clock::now();
            auto o=settings(options_.v8_tsv_probe_candidates);
            // The all-net pass must not spend quadratic MST work on a few
            // high-fanout nets. Those still receive the cached W/Q estimates
            // and remain eligible for the full bounded query on selection.
            auto candidates=query(i,o,true,nullptr);
            ++counters_.v8_tsv_probe_queries;
            if(options_.v9_tsv_coverage && v9_tsv_queried_seen_.insert(i).second)
                ++counters_.v9_tsv_queried_unique;
            if(initial) ++counters_.v8_tsv_probed_nets;
            const double excess=std::max(0.0,routes_[i].measured.wirelength-routes_[i].measured.reference);
            const double qprice=15/std::pow(1+best_metrics_.official.squared_normalized_overflow,1.5);
            DynamicTsvState::Estimate e;
            // Fanout is a deterministic initial runtime proxy; real elapsed
            // full-query/routing time replaces it after the net is visited.
            const double pins=double(def_.nets()[i].pins.size());
            e.work=measured_work[i]>0?measured_work[i]:1e-3*(1+pins*pins/16);
            e.value[0]=o.length_price*excess;
            e.value[2]=qprice*official_movable_priority(i);
            for(const auto& c:candidates) {
                e.value[0]=std::max(e.value[0],-o.length_price*c.proxy_length_delta_dbu);
                e.value[1]=std::max(e.value[1],c.thermal_score_delta-o.length_price*std::max(0.0,c.proxy_length_delta_dbu));
                e.value[2]=std::max(e.value[2],c.estimated_score_delta-c.thermal_score_delta);
            }
            estimates[i]=e;
            const double elapsed=std::chrono::duration<double>(Clock::now()-begin).count();
            counters_.v8_tsv_probe_seconds+=elapsed;
            probe_work[i]+=elapsed;
            if(attributed_work) estimates[i].work+=probe_work[i];
        };
        // Continue the cheap pass across slices instead of restarting at net
        // zero. Reserve most of a V9 slice for actual candidate visits, even
        // when the all-net probe cannot fit into one slice.
        const auto probe_start=Clock::now();
        const auto probe_deadline=attributed_work?
            probe_start+(deadline_-probe_start)/5:deadline_;
        for(size_t k=0;k<scope.size();++k) {
            if(timed_out() || (attributed_work && Clock::now()>=probe_deadline)) break;
            const size_t i=attributed_work?scope[state.probe_cursor++%scope.size()]:scope[k];
            probe(i,true);
        }

        struct Entry {
            double priority;
            size_t net;
            bool operator<(const Entry& other) const {
                return priority!=other.priority?priority<other.priority:net>other.net;
            }
        };
        std::array<std::priority_queue<Entry>,3> queues;
        const auto rebuild=[&] {
            for(auto& queue:queues) queue={};
            for(size_t i:scope) if(!visited[i]) for(size_t q=0;q<3;++q)
                queues[q].push({estimates[i].value[q]/estimates[i].work,i});
        };
        rebuild();
        const auto choose_queue=[&] {
            // Each queue receives an initial measurement. Thereafter use
            // recent measured delta(S)/second with a small exploration term.
            for(size_t q=0;q<3;++q) if(!local_visits[q]) return q;
            size_t best=0;double best_rate=-1;
            for(size_t q=0;q<3;++q) {
                const double rate=recent_gain[q]/std::max(1e-6,recent_seconds[q]);
                const double explore=1e-4/std::sqrt(double(local_visits[q]));
                if(rate+explore>best_rate) {best=q;best_rate=rate+explore;}
            }
            return best;
        };
        bool coverage_done=false;
        for(size_t batch=0;!timed_out() && !coverage_done &&
            (options_.v9_tsv_coverage || batch<size_t(options_.tsv_rounds));++batch) {
            size_t batch_accepted=0;
            std::set<size_t> batch_moved;
            std::unordered_set<int> batch_cells;
            for(size_t n=0;n<options_.v7_tsv_nets && !timed_out();++n) {
                if(visited_count==scope.size()) {
                    if(options_.v9_tsv_coverage) {
                        ++counters_.v9_tsv_sweeps;
                        stale_sweeps=sweep_accepted?0:stale_sweeps+1;
                        sweep_accepted=0;
                        coverage_done=stale_sweeps>=std::max(size_t(1),options_.v9_tsv_stale_sweeps) &&
                            (!options_.thermal_coverage || v6_thermal_queried_seen_.size()>=scope.size());
                    }
                    // Retire this epoch exactly once, including when the call
                    // ends here. A later slice starts a new epoch only after all
                    // assigned nets were visited in the previous one.
                    std::fill(visited.begin(),visited.end(),0);visited_count=0;
                    if(coverage_done) break;
                    rebuild();
                }
                size_t q=choose_queue();size_t i=routes_.size();
                const size_t pick=state.picks%4;
                const bool thermal_rotation=options_.thermal_coverage && pick==2;
                if(thermal_rotation) q=1;
                // A reserved rotation prevents a strong early queue from
                // permanently hiding cheap moves on previously unseen nets.
                const bool exploration=picks++%4==3;
                if(thermal_rotation) {
                    // Independent coverage of the thermal queue: ordinary
                    // all-net visits and cheap probes do not count as heat work.
                    for(size_t k=0;k<scope.size();++k) {
                        const size_t candidate=scope[(explore_cursor+k)%scope.size()];
                        if(!visited[candidate] && !v6_thermal_queried_seen_.count(candidate)) {
                            i=candidate;break;
                        }
                    }
                    if(i==routes_.size()) {
                        auto& queue=queues[q];
                        while(!queue.empty() && visited[queue.top().net]) queue.pop();
                        if(!queue.empty()) {i=queue.top().net;queue.pop();}
                    }
                } else if(exploration) {
                    for(size_t k=0;k<scope.size();++k) {
                        const size_t candidate=scope[explore_cursor++%scope.size()];
                        if(!visited[candidate]) {i=candidate;break;}
                    }
                } else {
                    auto& queue=queues[q];
                    while(!queue.empty()&&visited[queue.top().net]) queue.pop();
                    if(!queue.empty()) {i=queue.top().net;queue.pop();}
                }
                if(i==routes_.size()) break;
                visited[i]=1;++visited_count;
                if(ever_visited[i]) ++counters_.v8_tsv_repeat_visits;
                else {ever_visited[i]=1;++counters_.v8_tsv_unique_nets;}
                if(attributed_work && exploration) ++counters_.v9_tsv_explore_visits;
                else {++counters_.v8_tsv_queue_visits[q];++local_visits[q];}
                const auto visit_started=Clock::now();const double quality_before=best_metrics_.guide.quality;
                const bool thermal_visit=!exploration && q==1;
                TsvMoveSearchDiagnostics diagnostics;
                auto query_options=settings(options_.v7_tsv_candidates);
                if(thermal_visit) query_options.diagnostics=&diagnostics;
                const auto query_started=Clock::now();
                std::set<int> newly_admitted;
                auto candidates=query(i,query_options,false,&newly_admitted);
                std::set<int> region_slots;
                if(options_.v9_thermal_regions && !timed_out()) {
                    const auto regional_options=settings(std::min(size_t(4),options_.v7_tsv_candidates));
                    auto regional=(options_.tsv_landing_pricing || options_.resource_candidate_pool)?
                        detail::regional_tsv_candidates(proposals,graph_,policy_,demand_,i,routes_[i].slot,
                            best_metrics_.official.squared_normalized_overflow,regional_options,routes_[i].measured,
                            own_pin_nodes_[i],options_.official_metrics,options_.tsv_landing_pricing,
                            options_.resource_candidate_pool,thermal_visit?&diagnostics:nullptr,
                            TsvQuoteSettings{options_.tsv_finite_quote,nullptr,nullptr}):
                        detail::regional_tsv_candidates(proposals,graph_,policy_,demand_,i,routes_[i].slot,
                            best_metrics_.official.squared_normalized_overflow,regional_options);
                    counters_.v9_region_candidates+=regional.size();
                    for(const auto& candidate:regional) {
                        region_slots.insert(candidate.slot);
                        if(std::none_of(candidates.begin(),candidates.end(),[&](const auto& c){return c.slot==candidate.slot;}))
                            candidates.push_back(candidate);
                    }
                    std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b) {
                        return a.estimated_score_delta!=b.estimated_score_delta?
                            a.estimated_score_delta>b.estimated_score_delta:a.slot<b.slot;
                    });
                }
                if(thermal_visit) {
                    counters_.thermal_query_seconds+=std::chrono::duration<double>(Clock::now()-query_started).count();
                    if(v6_thermal_queried_seen_.insert(i).second) ++counters_.thermal_query_unique;
                    counters_.thermal_occupied_blocked+=diagnostics.occupied_sites;
                    counters_.thermal_resource_pruned+=diagnostics.resource_pruned;
                    bool positive=false;
                    for(const auto& candidate:candidates) if(candidate.correlation_after>proposals.current_correlation()+1e-14) {
                        positive=true;
                        const auto& slot=policy_.slots().at(size_t(candidate.slot));
                        const auto region=std::make_pair(graph_.col(slot.lower_node)/8,graph_.row(slot.lower_node)/8);
                        if(v6_thermal_regions_seen_.insert(region).second) ++counters_.thermal_regions_unique;
                    }
                    if(positive && v6_thermal_positive_seen_.insert(i).second) ++counters_.thermal_positive_unique;
                }
                const auto route_started=Clock::now();
                counters_.v7_tsv_candidates+=candidates.size();
                for(const auto& candidate:candidates) {
                    if(timed_out()) break;
                    ++counters_.v7_tsv_attempted;
                    const bool positive_heat=candidate.correlation_after>proposals.current_correlation()+1e-14;
                    if(thermal_visit && positive_heat) ++counters_.thermal_positive_trials;
                    const bool quote_admission=newly_admitted.count(candidate.slot)>0;
                    if(quote_admission) ++counters_.tsv_quote_new_admission_attempted;
                    if(thermal_visit && v6_thermal_rerouted_seen_.insert(i).second)
                        ++counters_.thermal_rerouted_unique;
                    if(options_.v9_tsv_coverage && v9_tsv_rerouted_seen_.insert(i).second)
                        ++counters_.v9_tsv_rerouted_unique;
                    const auto before=recovery_metrics(best_metrics_);NetRoute old=routes_[i];
                    bool removed=false,inserted=false;
                    const auto rollback=[&] {
                        if(inserted) account(i,routes_[i],-1);
                        if(removed) {routes_[i]=old;account(i,old,1);}
                        inserted=removed=false;best_metrics_=before;++counters_.v7_tsv_rollbacks;
                    };
                    try {
                        account(i,old,-1);removed=true;routes_[i]=NetRoute{};
                        bool tree_selected=false;
                        auto trial=[&]() -> NetRoute {
                            if(!options_.tsv_tree_move) return build_route(i,candidate.slot);
                            std::optional<NetRoute> tree;
                            const auto tree_started=Clock::now();
                            ++counters_.tsv_tree_move_attempted;
                            try {
                                tree=build_tree_tsv_move(i,candidate.slot,old);
                                if(tree) ++counters_.tsv_tree_move_built;
                                else ++counters_.tsv_tree_move_unavailable;
                            } catch(const std::exception& e) {
                                ++counters_.tsv_tree_move_failures;
                                ++rejection_reasons_[std::string("tree TSV candidate: ")+e.what()];
                            }
                            counters_.tsv_tree_move_seconds+=std::chrono::duration<double>(Clock::now()-tree_started).count();
                            std::optional<NetRoute> rebuilt;
                            try {if(!timed_out()) rebuilt=build_route(i,candidate.slot);}
                            catch(const std::exception&) {if(!tree) throw;}
                            if(tree && rebuilt) {
                                ++counters_.tsv_tree_move_compared;
                                counters_.tsv_tree_move_compared_length_delta+=tree->measured.wirelength-rebuilt->measured.wirelength;
                                const auto base=official_candidate_base();
                                tree_selected=official_candidate_key(*tree,base)<official_candidate_key(*rebuilt,base);
                            } else tree_selected=bool(tree);
                            if(tree_selected) {++counters_.tsv_tree_move_selected;return std::move(*tree);}
                            if(rebuilt) return std::move(*rebuilt);
                            throw std::runtime_error("tree TSV candidate and full rebuild unavailable");
                        }();
                        std::string error;
                        if(trial.measured.tsv_cells.size()!=1 ||
                           !policy_.validate_mapping(def_.nets()[i],trial.guides,trial.slot,error) ||
                           !validate_route(i,trial.guides,error))
                            throw std::logic_error("dynamic TSV invalid encoded candidate: "+error);
                        routes_[i]=std::move(trial);account(i,routes_[i],1);inserted=true;
                        auto after=recovery_metrics(before);
                        after.physical_length+=routes_[i].length-old.length;
                        after.vias=after.vias-old.vias+routes_[i].vias;
                        after.guide.correlation=candidate.correlation_after;
                        after.official.thermal_score=50*(1+std::tanh(options_.official_metrics.thermal_gain*after.guide.correlation));
                        after.guide.quality=(.45*after.official.wirelength_score+.30*after.official.congestion_score+.25*after.official.thermal_score)/100;
                        after.official.guide=after.guide;
                        if(!timed_out() && after.official.zero_capacity_cells<=before.official.zero_capacity_cells &&
                           after.guide.quality>before.guide.quality+1e-12) {
                            journal.try_emplace(i,old);
                            // Collect invalidated routing footprints before
                            // mutating proposal occupancy, so allocation errors
                            // still leave a transaction that can be rolled back.
                            batch_moved.insert(i);
                            for(auto [node,amount]:old.measured.demand) {(void)amount;batch_cells.insert(node);}
                            for(auto [node,amount]:routes_[i].measured.demand) {(void)amount;batch_cells.insert(node);}
                            if(!proposals.commit_move(i,candidate.slot)) throw std::logic_error("dynamic TSV occupancy commit rejected");
                            removed=inserted=false;best_metrics_=after;
                            if(tree_selected) {
                                ++counters_.tsv_tree_move_accepted;
                                const auto& m=routes_[i].measured;
                                counters_.tsv_tree_move_accepted_charge_delta[0]+=m.planar_wirelength-old.measured.planar_wirelength;
                                counters_.tsv_tree_move_accepted_charge_delta[1]+=m.via_wirelength-old.measured.via_wirelength;
                                counters_.tsv_tree_move_accepted_charge_delta[2]+=m.tsv_wirelength-old.measured.tsv_wirelength;
                                counters_.tsv_tree_move_accepted_length_delta+=after.guide.wirelength-before.guide.wirelength;
                                counters_.tsv_tree_move_accepted_q_delta+=after.official.squared_normalized_overflow-before.official.squared_normalized_overflow;
                                counters_.tsv_tree_move_accepted_rho_delta+=after.guide.correlation-before.guide.correlation;
                            }
                            ++counters_.v7_tsv_accepted;++batch_accepted;++sweep_accepted;
                            if(quote_admission) ++counters_.tsv_quote_new_admission_accepted;
                            if(thermal_visit && positive_heat) ++counters_.thermal_positive_accepted;
                            if(options_.v9_tsv_coverage && !v9_tsv_moved_seen_.count(i)) {
                                // Allocate rollback metadata before accepting
                                // this net into the persistent distinct set.
                                phase_new_moved.insert(i);
                                v9_tsv_moved_seen_.insert(i);++counters_.v9_tsv_moved_unique;
                            }
                            if(region_slots.count(candidate.slot)) ++counters_.v9_region_accepted;
                            break;
                        }
                        if(thermal_visit && !timed_out()) {
                            ++counters_.thermal_score_rejected;
                            if(positive_heat) {
                                if(after.guide.wirelength>before.guide.wirelength) ++counters_.thermal_rejected_length;
                                if(after.official.squared_normalized_overflow>before.official.squared_normalized_overflow)
                                    ++counters_.thermal_rejected_q;
                                if(after.guide.wirelength>before.guide.wirelength &&
                                   after.official.squared_normalized_overflow>before.official.squared_normalized_overflow)
                                    ++counters_.thermal_rejected_length_and_q;
                            }
                        }
                        rollback();
                    } catch(const std::exception& e) {
                        if(thermal_visit && !timed_out()) ++counters_.thermal_unreachable;
                        rollback();++counters_.v7_tsv_failures;
                        ++rejection_reasons_[std::string("dynamic TSV trial: ")+e.what()];
                    }
                }
                if(thermal_visit) counters_.thermal_route_seconds+=std::chrono::duration<double>(Clock::now()-route_started).count();
                const double elapsed=std::chrono::duration<double>(Clock::now()-visit_started).count();
                const double gain=100*std::max(0.0,best_metrics_.guide.quality-quality_before);
                const double charged=elapsed+(attributed_work?probe_work[i]+pending_overhead:0);
                probe_work[i]=0;pending_overhead=0;
                if(attributed_work && exploration) {
                    counters_.v9_tsv_explore_seconds+=charged;counters_.v9_tsv_explore_gain+=gain;
                } else {
                    counters_.v8_tsv_queue_seconds[q]+=charged;counters_.v8_tsv_queue_gain[q]+=gain;
                    recent_gain[q]=.75*recent_gain[q]+.25*gain;
                    recent_seconds[q]=.75*recent_seconds[q]+.25*charged;
                }
                measured_work[i]=measured_work[i]>0?.75*measured_work[i]+.25*charged:charged;
                estimates[i].work=std::max(1e-6,measured_work[i]);
            }
            // Refresh moved nets and current TSV owners touching either full
            // footprint, including cells released by a move. A bounded rotating
            // sample also sees global thermal/free-slot changes. Other queue
            // keys remain cached estimates; every selected net gets a fresh
            // candidate query against current occupancy and live demand.
            if(batch_accepted && !timed_out()) {
                const auto refresh_started=Clock::now();
                const double probe_before=counters_.v8_tsv_probe_seconds;
                auto affected=batch_moved;
                for(int node:batch_cells) {
                    if(timed_out()) break;
                    for(int owner:cell_owners(node)) if(routes_[size_t(owner)].slot>=0) affected.insert(size_t(owner));
                }
                const size_t rotating_limit=std::clamp(options_.v7_tsv_nets,size_t(32),size_t(128));
                size_t rotating=0;
                for(size_t k=0;k<scope.size() && rotating<rotating_limit && !timed_out();++k) {
                    const size_t i=scope[refresh_cursor++%scope.size()];
                    if(!visited[i]) {affected.insert(i);++rotating;}
                }
                for(size_t i:affected) if(routes_[i].slot>=0 && !timed_out()) {probe(i,false);++counters_.v8_tsv_refreshes;}
                rebuild();
                const double elapsed=std::chrono::duration<double>(Clock::now()-refresh_started).count();
                counters_.v9_tsv_refresh_seconds+=elapsed;
                if(attributed_work) pending_overhead+=std::max(0.0,elapsed-(counters_.v8_tsv_probe_seconds-probe_before));
            }
            std::cout<<"[dynamic TSV] batch="<<batch+1<<" accepted="<<batch_accepted
                     <<" unique="<<counters_.v8_tsv_unique_nets<<" repeated="<<counters_.v8_tsv_repeat_visits
                     <<" quality="<<best_metrics_.guide.quality*100<<'\n';
        }
        if(timed_out()) ++counters_.v7_tsv_budget_stops;
        struct RecordValidationTime {
            double& seconds; Clock::time_point start=Clock::now();
            ~RecordValidationTime(){seconds+=std::chrono::duration<double>(Clock::now()-start).count();}
        } validation_time{counters_.tsv_final_validation_seconds};
        complete_validation_cached_=false;validate_complete();
        const auto exact=measure();verify_official_ledger(exact);
        if(std::abs(exact.guide.correlation-best_metrics_.guide.correlation)>1e-9 ||
           std::abs(exact.guide.quality-best_metrics_.guide.quality)>1e-9 ||
           exact.guide.quality+1e-10<baseline.guide.quality ||
           exact.official.zero_capacity_cells>baseline.official.zero_capacity_cells)
            throw std::logic_error("dynamic TSV final exact checkpoint mismatch");
        best_metrics_=exact;
    } catch(const std::exception& e) {
        dynamic_tsv_search_.reset(); // accepted proposal moves are about to be rolled back
        for(auto& [i,old]:journal) {account(i,routes_[i],-1);routes_[i]=std::move(old);account(i,routes_[i],1);}
        counters_.v7_tsv_accepted=accepted_start;counters_.v8_tsv_queue_gain=queue_gain_start;
        counters_.tsv_quote_new_admission_accepted=quote_admission_accepted_start;
        counters_.thermal_positive_accepted=positive_accepted_start;
        counters_.tsv_tree_move_accepted=tree_accepted_start;
        counters_.tsv_tree_move_accepted_charge_delta=tree_charge_start;
        counters_.tsv_tree_move_accepted_length_delta=tree_length_start;
        counters_.tsv_tree_move_accepted_q_delta=tree_q_start;
        counters_.tsv_tree_move_accepted_rho_delta=tree_rho_start;
        for(size_t i:phase_new_moved) v9_tsv_moved_seen_.erase(i);
        counters_.v9_tsv_moved_unique=moved_start;counters_.v9_region_accepted=region_accepted_start;
        counters_.v9_tsv_explore_gain=explore_gain_start;
        // Rolled-back moves cannot justify another profitable coverage sweep.
        // Expensive visits remain recorded: that work really was performed.
        if(options_.v9_tsv_coverage) {
            v9_tsv_state_.sweep_accepted=0;v9_tsv_state_.stale_sweeps=0;
        }
        ++counters_.v7_checkpoint_restores;
        ++rejection_reasons_[std::string("dynamic TSV phase restored: ")+e.what()];
        complete_validation_cached_=false;validate_complete();best_metrics_=measure();verify_official_ledger(best_metrics_);
        if(std::abs(best_metrics_.guide.quality-baseline.guide.quality)>1e-9)
            throw std::logic_error("dynamic TSV rollback did not restore phase baseline");
    }
    if(options_.v9_tsv_coverage) {
        // Attribute leftover probes, refresh bookkeeping and final validation,
        // including failed transactions. No expensive phase work is free in
        // reported queue throughput; exploration remains a separate category.
        const double wall=std::chrono::duration<double>(Clock::now()-started).count();
        double charged=counters_.v9_tsv_explore_seconds-explore_seconds_start;
        size_t visits=counters_.v9_tsv_explore_visits-explore_visits_start;
        for(size_t q=0;q<3;++q) {
            charged+=counters_.v8_tsv_queue_seconds[q]-queue_seconds_start[q];
            visits+=counters_.v8_tsv_queue_visits[q]-queue_visits_start[q];
        }
        const double overhead=std::max(0.0,wall-charged);
        if(!visits) counters_.v9_tsv_explore_seconds+=overhead;
        else {
            counters_.v9_tsv_explore_seconds+=overhead*(counters_.v9_tsv_explore_visits-explore_visits_start)/visits;
            for(size_t q=0;q<3;++q) counters_.v8_tsv_queue_seconds[q]+=overhead*
                (counters_.v8_tsv_queue_visits[q]-queue_visits_start[q])/visits;
        }
    }
}
} // namespace eda
