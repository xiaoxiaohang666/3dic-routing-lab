#pragma once
#include "objectives/official_metrics.hpp"
#include "parser/def_parser.hpp"
#include <chrono>
#include <vector>

namespace eda {
// Bounds of the local Guide editing chain. TSV positions never move, so the
// thermal score is unchanged and only W and C decide whether an edit is kept.
struct GuideLocalEditOptions {
    std::chrono::steady_clock::time_point deadline;
    double max_length_growth{.01};
    size_t passes{3};
    size_t stack_nets{4096},stack_trials{1024},stack_rounds{64};
    int stack_radius{16};
    size_t subtree_nets{4096},subtree_trials{256},subtree_rounds{8},subtree_expansions{40000},subtree_windows{4};
    int subtree_radius{4};
    double subtree_net_seconds{5};
};
struct GuideLocalEditStats {
    size_t passes{},stack_accepted{},subtree_accepted{},attempted{},trials{},invalid{},
           pin_rejections{},canonical_rejections{},search_bound_hits{};
    bool timed_out{};
    double seconds{},length_before{},length_after{},q_before{},q_after{};
};
// Edits guides, measured and demand together. Every accepted edit raises
// 45*W+30*C on the shared demand; a stale ledger throws instead of guessing.
GuideLocalEditStats edit_guides_locally(const DefParser&,const RoutingGraph&,const LefParser&,const OfficialMetricOptions&,
    std::vector<std::vector<GuideRect>>& guides,std::vector<NetGuideMetrics>& measured,std::vector<int>& demand,
    double reference,const GuideLocalEditOptions&);
}
