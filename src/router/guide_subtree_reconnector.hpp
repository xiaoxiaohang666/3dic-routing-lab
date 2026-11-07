#pragma once
#include "objectives/official_metrics.hpp"
#include <chrono>
#include <functional>
namespace eda {
struct GuideSubtreeOptions {
    std::chrono::steady_clock::time_point deadline;
    size_t max_trials{32}, max_windows{4}, max_patch_nodes{64}, max_components{4}, max_expansions{4000}, pareto_limit{8};
    int radius{2}, search_margin{4};double net_seconds{0.5};
    double total_L{}, reference{}, total_Q{}, max_total_L{};bool reject_all{false};
};
struct GuideSubtreeCandidate {
    std::vector<GuideRect> guides;std::vector<int> removed,added;
    double delta_L{},delta_Q{},gain{};long long delta_vias{};
    size_t components{};std::string profile;
};
struct GuideSubtreeResult {
    std::vector<GuideRect> guides;std::vector<int> removed,added;
    std::vector<GuideSubtreeCandidate> pareto;
    size_t windows{},trials{},invalid{},pin_rejections{},canonical_rejections{},astar_expansions{},extraction_rejections{},multi_terminal_candidates{},search_bound_hits{};
    bool timed_out{},work_exhausted{};double delta_L{},delta_Q{},gain{};
};
// Pure saved-Guide candidate generator. No mutation of input/global ledger.
GuideSubtreeResult reconnect_guide_subtree(const Net&,const std::vector<GuideRect>&,
    const RoutingGraph&,const LefParser&,const OfficialMetricOptions&,const GuideSubtreeOptions&,
    const std::vector<int>&,const std::function<double(int)>&);
}
