#pragma once
#include "objectives/official_metrics.hpp"
#include <chrono>
#include <functional>
namespace eda {
struct GuideViaStackOptions {
    std::chrono::steady_clock::time_point deadline;
    size_t max_trials{256},max_windows{32};int radius{8};
    double total_L{},reference{},total_Q{},max_total_L{};
    bool allow_extra_vias{false},reject_all{false};
};
struct GuideViaStackResult {
    std::vector<GuideRect> guides;std::vector<int> removed,added;
    size_t windows{},trials{},invalid{},dominated{},pin_rejections{},canonical_rejections{};
    bool timed_out{};double delta_L{},delta_Q{},gain{};
};
// Pure candidate generation: input Guide and global demand are never mutated.
GuideViaStackResult relocate_guide_via_stack(const Net&,const std::vector<GuideRect>&,
    const RoutingGraph&,const LefParser&,const OfficialMetricOptions&,const GuideViaStackOptions&,
    const std::vector<int>&,const std::function<double(int)>&);
}
