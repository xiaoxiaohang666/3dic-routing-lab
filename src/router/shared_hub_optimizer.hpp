#pragma once
#include "objectives/official_metrics.hpp"
#include <chrono>
#include <functional>
#include <ostream>

namespace eda {
// A bounded post-route experiment. Groups are frozen from the input Guide's
// actual TSV owners. Only a whole group may move, and only to an empty cell.
// It neither regroups nets nor uses the single-owner dynamic TSV index.
struct SharedHubOptions {
    size_t max_groups{16}, max_moves{8}, max_candidates{12};
    size_t max_tail_nodes{512}, search_nodes{20000};
    int radius{3};
    double seconds{120};
    // Tests can interrupt a staged candidate, before any externally visible
    // mutation. Exceptions propagate with the current complete Guide intact.
    std::function<void()> before_commit;
};
struct SharedHubQuality {
    double L{}, Q{}, rho{}, W{}, C{}, T{}, S{};
    size_t ordinary_vias{}, zero_capacity{};
};
struct SharedHubGroup {
    int cell{-1};
    std::vector<size_t> members;
    double detachable_L{};
    size_t detached_nodes{};
    bool eligible{false};
    std::string reason;
};
struct SharedHubTrial {
    int from{-1}, to{-1};
    size_t members{}, expansions{};
    SharedHubQuality before, after;
    bool accepted{false};
    std::string reason;
};
struct SharedHubReport {
    SharedHubQuality before, after;
    std::vector<SharedHubGroup> groups;
    std::vector<SharedHubTrial> trials;
    size_t evaluated_groups{}, moves{}, candidate_failures{}, search_expansions{};
    bool timed_out{false};
    double setup_seconds{}, search_seconds{};
};
// Conservative GLOBAL_GRID geometry check, including zero-area electrical
// contacts outside the one selected landing. This does not replace full export
// validation and never alters the shared TSV policy or metric definitions.
bool shared_hub_cell_geometry(const std::vector<GuideRect>&, int cell, const RoutingGraph&);
SharedHubReport optimize_shared_hubs(const DefParser&, const LefParser&,
    const RoutingGraph&, const ThermalParser&, const OfficialMetricOptions&,
    const std::vector<double>& references, GuideMap&, const SharedHubOptions& = {});
void write_shared_hub_report(std::ostream&, const SharedHubReport&, const DefParser&);
} // namespace eda
