#pragma once
#include "objectives/guide_metrics.hpp"
namespace eda {
// Observation-only metric, never production candidate acceptance.
double site_rectangle_mst_diagnostic(const std::vector<GuideRect>& guides,const LefParser& lef,double pitch);
}
