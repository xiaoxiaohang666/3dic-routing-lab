#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace eda {
// Observed scalar conversion only. It does not change physical resources or
// license historical rectangle/PIN loopholes. Zero-capacity remains the
// explicit conservative legacy policy until official observations cover it.
inline long double congestion_cell_penalty(long double demand,long double capacity,long double scale) {
    if(!std::isfinite(demand)||!std::isfinite(capacity)||!std::isfinite(scale)||demand<0||capacity<0||scale<=0)
        throw std::invalid_argument("invalid congestion penalty inputs");
    if(demand==0) return 0;
    if(capacity==0) return 10000;
    const auto ratio=scale*std::max(0.0L,demand-capacity)/capacity;
    return ratio*ratio;
}
inline long double congestion_cell_delta(long double before,long double added,long double capacity,long double scale) {
    if(!std::isfinite(added)||before+added<0) throw std::invalid_argument("invalid congestion demand delta");
    return congestion_cell_penalty(before+added,capacity,scale)-congestion_cell_penalty(before,capacity,scale);
}
} // namespace eda
