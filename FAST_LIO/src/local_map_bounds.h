#pragma once
#include <algorithm>

inline double localMapMovementMargin(double side, double range, double threshold) {
    const double requested = threshold * range;
    return requested < side * .5 ? requested : side * .25;
}
inline double localMapMovementDistance(double side, double margin, double threshold) {
    return std::max((side - 2 * margin) * .5 * .9,
                    (margin / threshold) * (threshold - 1));
}
