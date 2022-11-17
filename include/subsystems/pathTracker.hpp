/**
 * @file pathTracker.hpp
 *
 * @brief Path tracking algorithms
 *
 */

#include <math.h>
#include <vector>

namespace pathTracker {
    inline float lookAheadRadius = 10;
    namespace ramsete {
        void setPath(std::vector<Waypoint> coords);

        int findLookAheadPoint();

        void followPath();
    }
}
