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
        void setPath(std::vector<Coordinates> coords);

        int findLookAheadPoint();

        void followPath();
    }
    namespace pure_pursuit {
        void setPath(std::vector<Coordinates> coords);

        int findLookAheadPoint();

        void followPath();
    }
}
