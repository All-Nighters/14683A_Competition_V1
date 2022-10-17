/**
 * @file odometry.hpp
 * @brief Odometry code
 */

#include "main.h"

namespace Odom {
	void save_results();
	int position_tracking();
	void init(odomMode mode);
    void debug();
	void set_state(QLength x, QLength y, QAngle angle);
}