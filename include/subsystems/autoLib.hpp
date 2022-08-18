#include "main.h"

typedef struct base_angle_position {
    double LF;
    double RF;
    double RB;
    double LB;
}base_angle_position;

namespace Auto {
    void moveDistance(QLength target_distance);
    void turnAngle(QAngle ang);
    void moveDistanceAsync(QLength target_distance);
    void turnAngleAsync(QAngle ang);
    void waitUntilSettled();
    void test();
}