#include "main.h"

typedef struct base_angle_position {
    float LF;
    float RF;
    float RB;
    float LB;
}base_angle_position;

namespace Auto {
    void moveDistance(float target_distance);
    void turnAngle(float ang);
    void moveDistanceAsync(float target_distance);
    void turnAngleAsync(float ang);
    void waitUntilSettled();
    void test();
}