#include "main.h"

typedef struct base_angle_position {
    float LF;
    float RF;
    float RB;
    float LB;
}base_angle_position;

namespace Auto {
    bool settled;
    void moveDistance(float target_percentage);
    void turnAngle(float ang);
    void moveDistanceAsync(float target_percentage);
    void turnAngleAsync(float ang);
    void faceCoordinate(float xPercent, float yPercent);
    void faceCoordinateAsync(float xPercent, float yPercent);
    void simpleMoveToPoint(float xPercent, float yPercent);
    void waitUntilSettled();
    void trackVelocityPID(float leftV, float rightV);
}