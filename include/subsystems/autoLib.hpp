/**
 * @file autoLib.hpp
 *
 * @brief Basic autonomous movement functions
 *
 */

#include "main.h"

namespace Auto {
    extern bool settled;
    void moveDistance(float target_percentage, float max_voltage = 8000);
    void distancePID(float percentage, float max_voltage = 8000);
    float directionPIDStep(float target_angle, float prev_error);
    void turnAngle(float ang);
    void faceAngle(float ang);
    void moveDistanceAsync(float target_percentage);
    void turnAngleAsync(float ang);
    void faceCoordinate(float xPercent, float yPercent, bool aimMode);
    void faceCoordinateAsync(float xPercent, float yPercent, bool aimMode);
    void simpleMoveToPoint(float xPercent, float yPercent);
    void simpleMoveToPointBackwards(float xPercent, float yPercent);
    void waitUntilSettled();
}