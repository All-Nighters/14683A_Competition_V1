#include "main.h"

float formatAngle(float a) {
    int sign = a < 0 ? -1 : 1;
    float positive_a = abs(a);
    float mod = std::fmod(positive_a, 360);
    if (mod <= 180) {
        return sign * mod;
    } 
    else {
        return sign * (mod - 360);
    }
}

float meterToPercentage(float m) {
    return m / fieldLength * 100.0;
}

float percentageToMeter(float p) {
    return p / 100.0 * fieldLength;
}

float clamp(float v, float lo, float hi ) {
    return v < lo ? lo : hi < v ? hi : v;
}

Coordinates absoluteToLocalCoordinate(Coordinates coord) {
    Odom::update_odometry();
    Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
    float xDist = coord.get_x() - selfCoordinate.get_x();
    float yDist = coord.get_y() - selfCoordinate.get_y();

    // apply rotation matrix
    float newX = yDist*cos(selfCoordinate.get_direction()*M_PI/180.0) - xDist*sin(selfCoordinate.get_direction()*M_PI/180.0);
    float newY = yDist*sin(selfCoordinate.get_direction()*M_PI/180.0) + xDist*cos(selfCoordinate.get_direction()*M_PI/180.0);

    return Coordinates(newX, newY, positionSI.theta);
}