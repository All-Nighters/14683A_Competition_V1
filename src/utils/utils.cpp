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