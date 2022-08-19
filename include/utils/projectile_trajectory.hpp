#include "main.h"
#include <math.h>

namespace projectile_trajectory {
    float trajectory(float a, float x, float v, float Vh, float m, float g, float p, float Av_temp, float Ah_temp, float Cv, float Ch, float launcher_height);
    float solveVelocity(float upper, float lower, float step, float epoch, float x, float a, float Vh, float m, float g, float p, float Av, float Ah, float Cv, float Ch, float y, float launcher_height);
}