#include "main.h"
#include <math.h>

namespace projectile_trajectory {
    /**
    * Calculates the height of the projectile
    * 
    * @param a input angle in radian, 
    * @param x distance x in meters,
    * @param v eject velocity in m/s,
    * @param Vh horizontal velocity in m/s,
    * @param m projectile mass in kg,
    * @param g gravity constant,
    * @param p air density
    * @param Av_temp vertical cross section area in m^3
    * @param Ah_temp horizontal cross section area in m^3
    * @param Cv vertical drag coefficient
    * @param Ch horizontal drag coefficient
    * @param launcher_height launcher height in meters
    * 
    * @returns height at x meters
    */

    float trajectory(float a, float x, float v, float Vh, float m, float g, float p, float Av_temp, float Ah_temp, float Cv, float Ch, float launcher_height) {
        // equations reference: https://www.desmos.com/calculator/67gt9txnba
        float n = a;
        float e = exp(1.0);
        float Av = Av_temp * cos(n) + Ah_temp * sin(n);
        float Ah = Ah_temp * cos(n) + Av_temp * sin(n);

        float Kv = 0.5 * p * Av * Cv;
        float Kh = 0.5 * p * Ah * Ch;

        float t = (m*(pow(e, (Kh*x/m))-1)) / (Kh*(v*cos(n)+Vh));
        float b1 = sqrt(m/(g*Kv)) * atan(v*sin(n)*sqrt(Kv/(m*g)));

        float y;

        if (t > b1) {
            y = -m/Kv * log(cosh(t*sqrt(g*Kv/m) - atan(v*sin(n)*sqrt(Kv / (m*g))))) + m / (2*Kv) * log(Kv * pow(v, 2)* pow(sin(n), 2) / (m*g) + 1) + launcher_height;
        } else {
            y = m/Kv * log(cos(t*sqrt(g*Kv/m) - atan(v*sin(n)*sqrt(Kv / (m*g))))) + m / (2*Kv) * log(Kv * pow(v, 2)* pow(sin(n), 2) / (m*g) + 1) + launcher_height;
        }
        return y;
    }


    /**
     * @brief Solve for eject velocity in (m/s)
     * 
     * @param upper upper bound of eject velocity
     * @param lower lower bound of eject velocity
     * @param step step size for binary approximation
     * @param epoch number of iterations of binary approximation
     * @param x distance to the robot in meters
     * @param a lanuch angle in degrees
     * @param Vh launcher horizontal velocity
     * @param m disk mass
     * @param g gavitational constant
     * @param p air pressure
     * @param Av vertical cross section area
     * @param Ah horizontal cross section area
     * @param Cv vertical drag coefficient 
     * @param Ch horizontal drag coefficient
     * @param y  goal hight in meters
     * @param launcher_height launcher height in meters
     * 
     * @returns eject velocity in (m/s) 
     */
    float solveVelocity(float upper, float lower, float step, float epoch, float x, float a, float Vh, float m, float g, float p, float Av, float Ah, float Cv, float Ch, float y, float launcher_height) {
        float v;
        float f;
        float v_try;

        a = a * M_PI / 180.0; // convert to radian

        for (int i = 0; i < epoch; i++) {
            v_try = (upper + lower) / 2;
            
            f = trajectory(a, x, v_try, Vh, m, g, p, Av, Ah, Cv, Ch, launcher_height) - (y);
            // printf("%f %f %f %f\n", upper, lower, v_try, f);
            
            if (lower > upper) {
                break;
            }
            else if (f > 0) {
                upper = v_try-step;
            }
            else if (f < 0) {
                lower = v_try+step;
            }
            else {
                break;
            }
        }
        if (abs(f) > 0.1) {
            return -1;
        }
            
        return v_try;
    }
}
