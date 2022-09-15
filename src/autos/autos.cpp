#include "main.h"

namespace Autos {

    Coordinates minimum[] = {
        Coordinates(10.083, 24.35, 0), // start (first roller)

        Coordinates(15, 24.35, 0), // move forward

        Coordinates(22.31, 13.79, 0), // second roller

        Coordinates(22.31, 13.79, 0), // move forward

        Coordinates(86.01, 78.88, 0), // third roller

        Coordinates(86.01, 78.88, 0), // move forward

        Coordinates(77.6, 86.38, 0), // fourth roller

        Coordinates(77.6, 86.38, 0), // move forward

    };

    void run(std::string mode) {
        int touchDelay = 300;

        int round_begin_milliseconds = pros::millis();
        // while (pros::millis() - round_begin_milliseconds < 45*1000) {
        //     printf("%f\n", pros::millis() - round_begin_milliseconds);
        // }
        if (mode == "minimum") {
            // Auto::moveDistance(-40);
            // Auto::turnAngle(90);

            Roller::roll("red");
            LFMotor.moveVelocity(400);
            RFMotor.moveVelocity(400);
            LBMotor.moveVelocity(400);
            RBMotor.moveVelocity(400);
            pros::delay(touchDelay);
            Roller::stop();
            Auto::moveDistance(-10);
            Auto::turnAngle(45);
            Auto::moveDistance(-10);
            pros::delay(40);
            while (pros::millis() - round_begin_milliseconds < 45*1000) {
                ;
            }
            piston.set_value(true); // remember to change the value in main.cpp
            pros::delay(500);
            Auto::moveDistance(10);



            // for (int i = 0; i < sizeof(minimum)/sizeof(minimum[0]); i++) {
            //     Coordinates c = minimum[i];
            //     Auto::simpleMoveToPoint(c.get_x(), c.get_y());

            //     // if (i == 0) { // first roller
            //     //     Roller::roll("blue");
            //     //     pros::delay(touchDelay);
            //     //     Roller::stop();
            //     // }
            //     // else if (i == 2) { // second roller
            //     //     Auto::turnAngle(135);
            //     //     Roller::roll("red");
            //     //     LFMotor.moveVelocity(-400);
            //     //     RFMotor.moveVelocity(-400);
            //     //     LBMotor.moveVelocity(-400);
            //     //     RBMotor.moveVelocity(-400);
            //     //     pros::delay(touchDelay);
            //     //     Roller::stop();
            //     // }
            //     // else if (i == 4) { // third roller
            //     //     Auto::turnAngle(135);
            //     //     Roller::roll("red");
            //     //     LFMotor.moveVelocity(-400);
            //     //     RFMotor.moveVelocity(-400);
            //     //     LBMotor.moveVelocity(-400);
            //     //     RBMotor.moveVelocity(-400);
            //     //     pros::delay(touchDelay);
            //     //     Roller::stop();
            //     // }
            //     // else if (i == 6) { // fourth roller
            //     //     Auto::turnAngle(135);
            //     //     Roller::roll("blue");
            //     //     LFMotor.moveVelocity(-400);
            //     //     RFMotor.moveVelocity(-400);
            //     //     LBMotor.moveVelocity(-400);
            //     //     RBMotor.moveVelocity(-400);
            //     //     pros::delay(touchDelay);
            //     //     Roller::stop();
            //     // }

            // }
            
            //     Auto::turnAngle(-45);
            
        }
    }
}