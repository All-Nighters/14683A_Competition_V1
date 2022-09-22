#include "main.h"
#include <string>

namespace Autos {

    Coordinates minimum[] = {
        Coordinates(10.083, 24.35, 0), // start (first roller)

        Coordinates(15, 24.35, 0), // move forward

        Coordinates(23.05, 13.79, 0), // second roller

        Coordinates(23.05, 13.79, 0), // move forward

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
            // Auto::moveDistance(40);
            // Auto::turnAngle(90);
            // Auto::simpleMoveToPoint(minimum[1].get_x(), minimum[1].get_y());
            // Auto::faceCoordinate(minimum[1].get_x(), minimum[1].get_y(), false);
            // Auto::faceCoordinate(1, 1, false);
            
            // controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
            // Auto::simpleMoveToPoint(minimum[0].get_x(), minimum[0].get_y());
            // controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
            // pros::delay(10000);
            // Auto::simpleMoveToPoint(minimum[1].get_x(), minimum[1].get_y());
            // controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
            // pros::delay(10000);
            // Auto::simpleMoveToPoint(minimum[2].get_x(), minimum[2].get_y());
            // controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
            // pros::delay(10000);
            // Auto::simpleMoveToPoint(minimum[3].get_x(), minimum[3].get_y());
            // controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
            // pros::delay(10000);


            // Roller::roll("blue");
            // pros::delay(touchDelay);
            // Roller::stop();

            for (int i = 0; i < sizeof(minimum)/sizeof(minimum[0]); i++) {
                Coordinates c = minimum[i];
                Auto::simpleMoveToPoint(c.get_x(), c.get_y());
                controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));
                pros::delay(10000);

                if (i == 2) { // second roller
                    Auto::turnAngle(135);
                    Roller::roll("red");
                    LFMotor.moveVelocity(-400);
                    RFMotor.moveVelocity(-400);
                    LBMotor.moveVelocity(-400);
                    RBMotor.moveVelocity(-400);
                    pros::delay(touchDelay);
                    Roller::stop();
                }
                else if (i == 4) { // third roller
                    Auto::turnAngle(135);
                    Roller::roll("red");
                    LFMotor.moveVelocity(-400);
                    RFMotor.moveVelocity(-400);
                    LBMotor.moveVelocity(-400);
                    RBMotor.moveVelocity(-400);
                    pros::delay(touchDelay);
                    Roller::stop();
                }
                else if (i == 6) { // fourth roller
                    Auto::turnAngle(135);
                    Roller::roll("blue");
                    LFMotor.moveVelocity(-400);
                    RFMotor.moveVelocity(-400);
                    LBMotor.moveVelocity(-400);
                    RBMotor.moveVelocity(-400);
                    pros::delay(touchDelay);
                    Roller::stop();
                }

            }
            
            //     Auto::turnAngle(-45);
            
        }
    }
}