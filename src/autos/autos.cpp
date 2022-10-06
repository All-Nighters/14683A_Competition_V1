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
        else if (mode == "BlueLeft") {
            Odom::set_state(0.368826057_m, 0.888775_m, 0_deg);

            Roller::roll("blue");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);

            Auto::moveDistance(2);

            Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);
            pros::delay(2000);

            Auto::simpleMoveToPoint(36.94444444444444, 53.05555555555555);
            Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);
            pros::delay(2000);
        }
        else if (mode == "BlueLeftSupportive") {
            Odom::set_state(0.368826057_m, 0.888775_m, 0_deg);

            Roller::roll("blue");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);

            Auto::moveDistance(2);

            Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);
            pros::delay(2000);

            Auto::faceAngle(50);
            Auto::moveDistance(90);

            Auto::faceAngle(-90);
            Roller::roll("blue");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(1.5*touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);
            // Auto::simpleMoveToPoint(64.07407407407408, 89.53703703703704);
        }
        else if (mode == "RedLeft") {
            Odom::set_state(3.261342592592593_m, 2.774675925925926_m, 180_deg);

            Roller::roll("red");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);

            Auto::moveDistance(2);

            Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], true);
            pros::delay(2000);

            Auto::simpleMoveToPoint(58.33333333333333, 42.5);
            Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], true);
            pros::delay(2000);
        }
        else if (mode == "RedLeftSupportive") {
            Odom::set_state(3.261342592592593_m, 2.774675925925926_m, 180_deg);

            Roller::roll("red");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);

            Auto::moveDistance(2);

            Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], true);
            pros::delay(2000);

            Auto::faceAngle(230);
            Auto::moveDistance(90);

            Auto::faceAngle(90);
            Roller::roll("red");
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            pros::delay(1.5*touchDelay);
            Roller::stop();
            LFMotor.moveVelocity(0);
            RFMotor.moveVelocity(0);
            LBMotor.moveVelocity(0);
            RBMotor.moveVelocity(0);
            pros::delay(1000);
        }
    }
}