#include "main.h"
#include <string>

namespace Autos {

    Coordinates minimum_path[] = {
        Coordinates(10.083, 24.35, 0), // start (first roller)

        Coordinates(15, 24.35, 0), // move forward

        Coordinates(23.05, 13.79, 0), // second roller

        Coordinates(23.05, 13.79, 0), // move forward

        Coordinates(86.01, 78.88, 0), // third roller

        Coordinates(86.01, 78.88, 0), // move forward

        Coordinates(77.6, 86.38, 0), // fourth roller

        Coordinates(77.6, 86.38, 0), // move forward

    };
    int touchDelay = 300;

    /**
     * @brief very minimum autonomous code
     * 
     */
    void minimum() {
        for (int i = 0; i < sizeof(minimum_path)/sizeof(minimum_path[0]); i++) {
            Coordinates c = minimum_path[i];
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
    }

    /**
     * @brief scoring mode for blue team at the first position (with roller).
     * 
     */
    void blue_first_scoring() {
        Odom::set_state(10.10, 24.35, 0);

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

        Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], false);
        Gun::shootDisk(2); // shoot out preloads

        Intake::turnOn();
        Auto::simpleMoveToPointBackwards(41.57407407407407, 58.7037037037037);
        Intake::turnOff();

        pros::delay(300);
        Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], false);
        Gun::shootDisk(3); // shoot out 3 disks at the middle
    }

    /**
     * @brief supportive mode for blue team at the first position (with roller).
     * 
     */
    void blue_first_supportive() {
        Odom::set_state(10.10, 24.35, 0);

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
        Gun::shootDisk(2); // shoot out preloads

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
    }

    /**
     * @brief scoring mode for blue team at the second position (without roller).
     * 
     */
    void blue_second_scoring() {
        Odom::set_state(41.9, 90.95, -90);
        Auto::moveDistance(34);

        Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);
        Gun::shootDisk(2);


        Auto::simpleMoveToPoint(76.85185185185185, 89.81481481481481);
        Auto::faceAngle(-90);

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
    }

    /**
     * @brief supportive mode for blue team at the second position (without roller).
     * 
     */
    void blue_second_supportive() {
        Odom::set_state(41.9, 90.95, -90);

        Auto::simpleMoveToPoint(41.94444444444444, 57.59259259259259);

        Auto::simpleMoveToPoint(9.907407407407407, 22.685185185185183);
        Auto::faceAngle(0);


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
    }

    /**
     * @brief scoring mode for red team at the first position (with roller).
     * 
     */
    void red_first_scoring() {
        Odom::set_state(86.38888888888889, 76.2037037037037, 180);
        pros::delay(5000);
        Odom::set_state(86.38888888888889, 76.2037037037037, 180);
        controller.setText(0,0,std::to_string(positionSI.theta));

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
        controller.setText(0,0,std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent));

        pros::delay(1000);

        controller.setText(0,0,std::to_string(positionSI.theta));
        Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], false);
        Gun::shootDisk(2);

        Intake::turnOn();
        Auto::simpleMoveToPointBackwards(56.85185185185185, 46.11111111111111);
        Intake::turnOff();

        Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], false);
        Gun::shootDisk(3);
    }

    /**
     * @brief supportive mode for red team at the first position (with roller).
     * 
     */
    void red_first_supportive() {
        Odom::set_state(86.38888888888889, 76.2037037037037, 180);

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
        Gun::shootDisk(2);

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
    
    /**
     * @brief scoring mode for red team at the second position (without roller).
     * 
     */
    void red_second_scoring() {
        Odom::set_state(58.05, 10.41, 90);

        Auto::simpleMoveToPoint(58.24074074074074, 42.87037037037037);

        Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], true);
        Gun::shootDisk(3);


        Auto::simpleMoveToPoint(22.407407407407405, 10.462962962962962);
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

    /**
     * @brief supportive mode for red team at the second position (without roller).
     * 
     */
    void red_second_supportive() {
        Odom::set_state(58.05, 10.41, 90);
        
        Auto::simpleMoveToPoint(58.148148148148145, 42.68518518518518);
        
        Auto::simpleMoveToPoint(89.16666666666666, 77.5);
        Auto::faceAngle(180);

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
        
    }

    void run(AutoProcedure mode) {

        int round_begin_milliseconds = pros::millis();
        // while (pros::millis() - round_begin_milliseconds < 45*1000) {
        //     printf("%f\n", pros::millis() - round_begin_milliseconds);
        // }
        if (mode == IDLE_FIRST) {
            ; // set odom position
        }
        else if (mode == IDLE_SECOND) {
            ; // set odom position
        }


        /*
            Blue team autonomous
            1. Scoring mode
            2. Supportive mode
            3. Idle
        */
        else if (mode == BLUE_FIRST_SCORING) {
            blue_first_scoring();
        }
        else if (mode == BLUE_FIRST_SUPPORTIVE) {
            blue_first_supportive();
        }
        else if (mode == RED_FIRST_SCORING) {
            red_first_scoring();
        }
        else if (mode == RED_FIRST_SUPPORTIVE) {
            red_first_supportive();
        } 
        else if (mode == DQ) {
            Auto::trackVelocityPID(200, 200);
            pros::delay(5000);
            Auto::trackVelocityPID(0, 0);
        }
    }
}