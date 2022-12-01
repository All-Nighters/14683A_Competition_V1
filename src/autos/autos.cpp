#include "main.h"
#include <string>

namespace Autos {
    float touchDelay = 300;

    /**
     * @brief scoring mode for blue team at the first position (with roller).
     * 
     */
    void blue_first_scoring() {
        Odom::set_state(10.10, 24.35, 0);

        // LFMotor.moveVelocity(-400);
        // RFMotor.moveVelocity(-400);
        // LBMotor.moveVelocity(-400);
        // RBMotor.moveVelocity(-400);
        // pros::delay(touchDelay);
        // LFMotor.moveVelocity(0);
        // RFMotor.moveVelocity(0);
        // LBMotor.moveVelocity(0);
        // RBMotor.moveVelocity(0);
        // pros::delay(1000);

        Auto::moveDistance(2);

        Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], false);
        Flywheel::setLinearEjectVelocity(7);
        pros::delay(3000);
        Intake::turnOn();
        Gun::shootDisk();
        pros::delay(1000);
        Gun::shootDisk();
        Intake::turnOff();

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

        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(touchDelay);
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
        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(1.5*touchDelay);
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
        Odom::set_state(39.44444444444444, 90.55555555555556, -90);
        Flywheel::setLinearEjectVelocity(8);
        Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);

        pros::delay(4000);

        Intake::turnOn();
        Gun::shootDisk();
        pros::delay(800);
        Gun::shootDisk();
        Flywheel::setLinearEjectVelocity(0);

        // Auto::simpleMoveToPointBackwards(39.44444444444444, 84.07);
        // Auto::simpleMoveToPointBackwards(39.44444444444444, 78.14);
        Auto::simpleMoveToPointBackwards(39.44444444444444, 60.55555555555556);
        pros::delay(1000);
        Intake::turnOff();
        printf("%f %f\n", positionSI.xPercent, positionSI.yPercent);
        Auto::simpleMoveToPointBackwards(24.62962962962963, 60.55555555555556);

        // Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], true);
        // Flywheel::setLinearEjectVelocity(7);
        // pros::delay(3000);
        // Intake::turnOn();
        // Gun::shootDisk();
        // pros::delay(500);
        // Gun::shootDisk();
        // pros::delay(500);
        // Gun::shootDisk();
        // pros::delay(500);
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


        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(touchDelay);
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

        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(touchDelay);
        LFMotor.moveVelocity(0);
        RFMotor.moveVelocity(0);
        LBMotor.moveVelocity(0);
        RBMotor.moveVelocity(0);
        pros::delay(1000);

        Auto::moveDistance(2, 4000);
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

        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(touchDelay);
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
        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(1.5*touchDelay);
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

        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(1.5*touchDelay);
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

        LFMotor.moveVelocity(-400);
        RFMotor.moveVelocity(-400);
        LBMotor.moveVelocity(-400);
        RBMotor.moveVelocity(-400);
        pros::delay(touchDelay);
        LFMotor.moveVelocity(0);
        RFMotor.moveVelocity(0);
        LBMotor.moveVelocity(0);
        RBMotor.moveVelocity(0);
        pros::delay(1000);
        
    }

    void run(AutoProcedure mode) {

        int round_begin_milliseconds = pros::millis();

        /*
            Blue team autonomous
            1. Scoring mode
            2. Supportive mode
            3. Idle
        */
        if (mode == BLUE_FIRST_SCORING) {
            blue_first_scoring();
        }
        else if (mode == BLUE_FIRST_SUPPORTIVE) {
            ;
        }
        else if (mode == BLUE_SECOND_SCORING) {
            blue_second_scoring();
        }
        else if (mode == BLUE_SECOND_SUPPORTIVE) {
            ;
        }
        else if (mode == RED_FIRST_SCORING) {
            red_first_scoring();
        }
        else if (mode == RED_FIRST_SUPPORTIVE) {
            ;
        }
        else if (mode == RED_SECOND_SCORING) {
            red_second_scoring();
        }
        else if (mode == RED_SECOND_SUPPORTIVE) {
            ;
        }


    }
}