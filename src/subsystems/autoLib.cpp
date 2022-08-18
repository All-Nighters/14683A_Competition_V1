#include "main.h"
#include <math.h>

namespace Auto {
    double gearRatio = (double) 36/84;
    double readingToAngleGain = (double)305 / 360;

    // traslational PID Coefficients
    double Tp = 15;
    double Ti = 0;
    double Td = 40;

    // rotational PID Coefficients
    double Rp = 170;
    double Ri = 0;
    double Rd = 300;

    bool settled = true;

    QLength target_distance_for_async; // target_distance only for usage in moveDistanceOnlyAsync()
    QAngle ang_for_async; // ang only for usage in turnAngleOnlyAsync()

    std::shared_ptr<ChassisController> drive =
            ChassisControllerBuilder()
                .withMotors(
                    frontLeftMotorPort,  // Top left
                    frontRightMotorPort, // Top right (reversed)
                    bottomRightMotorPort, // Bottom right (reversed)
                    bottomLeftMotorPort   // Bottom left
                )
                // Green gearset, 4 in wheel diam, 11.5 in wheel track
                .withDimensions({AbstractMotor::gearset::blue, (84.0 / 36.0)}, {{wheelDiameter, wheeltrackLength}, imev5BlueTPR})
                .withMaxVelocity(maximum_velocity)
                .withSensors(
                    ADIEncoder{leftEncoderPort[0], leftEncoderPort[1]}, // Left encoder in ADI ports A & B (reversed)
                    ADIEncoder{rightEncoderPort[0], rightEncoderPort[1]}  // Right encoder in ADI ports C & D
                )
                .withOdometry({{trackingWheelDiameter, wheeltrackLength}, quadEncoderTPR})
                .withGains(
                    {distancePIDCoefficient[0], distancePIDCoefficient[1], distancePIDCoefficient[2]}, // Distance controller gains
                    {turnPIDCoefficient[0], turnPIDCoefficient[1], turnPIDCoefficient[2]}, // Turn controller gains
                    {anglePIDCoefficient[0], anglePIDCoefficient[1], anglePIDCoefficient[2]} // Angle controller gains (helps drive straight)
                )
                .buildOdometry();

    void test() {
        while (true) {
            Odom::update_odometry();
            // printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);
            pros::delay(20);
        }
    }
    /**
     * @brief PID for controlling forward distance
     * 
     * @param leftTW left tracking wheel encoder
     * @param rightTW right tracking wheel encoder
     * @param target_distance target distance
     */
    void distancePID(QLength target_distance) {
        
        double target_distance_in_meters = target_distance.convert(meter);
        double revs = target_distance_in_meters / (M_PI*(trackingWheelDiameter.convert(meter)));
        double lefttargetAngle = revs * 360 + leftTW.get();
        double righttargetAngle = revs * 360 + rightTW.get();
        double targetFaceAngle = positionSI.theta;        

        int direction;

        if (revs * 360  < 0) {
            direction = -1;
        } else {
            direction = 1;
        }
        
        double prevErrorLeft = abs(lefttargetAngle - leftTW.get());
        double prevErrorRight = abs(righttargetAngle - rightTW.get());
        double prevFaceAngleError = 0;


        settled = false;

        while (abs(((lefttargetAngle + righttargetAngle)/2.0) - ((leftTW.get() + rightTW.get())/2.0)) >= 10) {

            Odom::update_odometry();
            // printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);

            double error_Left = abs(lefttargetAngle - leftTW.get());
            double error_Right = abs(righttargetAngle - rightTW.get());
            double error_Facing = positionSI.theta - targetFaceAngle;


            double deriv_Left = error_Left - prevErrorLeft;
            double deriv_Right = error_Right - prevErrorRight;
            double deriv_Facing = error_Facing - prevFaceAngleError;


            double control_output_Left = error_Left * Tp + deriv_Left * Td;
            double control_output_Right = error_Right * Tp + deriv_Right * Td;
            double control_output_Facing = error_Facing * Rp + deriv_Facing * Rd;


            control_output_Left = direction * std::fmax(std::fmin(control_output_Left - control_output_Facing, 12000), 1000);
            control_output_Right = direction * std::fmax(std::fmin(control_output_Right + control_output_Facing, 12000), 1000);


            prevErrorLeft = error_Left;
            prevErrorRight = error_Right;


            LFMotor.moveVoltage(control_output_Left);
            RFMotor.moveVoltage(control_output_Right);
            LBMotor.moveVoltage(control_output_Left);
            RBMotor.moveVoltage(control_output_Right);

            
            pros::delay(20);
        }
        // printf("Angle reached\n");
        LFMotor.moveVoltage(0);
        RFMotor.moveVoltage(0);
        LBMotor.moveVoltage(0);
        RBMotor.moveVoltage(0);

        settled = true;
    }
    /**
     * @brief move the robot forward with a specific distance
     * 
     */
    void moveDistance(QLength target_distance) {
        
        distancePID(target_distance);
    }

    /**
     * @brief PID controlling robot's rotation
     * 
     * @param ang angle to the right
     */
    
    void directionPID(QAngle ang) {
        double angle = ang.convert(degree);
        double target_angle = positionSI.theta + angle;
        double prev_error = abs(angle);

        settled = false;
        // printf("%f %f %f %f\n", target_angle, positionSI.x, positionSI.y, positionSI.theta);
        // Odom::update_odometry();
        // printf("%f %f %f %f\n", target_angle, positionSI.x, positionSI.y, positionSI.theta);
        while (abs(target_angle - positionSI.theta) >= 1) {
            Odom::update_odometry();
            // printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);

            double error = abs(target_angle - positionSI.theta);

            double deriv_error = error - prev_error;

            // printf("%f %f\n", error, deriv_error);

            double control_output = error * Rp + deriv_error * Rd;

            control_output = std::fmax(std::fmin(control_output, 12000), 2000);

            prev_error = error;

            if (target_angle - positionSI.theta > 0) {
                LFMotor.moveVoltage(control_output);
                LBMotor.moveVoltage(control_output);
                RFMotor.moveVoltage(-control_output);
                RBMotor.moveVoltage(-control_output);
            } else {
                LFMotor.moveVoltage(-control_output);
                LBMotor.moveVoltage(-control_output);
                RFMotor.moveVoltage(control_output);
                RBMotor.moveVoltage(control_output);
            }


            pros::delay(20);
        }
        printf("Angle reached\n");

        LFMotor.moveVoltage(0);
        RFMotor.moveVoltage(0);
        LBMotor.moveVoltage(0);
        RBMotor.moveVoltage(0);
        
        settled = true;

        Odom::setState(position.x, position.y, position.theta);
    }

    /**
     * @brief rotate the robot with a specific angle
     * 
     * @param ang angle to the right
     */
    void turnAngle(QAngle ang) {
        directionPID(ang);
    }

    /**
     * @brief move distance which only intended to respond from the call from moveDistanceAsync()
     * 
     */
    void moveDistanceOnlyAsync() {

        distancePID(target_distance_for_async);
        target_distance_for_async = 0_m;
        
    }

    /**
     * @brief turn angle which only intended to respond from the call from turnAngleAsync()
     * 
     */
    void turnAngleOnlyAsync() {
        directionPID(ang_for_async);
        ang_for_async = 0_deg;
    }


    /**
     * @brief creates task to move distance asynchronously
     * 
     */
    void moveDistanceAsync(QLength target_distance) {
        target_distance_for_async = target_distance;
        settled = false;
        pros::Task move(moveDistanceOnlyAsync);
    }

    /**
     * @brief creates task to turn asynchronously
     * 
     */
    void turnAngleAsync(QAngle ang) {
        ang_for_async = ang;
        settled = false;
        pros::Task move(turnAngleOnlyAsync);
    }

    /**
     * @brief wait until the robot is settled
     * 
     */
    void waitUntilSettled() {
        while(!settled) {
            pros::delay(20);
        }
    }
}