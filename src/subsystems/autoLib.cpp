#include "main.h"
#include <math.h>

namespace Auto {
    float gearRatio = (float) 36/84;
    float readingToAngleGain = (float)305 / 360;

    // traslational (position) PID Coefficients
    float Tp = 15;
    float Ti = 0;
    float Td = 40;

    // rotational PID Coefficients
    float Rp = 170;
    float Ri = 0;
    float Rd = 300;


    bool settled = true;

    float target_percentage_for_async; // target_distance in meters only for usage in moveDistanceOnlyAsync()
    float ang_for_async; // angle in degrees only for usage in turnAngleOnlyAsync()
    float xPercent_for_async;
    float yPercent_for_async;
    float aimMode_for_async;

    /**
     * @brief PID (nonblocking) for controlling velocity
     * 
     * @param leftV left side target velocity in RPM
     * @param rightV right side target velocity in RPM
     */
    void trackVelocityPID(float leftV, float rightV) {
        LFMotor.moveVelocity(leftV);
        RFMotor.moveVelocity(rightV);
        LBMotor.moveVelocity(leftV);
        RBMotor.moveVelocity(rightV);
    }
    /**
     * @brief PID for controlling forward distance
     * 
     * @param leftTW left tracking wheel encoder
     * @param rightTW right tracking wheel encoder
     * @param target_distance target distance
     */
    void distancePID(float percentage) {
        
        float target_distance = fieldLength * percentage / 100;
        float revs = target_distance / (M_PI*(trackingWheelDiameter.convert(meter)));
        float lefttargetAngle = revs * 360 + leftTW.get();
        float righttargetAngle = revs * 360 + rightTW.get();
        float targetFaceAngle = positionSI.theta;        

        int direction;

        if (revs * 360  < 0) {
            direction = -1;
        } else {
            direction = 1;
        }
        
        float prevErrorLeft = abs(lefttargetAngle - leftTW.get());
        float prevErrorRight = abs(righttargetAngle - rightTW.get());
        float prevFaceAngleError = 0;


        settled = false;

        while (abs(((lefttargetAngle + righttargetAngle)/2.0) - ((leftTW.get() + rightTW.get())/2.0)) >= 10) {

            Odom::update_odometry();
            // printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);

            float error_Left = abs(lefttargetAngle - leftTW.get());
            float error_Right = abs(righttargetAngle - rightTW.get());
            float error_Facing = positionSI.theta - targetFaceAngle;


            float deriv_Left = error_Left - prevErrorLeft;
            float deriv_Right = error_Right - prevErrorRight;
            float deriv_Facing = error_Facing - prevFaceAngleError;


            float control_output_Left = error_Left * Tp + deriv_Left * Td;
            float control_output_Right = error_Right * Tp + deriv_Right * Td;
            float control_output_Facing = error_Facing * Rp + deriv_Facing * Rd;


            control_output_Left = direction * std::fmax(std::fmin(control_output_Left, 8000), -8000) - std::fmax(std::fmin(control_output_Facing, 4000), -4000);
            control_output_Right = direction * std::fmax(std::fmin(control_output_Right, 8000), -8000) + std::fmax(std::fmin(control_output_Facing, 4000), -4000);


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
    void moveDistance(float percentage) {
        
        distancePID(percentage);
    }

    /**
     * @brief PID controlling robot's rotation
     * 
     * @param ang angle to the right
     */
    
    void directionPID(float angle) {
        float target_angle = positionSI.theta + angle;
        float prev_error = abs(angle);

        settled = false;
        // printf("%f %f %f %f\n", target_angle, positionSI.x, positionSI.y, positionSI.theta);
        // Odom::update_odometry();
        // printf("%f %f %f %f\n", target_angle, positionSI.x, positionSI.y, positionSI.theta);
        while (abs(target_angle - positionSI.theta) >= 1) {
            Odom::update_odometry();
            // printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);

            float error = abs(target_angle - positionSI.theta);

            float deriv_error = error - prev_error;

            // printf("%f %f\n", error, deriv_error);

            float control_output = error * Rp + deriv_error * Rd;

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
     * @param angle angle to the right
     */
    void turnAngle(float angle) {
        directionPID(angle);
    }


    /**
     * @brief face the robot to a specific coordinate in percent
     * 
     * @param xPercent x coordinate of the target
     * @param yPercent y coordinate of the target
     * @param aimMode whether the robot is aiming the goal
     */
    void faceCoordinate(float xPercent, float yPercent, bool aimMode) {
        settled = false;
        Odom::update_odometry();
        float xDist = xPercent - positionSI.xPercent;
        float yDist = yPercent - positionSI.yPercent;
        float dist = sqrt(xDist*xDist + yDist*yDist);

        float relativeAngle;

        if (xDist > 0 && yDist > 0) { // first quadrant
            relativeAngle = atan(abs(yDist/xDist)) * 180 / M_PI;
        }
        else if (xDist > 0 && yDist < 0) { // second quadrant
            relativeAngle = -atan(abs(yDist/xDist)) * 180 / M_PI;
        }
        else if (xDist < 0 && yDist < 0) { // third quadrant
            relativeAngle = -180 + (atan(abs(yDist/xDist)) * 180 / M_PI);
        }
        else if (xDist < 0 && yDist > 0) { // fourth quadrant
            relativeAngle = 180 - (atan(abs(yDist/xDist)) * 180 / M_PI);
        }
        else if (xDist == 0 && yDist != 0) {
            relativeAngle = (yDist / abs(yDist))*90;
        }
        else if (xDist != 0 && yDist == 0) {
            relativeAngle = 0;
        } else {
            settled = true;
            return;
        }

        float faceAngle;

        if (aimMode) {
            faceAngle = formatAngle((relativeAngle - positionSI.theta) + aimAngleDeviation);
        } else {
            faceAngle = formatAngle(relativeAngle - positionSI.theta);
        }

        turnAngle(faceAngle);
        settled = true;
    }

    /**
     * @brief face position which only intended to respond from the call from faceCoordinateAsync()
     * 
     */

    void faceCoordinateOnlyAsync() {
        faceCoordinate(xPercent_for_async, yPercent_for_async, aimMode_for_async);
        xPercent_for_async = 0;
        yPercent_for_async = 0;
    }
    
    /**
     * @brief creates task to face position asynchronously
     * 
     */
    
    void faceCoordinateAsync(float xPercent, float yPercent, bool aimMode) {
        settled = false;
        xPercent_for_async = xPercent;
        yPercent_for_async = yPercent;
        aimMode_for_async = aimMode;
        pros::Task move(faceCoordinateOnlyAsync);
    }

    void simpleMoveToPoint(float xPercent, float yPercent) {
        float xDist = xPercent - positionSI.xPercent;
        float yDist = yPercent - positionSI.yPercent;
        float dist = sqrt(xDist*xDist + yDist*yDist);

        faceCoordinate(xPercent, yPercent, false);
        moveDistance(dist);

    }

    void simpleMoveToPointBackwards(float xPercent, float yPercent) {
        float xDist = xPercent - positionSI.xPercent;
        float yDist = yPercent - positionSI.yPercent;
        float dist = sqrt(xDist*xDist + yDist*yDist);

        faceCoordinate(-xPercent, -yPercent, false);
        moveDistance(-dist);

    }

    /**
     * @brief move distance which only intended to respond from the call from moveDistanceAsync()
     * 
     */
    void moveDistanceOnlyAsync() {

        distancePID(target_percentage_for_async);
        target_percentage_for_async = 0;
        
    }

    /**
     * @brief turn angle which only intended to respond from the call from turnAngleAsync()
     * 
     */
    void turnAngleOnlyAsync() {
        directionPID(ang_for_async);
        ang_for_async = 0;
    }


    /**
     * @brief creates task to move distance asynchronously
     * 
     */
    void moveDistanceAsync(float target_percentage) {
        target_percentage_for_async = target_percentage;
        settled = false;
        pros::Task move(moveDistanceOnlyAsync);
    }

    /**
     * @brief creates task to turn asynchronously
     * 
     */
    void turnAngleAsync(float ang) {
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