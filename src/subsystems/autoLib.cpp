#include "main.h"
#include <math.h>

namespace Auto {
    // traslational (position) PID Coefficients
    float Tp = 15;
    float Ti = 0;
    float Td = 100;

    // facing PID Coefficients (keeping robot straight)
    float Fp = 100;
    float Fi = 0;
    float Fd = 300;

    // rotational PID Coefficients
    float Rp = 200;
    float Ri = 0;
    float Rd = 600;


    bool settled = true;

    float target_percentage_for_async; // target_distance in meters only for usage in moveDistanceOnlyAsync()
    float ang_for_async; // angle in degrees only for usage in turnAngleOnlyAsync()
    float xPercent_for_async;
    float yPercent_for_async;
    float aimMode_for_async;

    /**
     * @brief Get the degree of rotations forward
     * 
     * @return degree of rotation
     */
    float getMotorPosition() {
        if (Odom::getOdomMode() == MOTOR_IMU) {
            float right_track_velocity = Drivetrain::getRightPosition();
            return right_track_velocity;
        } 
        else if (Odom::getOdomMode() == RIGHTTW_IMU) {
            return rightTW.get();
        }
        else if (Odom::getOdomMode() == LEFTTW_IMU || Odom::getOdomMode() == THREEWHEEL) {
            // return leftTW.get();
            return 0;
        } else {
            return -1;
        }
    }
    /**
     * @brief PID for controlling forward distance
     * 
     * @param percentage forward distance in percentage unit
     * @param max_voltage maximum voltage for forward movement
     */
    void distancePID(float percentage, float max_voltage) {
        float target_distance = fieldLength * percentage / 100;
        float revs;
        if (Odom::getOdomMode() == MOTOR_IMU) {
            revs = target_distance / (M_PI*(wheelDiameter.convert(meter))); // # of revolutions of wheels
        } else {
            revs = target_distance / (M_PI*(trackingWheelDiameter.convert(meter))); // # of revolutions of tracking wheels
        }

        float targetAngle = revs * 360 + getMotorPosition();

        float targetFaceAngle = (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2; 
        float start_time = pros::millis();  
        float timeout = 10; // maximum runtime in seconds

        int direction;

        if (revs < 0) {
            direction = -1;
        } else {
            direction = 1;
        }
        
        float prevErrorPosition = abs(targetAngle - getMotorPosition());
        float prevFaceAngleError = 0;


        settled = false;


        while (abs(targetAngle - getMotorPosition()) >= 10 && 
        pros::millis() - start_time <= timeout*1000) {

            float error_position = abs(targetAngle - getMotorPosition());

            prevErrorPosition = abs(targetAngle - getMotorPosition());
            float error_Facing = targetFaceAngle- ((imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2);


            float deriv_position = error_position - prevErrorPosition;
            float deriv_Facing = error_Facing - prevFaceAngleError;


            float control_output = error_position * Tp + deriv_position * Td;
            float control_output_Facing = error_Facing * Fp + deriv_Facing * Fd;

            float control_output_Left = direction * std::fmax(std::fmin(control_output, max_voltage), -max_voltage) + std::fmax(std::fmin(control_output_Facing, max_voltage * 0.25), -max_voltage * 0.25);
            float control_output_Right = direction * std::fmax(std::fmin(control_output, max_voltage), -max_voltage) - std::fmax(std::fmin(control_output_Facing, max_voltage * 0.25), -max_voltage * 0.25);

            if (abs(control_output_Left) < 2000 && direction < 0) {
                control_output_Left = -2000;
                control_output_Right = -2000;
            }
            else if (abs(control_output_Left) < 2000 && direction > 0) {
                control_output_Left = 2000;
                control_output_Right = 2000;
            }


            prevErrorPosition = error_position;


            Drivetrain::moveVoltage(control_output_Left, control_output_Right);

            pros::delay(20);
        }
        Drivetrain::moveVoltage(0);

        settled = true;
    }
    /**
     * @brief move the robot forward with a specific distance
     * 
     * @param percentage percentage forward distance in percentage unit
     * @param max_voltage maximum voltage for forward movement
     */
    void moveDistance(float percentage, float max_voltage) {
        
        distancePID(percentage, max_voltage);
    }

    /**
     * @brief PID controlling robot's rotation
     * 
     * @param ang angle to the right
     */
    
    void directionPID(float angle) {
        float target_angle = positionSI.theta + angle;
        float prev_error = abs(angle);
        float start_time = pros::millis();  
        float timeout = 10; // maximum runtime in seconds

        settled = false;

        while (abs(target_angle - positionSI.theta) >= 1 && pros::millis() - start_time <= timeout*1000) {
            
            float error = target_angle - positionSI.theta;
            float deriv_error = error - prev_error;

            float control_output = clamp(error * Rp + deriv_error * Rd, -12000, 12000);

            if (abs(control_output) < 2000) {
                control_output = control_output > 0 ? 2000 : -2000;
            }

            prev_error = error;

            Drivetrain::moveVoltage(control_output, -control_output);

            pros::delay(20);
        }

        Drivetrain::moveVoltage(0);
        
        settled = true;
    }

    /**
     * @brief PID controlling robot's rotation (a step only)
     * 
     * @param target_angle targeted angle
     * @param prev_error previous angle error
     * 
     * @returns previous direction error
     */
    float directionPIDStep(float target_angle, float prev_error) {
        float error = abs(target_angle - positionSI.theta);
        float deriv_error = error - prev_error;

        float control_output = std::fmax(error * Rp + deriv_error * Rd, 2000);


        prev_error = error;

        if (target_angle - positionSI.theta > 0) {
            Drivetrain::moveVoltage(control_output, -control_output);
        } else {
            Drivetrain::moveVoltage(-control_output, control_output);
        }
        return prev_error;
    }
    /**
     * @brief PID controlling robot's absolute direction
     * 
     * @param ang disired absolute angle facing
     */
    
    void directionPIDAbs(float angle) {
        float target_angle = angle;
        float prev_error = abs(formatAngle(target_angle) - formatAngle(positionSI.theta));
        float start_time = pros::millis();  
        float timeout = 1000; // maximum runtime in seconds

        settled = false;

        while (abs(formatAngle(target_angle) - formatAngle(positionSI.theta)) >= 0.5 && pros::millis() - start_time <= timeout*1000) {

            float error = abs(formatAngle(target_angle) - formatAngle(positionSI.theta));

            float deriv_error = error - prev_error;

            float control_output = clamp(error * Rp + deriv_error * Rd, -12000, 12000);

            if (abs(control_output) < 2000) {
                control_output = control_output > 0 ? 2000 : -2000;
            }

            Drivetrain::moveVoltage(control_output, -control_output);
            prev_error = error;

            pros::delay(20);
        }

        Drivetrain::moveVoltage(0);
        pros::delay(200);
        
        settled = true;
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
     * @brief face the robot with a specific angle
     * 
     * @param angle angle to the right
     */
    void faceAngle(float angle) {
        directionPIDAbs(angle);
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
        float xDist = xPercent - positionSI.xPercent;
        float yDist = yPercent - positionSI.yPercent;

        if (abs(xDist) < 0.1 && abs(yDist) < 0.1) {
            return;
        }

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
     * @param x x coorinate in percentage of the target
     * @param y y coorinate in percentage of the target
     * @param aimMode whether the robot is aiming to shoot
     */
    
    void faceCoordinateAsync(float xPercent, float yPercent, bool aimMode) {
        settled = false;
        xPercent_for_async = xPercent;
        yPercent_for_async = yPercent;
        aimMode_for_async = aimMode;
        pros::Task move(faceCoordinateOnlyAsync);
    }

    /**
     * Move the robot to a specific coordinate in the field. The function may be inaccurate over long distances.
     * 
     * @param xPercent x coordinate of the target
     * @param yPercent y coordinate of the target
     */
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

        faceCoordinate(positionSI.xPercent - xDist, positionSI.yPercent - yDist, false);
        if (Intake::is_enabled()) {
            moveDistance(-dist, 3000);
        } else {
            moveDistance(-dist);
        }
        

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
     * @param target_percentage forward distance in percentage unit
     */
    void moveDistanceAsync(float target_percentage) {
        target_percentage_for_async = target_percentage;
        settled = false;
        pros::Task move(moveDistanceOnlyAsync);
    }

    /**
     * @brief creates task to turn asynchronously
     * 
     * @param angle angle to the right
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