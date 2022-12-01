#include "main.h"

namespace Drivetrain {

    // drive model
    std::shared_ptr<ChassisController> drive =
    ChassisControllerBuilder()
        .withMotors(
            {frontLeftMotorPort, bottomLeftMotorPort},
            {frontRightMotorPort, bottomRightMotorPort}
        )
        .withDimensions(chassis_motor_gearset, {{wheelDiameter, wheeltrackLength}, imev5BlueTPR})
        .withGains(
            {0.001, 0, 0.00001}, // Distance controller gains
            {0.008, 0, 0.0001}, // Turn controller gains
            {0.001, 0, 0.00001}  // Angle controller gains (helps drive straight)
        )
        .build();

    /**
     * @brief Get the drive model object
     * 
     * @return drive model object
     */
    std::shared_ptr<ChassisController> get_drive_model() {
        return drive;
    }

    /**
     * @brief Set brake mode for drivetrain
     * 
     * @param brake_mode brake mode
     */
    void setBrakeMode(okapi::AbstractMotor::brakeMode brake_mode) {
        LFMotor.setBrakeMode(brake_mode);
        RFMotor.setBrakeMode(brake_mode);
        RBMotor.setBrakeMode(brake_mode);
        LBMotor.setBrakeMode(brake_mode);
    }
    /**
     * @brief Tare motor position readings
     * 
     */
    void tarePosition() {
        LFMotor.tarePosition();
        RFMotor.tarePosition();
        LBMotor.tarePosition();
        RBMotor.tarePosition();
    }
    /**
     * @brief Get the left position
     * 
     * @return left track velocity.
     */
    float getLeftPosition() {
        return ((LFMotor.getPosition() + LBMotor.getPosition()) / 2.0) * external_gear_ratio;
    }

    /**
     * @brief Get the right position
     * 
     * @return right track velocity.
     */
    float getRightPosition() {
        return ((RFMotor.getPosition() + RBMotor.getPosition()) / 2.0) * external_gear_ratio;
    }

    /**
     * @brief Move the drivetrain forward with specific velocity
     * 
     * @param velocity 
     */
    void moveVelocity(float velocity) {
        LFMotor.moveVelocity(velocity);
        RFMotor.moveVelocity(velocity);
        LBMotor.moveVelocity(velocity);
        RBMotor.moveVelocity(velocity);
    }

    /**
     * @brief Move the drivetrain forward with specific velocity
     * 
     * @param leftV left track target velocity
     * @param rightV right track target velocity
     */
    void moveVelocity(float leftV, float rightV) {
        LFMotor.moveVelocity(leftV);
        RFMotor.moveVelocity(rightV);
        LBMotor.moveVelocity(leftV);
        RBMotor.moveVelocity(rightV);
    }

    /**
     * @brief Move the drivetrain forward with specific voltage
     * 
     * @param voltage voltage
     */
    void moveVoltage(float voltage) {
        LFMotor.moveVoltage(voltage);
        RFMotor.moveVoltage(voltage);
        LBMotor.moveVoltage(voltage);
        RBMotor.moveVoltage(voltage);
    }

    /**
     * @brief Move the drivetrain forward with specific voltage
     * 
     * @param left_volt left voltage
     * @param right_volt right voltage
     */
    void moveVoltage(float left_volt, float right_volt) {
        LFMotor.moveVoltage(left_volt);
        RFMotor.moveVoltage(right_volt);
        LBMotor.moveVoltage(left_volt);
        RBMotor.moveVoltage(right_volt);
    }
}