#include "main.h"

namespace Drivetrain {
    std::shared_ptr<ChassisController> get_drive_model();
    void setBrakeMode(okapi::AbstractMotor::brakeMode brake_mode);
    void tarePosition();
    float getLeftPosition();
    float getRightPosition();
    void moveVelocity(float velocity);
    void moveVelocity(float leftV, float rightV);
    void moveVoltage(float voltage);
    void moveVoltage(float left_volt, float right_volt);
}