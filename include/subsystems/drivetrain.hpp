#include "main.h"

namespace Drivetrain {
    std::shared_ptr<ChassisController> get_drive_model();
    void tarePosition();
    float getLeftPosition();
    float getRightPosition();
    void moveVelocity(float velocity);
    void moveVelocity(float leftV, float rightV);
}