#include "main.h"

namespace Drivetrain {
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
}