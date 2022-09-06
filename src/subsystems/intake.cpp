/**
 * @file intake.cpp
 *
 * @brief Intake code
 *
 */

#include "main.h"

namespace Intake {
    float enabled = false;

    void toggle() {
        enabled = !enabled;
        if (enabled) {
            IntakeMotor.moveVoltage(10000);
        } else {
            IntakeMotor.moveVoltage(0);
        }
    }
}