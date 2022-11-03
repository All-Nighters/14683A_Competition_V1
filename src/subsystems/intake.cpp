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
    void turnOn() {
        enabled = true;
        IntakeMotor.moveVoltage(10000);
    }
    void turnOff() {
        enabled = false;
        IntakeMotor.moveVoltage(0);
    }
}