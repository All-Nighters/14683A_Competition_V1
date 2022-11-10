#include "main.h"

namespace Intake {
    float enabled = false;
    int diskHolding = 0;

    int getDiskHolding() {
        return diskHolding;  
    }
    void setDiskHolding(int disks) {
        if (disks >= 0) {
            diskHolding = disks;
        }
    }
    void toggle() {
        enabled = !enabled;
        if (enabled) {
            IntakeMotor.moveVoltage(10000);
        } else {
            IntakeMotor.moveVoltage(0);
        }
    }

    int desired_disks = 0; // only used in takeDiskHelper()

    void takeDiskHelper() {
        turnOn();
        
        int prev_sensor_value = 0;
        while (diskHolding < desired_disks) {
            int current_sensor_value = intake_sensor.get_value();
            if (current_sensor_value != prev_sensor_value && current_sensor_value == 1) {
                diskHolding++;
            }
            prev_sensor_value = current_sensor_value;
            pros::delay(20);
        }
        turnOff();
    }

    void takeDisk(int disks) {
        desired_disks = disks;
        pros::Task take(takeDiskHelper);
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