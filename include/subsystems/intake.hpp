/**
 * @file intake.hpp
 *
 * @brief Intake code
 *
 */

namespace Intake {
    int getDiskHolding();
    bool is_enabled();
    void setDiskHolding(int disks);
    void toggle();
    void turnOn();
    void turnOnRev();
    void turnOff();
    void takeDisk(int disks);
}