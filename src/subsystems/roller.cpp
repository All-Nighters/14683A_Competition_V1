#include "main.h"

namespace Roller {
    void roll(std::string rollerTeam) {
        if (team == REDTEAM) { // red team
            if (rollerTeam == "red") {
                RollerMotor.moveVelocity(-100);
            }
            else if (rollerTeam == "blue") {
                RollerMotor.moveVelocity(100);
            }
        }
        else if (team == BLUETEAM) { // blue team
            if (rollerTeam == "red") {
                RollerMotor.moveVelocity(100);
            }
            else if (rollerTeam == "blue") {
                RollerMotor.moveVelocity(-100);
            }
        }
    }
    void stop() {
        RollerMotor.moveVelocity(0);
    }
}