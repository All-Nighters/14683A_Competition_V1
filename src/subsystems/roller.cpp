#include "main.h"

namespace Roller {
    void roll(std::string rollerTeam) {
        if (teamColor == 0) { // red team
            if (rollerTeam == "red") {
                RollerMotor.moveVelocity(200);
            }
            else if (rollerTeam == "blue") {
                RollerMotor.moveVelocity(-200);
            }
        }
        else if (teamColor == 1) { // blue team
            if (rollerTeam == "red") {
                RollerMotor.moveVelocity(-200);
            }
            else if (rollerTeam == "blue") {
                RollerMotor.moveVelocity(200);
            }
        }
    }
}