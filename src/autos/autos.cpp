#include "main.h"

namespace Autos {
    Coordinates minimum[] = {
        Coordinates(10.37, 26.39, 0), // start

        Coordinates(10.37, 22.59, 0), // first roller

        Coordinates(22.40, 10.46, 0), // second roller

        Coordinates(89.54, 78.42, 0), // third roller

        Coordinates(77.5, 89.54, 0), // fourth roller

    };

    void run(std::string mode) {
        if (mode == "minimum") {
            for (int i = 0; i < sizeof(minimum)/sizeof(minimum[0]); i++) {
                Coordinates c = minimum[i];
                Auto::simpleMoveToPointBackwards(c.get_x(), c.get_y());

                // if (i == 1) {
                //     Auto::turnAngle(90);
                //     LFMotor.moveVelocity(-400);
                //     RFMotor.moveVelocity(-400);
                //     LBMotor.moveVelocity(-400);
                //     RBMotor.moveVelocity(-400);
                // }
                // else if (i == 2) {
                //     Auto::turnAngle(135);
                //     LFMotor.moveVelocity(-400);
                //     RFMotor.moveVelocity(-400);
                //     LBMotor.moveVelocity(-400);
                //     RBMotor.moveVelocity(-400);
                // }
                // else if (i == 3) {
                //     Auto::turnAngle(-135);
                //     LFMotor.moveVelocity(-400);
                //     RFMotor.moveVelocity(-400);
                //     LBMotor.moveVelocity(-400);
                //     RBMotor.moveVelocity(-400);
                // }
                // else if (i == 4) {
                //     Auto::turnAngle(135);
                //     LFMotor.moveVelocity(-400);
                //     RFMotor.moveVelocity(-400);
                //     LBMotor.moveVelocity(-400);
                //     RBMotor.moveVelocity(-400);
                // }
            }
        }
    }
}