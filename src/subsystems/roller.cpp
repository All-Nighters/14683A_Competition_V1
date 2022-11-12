#include "main.h"

namespace Roller {
    bool basic_mode = true;

    void roll(std::string rollerTeam) {
        printf("Rolling in basic mode?: %d", basic_mode);
        int velocity = 0;
        int color_locations[2];
        color_locations[0] = -1;
        color_locations[1] = -1;

        if (team == REDTEAM) { // red team
            if (rollerTeam == "red") {
                velocity = -100;
            }
            else if (rollerTeam == "blue") {
                velocity = 100;
            }

            RollerMotor.moveVelocity(velocity); // move the roller motor

            // push the robot on the roller
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);
            
            /*
            If using time based mode, run for a given interval and exit
            */
            if (basic_mode) {
                pros::delay(300);

                LFMotor.moveVelocity(0);
                RFMotor.moveVelocity(0);
                LBMotor.moveVelocity(0);
                RBMotor.moveVelocity(0);

                velocity = 0;
                RollerMotor.moveVelocity(velocity);
                return;
            }

            while (true) {
                int size_index = -1;
                while (true) {
                    size_index++;
                    auto object = vision_sensor.get_by_size(size_index);

                    if (object.width < 50 || object.signature == 255) break;
                    if (color_locations[object.signature - 1] > 0) continue;
                    color_locations[object.signature - 1] = object.top_coord;
                }
                
                if (color_locations[0] >= 0 && color_locations[1] >= 0) { // detected two colors
                    printf("Detected two colors\n");
                    if (color_locations[0] < color_locations[1]) { // red above blue
                        printf("Red above blue\n");
                        break;
                    }
                }
                pros::delay(10);
            }

        }
        else if (team == BLUETEAM) { // blue team
            if (rollerTeam == "red") {
                velocity = 100;
            }
            else if (rollerTeam == "blue") {
                velocity = -100;
            }

            RollerMotor.moveVelocity(velocity); // move the roller motor

            // push the robot on the roller
            LFMotor.moveVelocity(-400);
            RFMotor.moveVelocity(-400);
            LBMotor.moveVelocity(-400);
            RBMotor.moveVelocity(-400);

            /*
            If using time based mode, run for a given interval and exit
            */
            if (basic_mode) {
                pros::delay(300);

                LFMotor.moveVelocity(0);
                RFMotor.moveVelocity(0);
                LBMotor.moveVelocity(0);
                RBMotor.moveVelocity(0);

                velocity = 0;
                RollerMotor.moveVelocity(velocity);
                return;
            }

            while (true) {
                int size_index = -1;
                while (true) {
                    size_index++;
                    auto object = vision_sensor.get_by_size(size_index);

                    if (object.width < 50 || object.signature == 255) break;
                    if (color_locations[object.signature - 1] > 0) continue;
                    color_locations[object.signature - 1] = object.top_coord;
                }
                
                if (color_locations[0] >= 0 && color_locations[1] >= 0) { // detected two colors
                    printf("Detected two colors\n");
                    if (color_locations[1] < color_locations[0]) { // blue above red
                        printf("Blue above red\n");
                        break;
                    }
                }
                pros::delay(10);
            }
        }

        LFMotor.moveVelocity(0);
        RFMotor.moveVelocity(0);
        LBMotor.moveVelocity(0);
        RBMotor.moveVelocity(0);

        velocity = 0;
        RollerMotor.moveVelocity(velocity);
        printf("Finished rolling\n");
    }
    void stop() {
        RollerMotor.moveVelocity(0);
    }
}