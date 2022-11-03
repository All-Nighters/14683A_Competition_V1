#include "main.h"


namespace Gun {
    double Cv = 0.47; // y direction drag coefficient
    double Ch = 0.47; // x direction drag coefficient

    int diskCountForAsync = 0;
    int desiredLinearEjectVelocity = 0;
    shootMode shoot_mode;

    void init(shootMode mode) {
        printf("INIT\n");
        shoot_mode = mode;
        IndexerMotor.moveVoltage(8000); // move the indexer
        while (load_sensor.get_value() == 0) { 
            printf("%d\n", load_sensor.get_value());
            ; // wait until the indexer is loaded
        }
        IndexerMotor.moveVoltage(0);
    }

    float planVeloity() {
        float target[3];
        if (team == REDTEAM) {
            for (int i = 0; i < 3; i++) {
                target[i] = redHighGoalPosition_percent[i];
            }
        }
        else if (team == BLUETEAM) {
            for (int i = 0; i < 3; i++) {
                target[i] = blueHighGoalPosition_percent[i];
            }
        }

        float xDist = sqrt(std::pow(target[0]-positionSI.xPercent, 2) + std::pow(target[1]-positionSI.yPercent, 2));
        
        desiredLinearEjectVelocity = projectile_trajectory::solveVelocity(100, 0, 0.01, 20, 4, launch_angle.convert(degree), 0, diskMass.convert(kg), 9.81, 1.225, diskHorizontalArea, diskVerticalArea, Cv, Ch, percentageToMeter(target[2]), launcher_height.convert(meter));
        return desiredLinearEjectVelocity;
    }

    void aim() {
        float velocity = planVeloity();
        // printf("%f\n",velocity);
        Flywheel::setLinearEjectVelocity(velocity);
    }

    void autoPosition(){
        pros::delay(200);
        if (load_sensor.get_value() == 0) {
            init(shoot_mode);
        }
    }

    void trigger() {
        IndexerMotor.moveVoltage(8000); // move the indexer
        while (load_sensor.get_value() == 1) { 
            ; // wait until the disk is ejected
        }
        while (load_sensor.get_value() == 0) { 
            ; // wait until the indexer is loaded
        }
    }

    bool readyToShoot() {
        printf("%f %f\n", desiredLinearEjectVelocity, Flywheel::getCurrentEjectVelocity());
        return abs(desiredLinearEjectVelocity - Flywheel::getCurrentEjectVelocity()) <= 0.1;
        
    }

    void shootDisk() {
        if (shoot_mode == ACCURATE_MODE) {
            planVeloity();
            while (!readyToShoot()) {
                Flywheel::grapher::graph_velocity(3000, Flywheel::getCurrentVelocity());
                aim();
                pros::delay(20);
            }
            trigger();
            FlywheelMotor1.moveVoltage(0);
            FlywheelMotor2.moveVoltage(0);
        } 
        else if (shoot_mode == FORCE_MODE) {
            trigger();
        }
        IndexerMotor.moveVoltage(0);
        
        autoPosition();
    }

    void shootDisk(int diskCount) {
        if (shoot_mode == ACCURATE_MODE) {
            for (int i = 0; i < diskCount; i++) {
                while (!readyToShoot()) {
                    aim();
                }
                trigger();
            }
            
        } 
        else if (shoot_mode == FORCE_MODE) {
            for (int i = 0; i < diskCount; i++) {
                trigger();
            }
        }

        
        IndexerMotor.moveVoltage(0);
        autoPosition();
    }
    
    void shootTaskCaller() {
        shootDisk(diskCountForAsync);
    }

    void shootDiskAsync(){
        shootDiskAsync(1);
    }

    void shootDiskAsync(int diskCount){
        diskCountForAsync = diskCount;
        pros::Task shootAll(shootTaskCaller);
        diskCountForAsync = 0;
    }

}