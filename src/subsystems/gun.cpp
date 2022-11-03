#include "main.h"


namespace Gun {
    double Cv = 0.47; // y direction drag coefficient
    double Ch = 0.47; // x direction drag coefficient

    int diskCountForAsync = 0;
    int desiredLinearEjectVelocity = 0;
    shootMode shoot_mode;

    /**
     * @brief Initialize the shooter
     * 
     * @param mode shooting behavior
     */
    void init(shootMode mode) {
        shoot_mode = mode;
        reposition();
    }

    /**
     * @brief Reload the indexer
     * 
     */
    void reposition() {
        IndexerMotor.moveVoltage(8000); // move the indexer
        while (load_sensor.get_value() == 0) { 
            printf("%d\n", load_sensor.get_value());
            ; // wait until the indexer is loaded
        }
        IndexerMotor.moveVoltage(0);
    }

    /**
     * @brief Reload the indexer if the gear slipped
     * 
     */
    void autoPosition(){
        pros::delay(200);
        if (load_sensor.get_value() == 0) {
            reposition();
        }
    }

    /**
     * @brief Fire the indexer for 1 time
     * 
     */
    void trigger() {
        IndexerMotor.moveVoltage(8000); // move the indexer
        while (load_sensor.get_value() == 1) { 
            ; // wait until the disk is ejected
        }
        while (load_sensor.get_value() == 0) { 
            ; // wait until the indexer is loaded
        }
    }

    /**
     * @brief Check if the flywheel is running in the desired velocity
     * 
     */
    bool readyToShoot() {
        printf("%f %f\n", desiredLinearEjectVelocity, Flywheel::getCurrentEjectVelocity());
        return abs(desiredLinearEjectVelocity - Flywheel::getCurrentEjectVelocity()) <= 0.1;
        
    }

    /**
     * @brief Plan flywheel's linear eject velocity (m/s)
     * 
     * @returns flywheel's linear eject velocity (m/s) 
     */
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

    /**
     * @brief Power the flywheel to the appropriate velocity for shooting
     * 
     */
    void aim() {
        float velocity = planVeloity();
        Flywheel::setLinearEjectVelocity(velocity);
    }

    /**
     * @brief Shoot 1 disk
     * 
     */
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

    /**
     * @brief Shoot disks
     * 
     * @param diskCount number of disks to shoot
     */
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
    
    /**
     * @brief Shoot function that should only be called by shootDiskAsync()
     * 
     */
    void shootTaskCaller() {
        shootDisk(diskCountForAsync);
    }

    /**
     * @brief Shoot 1 disk asynchronously
     * 
     */
    void shootDiskAsync(){
        shootDiskAsync(1);
    }

    /**
     * @brief Shoot disks asynchronously
     * 
     * @param diskCount number of disks to shoot
     */
    void shootDiskAsync(int diskCount){
        diskCountForAsync = diskCount;
        pros::Task shootAll(shootTaskCaller);
        diskCountForAsync = 0;
    }

}