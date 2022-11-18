#include "main.h"


namespace Gun {

    int diskCountForAsync = 0;
    int desiredLinearEjectVelocity = 0;
    shootMode shoot_mode = ACCURATE_MODE;

    /**
     * @brief Initialize the shooter
     * 
     * @param mode shooting behavior
     */
    void init(shootMode mode) {
        shoot_mode = mode;
        pros::delay(200);
        reposition();
    }

    /**
     * @brief Reload the indexer
     * 
     */
    void reposition() {
        if (load_sensor.get_value() == 0) {
            IndexerMotor.moveVoltage(7000); // move the indexer
            while (load_sensor.get_value() == 0) { 
                ; // wait until the indexer is loaded
            }
            IndexerMotor.moveVoltage(0);
        }
    }

    /**
     * @brief Reload the indexer if the gear slipped
     * 
     */
    void autoPosition(){
        int delay_ms = 150;
        pros::delay(delay_ms);
        while (load_sensor.get_value() == 0) {
            reposition();
            pros::delay(delay_ms);
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
        Intake::setDiskHolding(Intake::getDiskHolding()-1);
    }

    /**
     * @brief Check if the flywheel is running in the desired velocity
     * 
     */
    bool readyToShoot() {
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
                target[i] = redHighGoalPosition_m[i];
            }
        }
        else if (team == BLUETEAM) {
            for (int i = 0; i < 3; i++) {
                target[i] = blueHighGoalPosition_m[i];
            }
        }

        float xDist = sqrt(std::pow(target[0]-positionSI.x, 2) + std::pow(target[1]-positionSI.y, 2));
        
        // constants could be modified in globals.hpp
        return (projectile_trajectory::solveVelocity(maxEjectVel, minEjectVel, 0.01, 20, xDist, launch_angle.convert(degree), 0, diskMass.convert(kg), g, p, diskHorizontalArea, diskVerticalArea, Cv, Ch, target[2], launcher_height.convert(meter))) / velocityLossConstant;
    }

    /**
     * @brief Power the flywheel to the appropriate velocity for shooting
     * 
     */
    void aim() {
        float velocity = planVeloity();
        desiredLinearEjectVelocity = velocity;
        printf("setting desired linear velocity to %f\n", desiredLinearEjectVelocity);
        Flywheel::setLinearEjectVelocity(velocity);
    }

    /**
     * @brief Shoot 1 disk
     * 
     */
    void shootDisk(shootMode mode_override) {
        shootMode mode = mode_override == shoot_mode ? shoot_mode : mode_override;
        
        if (mode == ACCURATE_MODE) {
            while (!readyToShoot()) {
                aim();
                pros::delay(20);
            }
            trigger();
            FlywheelMotor1.moveVoltage(0);
            FlywheelMotor2.moveVoltage(0);
        } 
        else if (mode == FORCE_MODE) {
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
    void shootDisk(int diskCount, shootMode mode_override) {
        shootMode mode = mode_override == shoot_mode ? shoot_mode : mode_override;

        if (mode == ACCURATE_MODE) {
            for (int i = 0; i < diskCount; i++) {
                while (!readyToShoot()) {
                    aim();
                    pros::delay(20);
                }
                trigger();
            }
            
        } 
        else if (mode == FORCE_MODE) {
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