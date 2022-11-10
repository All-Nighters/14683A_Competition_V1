/**
 * @file gun.hpp
 *
 * @brief Shooter functions
 *
 */

namespace Gun {
    extern shootMode shoot_mode;

    void init(shootMode mode);

    void reposition();

    bool readyToShoot();

    void shootDisk(shootMode mode_override = shoot_mode);

    void shootDisk(int diskCount, shootMode mode_override = shoot_mode);

    void shootDiskAsync();

    void shootDiskAsync(int diskCount);

    void aim();

    void powerFlyWheel();
}