/**
 * @file gun.hpp
 *
 * @brief Shooter functions
 *
 */

namespace Gun {
    extern shootMode shoot_mode;

    void init(shootMode mode);

    void trigger(float indexer_power = 8000);

    void reposition();

    bool readyToShoot();

    void shootDisk(shootMode mode_override = shoot_mode, float indexer_power = 8000);

    void shootDisk(int diskCount, shootMode mode_override = shoot_mode, float indexer_power = 8000);

    void shootDiskAsync();

    void shootDiskAsync(int diskCount);

    void aim();

    void powerFlyWheel();
}