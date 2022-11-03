/**
 * @file gun.hpp
 *
 * @brief Shooter functions
 *
 */

namespace Gun {
    void init(shootMode mode);

    void reposition();

    bool readyToShoot();

    void shootDisk();

    void shootDisk(int diskCount);

    void shootDiskAsync();

    void shootDiskAsync(int diskCount);

    void aim();

    void powerFlyWheel();
}