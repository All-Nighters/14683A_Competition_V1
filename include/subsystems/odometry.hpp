extern std::shared_ptr<OdomChassisController> odomChassis;

namespace Odom {
    void tare_odometry();
    void update_odometry();
    void setState(QLength x, QLength y, QAngle ang);
}