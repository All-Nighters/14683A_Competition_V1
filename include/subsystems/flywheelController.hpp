/**
 * @file flywheelController.hpp
 *
 * @brief Basic flywheel controlling functions
 *
 */


namespace Flywheel {
    void spinVelocityRPM(float rpm);
    void setLinearEjectVelocity(float velocity);
    float getCurrentVelocity();
    float getExpectRPMFromEjectVelocity(float velocity);
    float getCurrentEjectVelocity();
    void idle();
    void stop();
    void startControlLoop();
    void stopControlLoop();
    namespace grapher {
        void start_graphing();
        void stop_graphing();
    }
}