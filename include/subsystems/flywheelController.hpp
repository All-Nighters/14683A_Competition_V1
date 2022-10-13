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
    namespace grapher {
        void graph_velocity(float target, float current);
        
    }
}