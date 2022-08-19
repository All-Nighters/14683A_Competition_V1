namespace Flywheel {
    void spinVelocityRPM(float rpm);
    void setLinearEjectVelocity(float velocity);
    float getCurrentVelocity();
    float getExpectRPMFromEjectVelocity(float velocity);
    
    namespace grapher {
        void graph_velocity(float target, float current);
        
    }
}