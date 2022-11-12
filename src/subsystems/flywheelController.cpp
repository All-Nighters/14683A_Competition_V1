#include "main.h"

namespace Flywheel {
    
    float Vp = 0.08;
    float Vi = 0;
    float Vd = 2;

    float idleLinearVelocity = 2;


    float prevVError = 0;
    float prevCtlOutput = 0;

    float queuedLinearVelocity = 0;

    bool keepRunning = true; // to stop the control loop, set this to false

    namespace grapher {

        bool keepRunningGrapher = true;

        lv_obj_t * chart;
        
        /**
         * @brief Graph current and target flywheel RPM
         * 
         */
        void graph_velocity()
        {
            const int graph_length = 1000;
            float current_vel[graph_length];
            float target_vel[graph_length];
            bool written[graph_length] = {false};
            while (keepRunningGrapher) 
            {
                float target = Flywheel::getExpectRPMFromEjectVelocity(queuedLinearVelocity);
                float current = Flywheel::getCurrentVelocity();
                // printf("%f\n", current);

                lv_obj_clean(lv_scr_act());
                /*Create a chart*/
                chart = lv_chart_create(lv_scr_act(), NULL);
                lv_obj_set_size(chart, 300, 200);
                lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
                lv_chart_set_range(chart, -3600, 3600);
                lv_chart_set_type(chart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/
                lv_chart_set_point_count(chart, graph_length);

                /*Add two data series*/
                lv_chart_series_t * currentVelocityPlot = lv_chart_add_series(chart, LV_COLOR_RED);
                lv_chart_series_t * targetVelocityPlot = lv_chart_add_series(chart, LV_COLOR_GREEN);

                bool writtenData = false;
                for (int i = 0; i < graph_length; i++) {                
                    if (!written[i]) {
                        written[i] = true;
                        writtenData = true;
                        current_vel[i] = current;
                        target_vel[i] = target;
                        break;
                    }
                }

                if (!writtenData) {
                    // shift the graph to the left
                    for (int i = 1; i < graph_length; i++) {
                        current_vel[i-1] = current_vel[i];
                    }
                    written[graph_length-1] = true;
                    current_vel[graph_length-1] = current;
                    target_vel[graph_length-1] = target;
                }


                for (int i = 0; i < graph_length; i++) {
                    currentVelocityPlot->points[i] = current_vel[i];
                    targetVelocityPlot->points[i] = target_vel[i];
                }

                lv_chart_refresh(chart); /*Required after direct set*/
                pros::delay(20);
            }

            
        }

        void start_graphing() {
            keepRunningGrapher = true;
            pros::Task graph(graph_velocity);
        }

        void stop_graphing() {
            keepRunningGrapher = false;
            lv_obj_clean(lv_scr_act());
        }
    }


    /**
     * @brief Get the current velocity of the flywheel in RPM
     * 
     * @return current RPM of the flywheel
     */
    float getCurrentVelocity() {
        float velocity = (FlywheelMotor1.getActualVelocity() + FlywheelMotor2.getActualVelocity()) / 2.0 * 15;
        return velocity;
    }

    /**
     * @brief PID for controlling flywheel velocity
     * 
     * @param target_velocity target RPM of the flywheel
     */
    void velocityPID(float target_velocity) {
        
        float current_velocity = getCurrentVelocity();

        float v_error = std::fmax(std::fmin(target_velocity, 3000), 0) - current_velocity;
        float deriv_error = v_error - prevVError;
        

        prevCtlOutput += v_error * Vp + deriv_error * Vd;

        FlywheelMotor1.moveVoltage(clamp(prevCtlOutput, -12000.0, 12000.0));
        FlywheelMotor2.moveVoltage(clamp(prevCtlOutput, -12000.0, 12000.0));

        prevVError = v_error;
    }
    
    /**
     * @brief Spin the flywheel with specific velicity
     * 
     * @param rpm target RPM of the flywheel
     */
    void spinVelocityRPM(float rpm) {
        // FlywheelMotor1.moveVelocity(rpm / 18);
        // FlywheelMotor2.moveVelocity(rpm / 18);

        velocityPID(rpm);
    }

    /**
     * @brief Get the Expect R P M From Eject Velocity object
     * 
     * @param velocity eject velocity in meters/second
     * @returns target RPM of the flywheel 
     */
    float getExpectRPMFromEjectVelocity(float velocity) {
        velocity *= 2;
        float angularVelocity = velocity / (flyWheelDiameter.convert(meter) / 2.0);
        float rpm = angularVelocity / (2 * M_PI) * 60;
        return rpm;
    }

    /**
     * @brief Set the linear eject velocity (meters/second) of the flywheel.
     * 
     * Note that the actual flywheel velocity would be lower due to the loss in kinetic energy.
     * Use velocityLossConstant in globals.hpp to take account to the problem.
     * 
     * @param velocity target eject velocity in meters/second
     */
    void setLinearEjectVelocity(float velocity) {
        queuedLinearVelocity = velocity;
    }
    
    /**
     * @brief Run the flywheel with idling speed
     * 
     */
    void idle() {
        setLinearEjectVelocity(idleLinearVelocity);
    }

    /**
     * @brief Stop the flywheel
     * 
     */
    void stop() {
        setLinearEjectVelocity(0);
    }

    /**
     * @brief Get the current eject velocity of the flywheel (meters/second)
     * 
     * @return current eject velocity of the flywheel (meters/second)
     */
    float getCurrentEjectVelocity() {
        return getCurrentVelocity() / 60 * (2 * M_PI) * (flyWheelDiameter.convert(meter) / 2.0) / 2;
    }


    /**
     * @brief Control loop of the flywheel (runs in background)
     * 
     */
    void velocityControlLoop() {
        while (keepRunning) {
            float rpm = getExpectRPMFromEjectVelocity(queuedLinearVelocity);
            spinVelocityRPM(rpm);
            pros::delay(20);
        }
    }


    /**
     * @brief Start the velocity control loop
     * 
     */
    void startControlLoop() {
        keepRunning = true;
        pros::Task loop(velocityControlLoop);
    }

    /**
     * @brief Stop the velocity control loop
     * 
     */
    void stopControlLoop() {
        keepRunning = false;
    }
}
