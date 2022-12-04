#include "main.h"

namespace Flywheel {
    
    float VpHI = 10;
    float ViHI = 0;
    float VdHI = 0;

    float VpLO = 1.5;
    float ViLO = 0;
    float VdLO = 0;

    float Vp = VpLO;
    float Vi = ViLO;
    float Vd = VdLO;

    float idle_linear_velocity = 2;


    float prev_v_error = 0;
    float v_control_coutput = 0;

    float queued_linear_velocity = 0;

    bool keep_running = true; // to stop the control loop, set this to false

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
                float target = Flywheel::getExpectRPMFromEjectVelocity(queued_linear_velocity);
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
     * @brief Get the current velocity of the flywheel in RPM. Return 0 if unreadable
     * 
     * @return current RPM of the flywheel
     */
    float getCurrentVelocity() {
        float flywheel_1_velocity = FlywheelMotor1.getActualVelocity();
        float flywheel_2_velocity = FlywheelMotor2.getActualVelocity();
        float velocity;

        if (isinf(flywheel_1_velocity) && isinf(flywheel_2_velocity)) {
            printf("Warning: flywheel 1 and 2 velocity is inf\n");
            velocity = 0;
        }
        else if (isinf(flywheel_1_velocity)) {
            printf("Warning: flywheel 1 velocity is inf\n");
            velocity = (FlywheelMotor2.getActualVelocity()) * 15;
        }
        else if (isinf(flywheel_2_velocity)) {
            printf("Warning: flywheel 2 velocity is inf\n");
            velocity = (FlywheelMotor1.getActualVelocity()) * 15;
        } else {
            velocity = (FlywheelMotor1.getActualVelocity() + FlywheelMotor2.getActualVelocity()) / 2.0 * 15;
        }
        return velocity;
    }

    float prev_vel = 0;
    float prev_prev_vel = 0;
    float prev_ctlout = 0;

    float prev_curr_vel = 0;

    /**
     * @brief PID for controlling flywheel velocity
     * 
     * @param target_velocity target RPM of the flywheel
     */
    void velocityPID(float target_velocity) {
        
        float a1 = 0.6;
        float a2 = 0.6;
        
        float vel_reading = getCurrentVelocity() / 15.0;
        float current_velocity = std::fmax(std::fmin(vel_reading, prev_vel), std::fmin(std::fmax(getCurrentVelocity() / 15.0, prev_vel), prev_prev_vel));
        current_velocity = a1 * current_velocity + (1-a1) * prev_curr_vel;
        // float current_velocity = getCurrentVelocity() / 15.0;
        

        float v_error = std::fmax(std::fmin(target_velocity, 3000), 0) / 15.0 - current_velocity;
        
        float used_vp = queued_linear_velocity == 0 ? 100 : Vp;

        float deriv_error = v_error - prev_v_error;
        
        v_control_coutput = a2 * clamp(v_control_coutput + (v_error * Vp + deriv_error * Vd), 0, 12000.0) + (1 - a2) * prev_ctlout;

        FlywheelMotor1.moveVoltage(v_control_coutput);
        FlywheelMotor2.moveVoltage(v_control_coutput * 0.8);

        prev_v_error = v_error;
        prev_prev_vel = prev_vel;
        prev_vel = vel_reading;

        prev_curr_vel = current_velocity;

        prev_ctlout = v_control_coutput;
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
        if (velocity > 6) {
            Vp = VpHI;
            Vi = ViHI;
            Vd = VdHI;
        } else {
            Vp = VpLO;
            Vi = ViLO;
            Vd = VdLO;
        }
        queued_linear_velocity = velocity;
    }
    
    /**
     * @brief Run the flywheel with idling speed
     * 
     */
    void idle() {
        setLinearEjectVelocity(idle_linear_velocity);
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
        while (keep_running) {
            float rpm = getExpectRPMFromEjectVelocity(queued_linear_velocity);
            spinVelocityRPM(rpm);
            pros::delay(20);
        }
    }


    /**
     * @brief Start the velocity control loop
     * 
     */
    void startControlLoop() {
        keep_running = true;
        pros::Task loop(velocityControlLoop);
    }

    /**
     * @brief Stop the velocity control loop
     * 
     */
    void stopControlLoop() {
        keep_running = false;
    }
}
