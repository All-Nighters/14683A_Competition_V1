class MotionProfile {

protected:
    float motion_distance;
    float motion_velocity_max;
    float motion_acceleration;
    float motion_time_full;

private:
    float motion_time_sliding;
    float motion_time_speeding;

public:
    MotionProfile(float distance, float velocity_max, float acceleration);
    float get_distance(float time);
    float get_velocity(float time);
    float get_time();

};


class S_CurveProfile : public MotionProfile{

private:
    float motion_jerk;
    float t2, t3, t4, t5, t6, t7;
    float motion_time_accelerate;
    float last_dist, last_velocity, last_time;
    unsigned int time_section_pointer;
public:
    S_CurveProfile(float distance, float velocity_max, float acceleration, float jerk);
    float get_distance(float time);
    float get_velocity(float time);
    float get_acceleration(float time);
    unsigned int time_section_selection(float time);

};
