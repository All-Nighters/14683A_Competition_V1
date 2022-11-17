#include <cmath>
#include "motion_profile.h"
#include "/Users/rockychen/Desktop/Control theory/competition-code/src/utils/linear_approximation.cpp"
#include <fstream>

/**
 * Construct a new Motion Profile object
 *
 * @param distance The total distance of the path
 * @param velocity_max The maximum velocity during the motion
 * @param acceleration The acceleration of the motion
 */
MotionProfile::MotionProfile(float distance, float velocity_max, float acceleration) {
    // constants
    this->motion_distance = distance;
    this->motion_acceleration = acceleration;
    // calculates the reachable maximum velocity
    float velocity_max_actual = std::fmin(std::sqrt(acceleration * distance), velocity_max);
    this->motion_velocity_max = velocity_max_actual;
    // calculates the time of motion
    float speeding_time = velocity_max_actual / acceleration; // for either accelerate/decelerate
    float speeding_distance = velocity_max_actual * speeding_time;
    float sliding_distance = distance - speeding_distance;
    float sliding_time = sliding_distance / velocity_max_actual;
    this->motion_time_speeding = speeding_time;
    this->motion_time_sliding = sliding_time;
    this->motion_time_full = 2 * speeding_time + sliding_time;
}

/**
 * Calculates the instantaneous distance at time
 *
 * @param time The time since the start of the motion
 * @return instantaneous distance at time
 */
float MotionProfile::get_distance(float time) {
    float distance_net = 0.0f;
    // accelerate
    float accelerate_time = std::fmin(time, this->motion_time_speeding);
    distance_net += 0.5f * this->motion_acceleration * std::pow(accelerate_time, 2);
    // slide
    float slide_time = std::fmin(time - this->motion_time_speeding, this->motion_time_sliding);
    if (slide_time > 0) distance_net += this->motion_velocity_max * slide_time;
    // decelerate
    float decelerate_time = time - this->motion_time_speeding - this->motion_time_sliding;
    if (decelerate_time > 0) distance_net += this->motion_velocity_max * decelerate_time - 0.5f * this->motion_acceleration * std::pow(decelerate_time, 2);
    return distance_net;
}

/**
 * Calculates the instantaneous velocity at time
 *
 * @param time The time since the start of the motion
 * @return Instantaneous velocity
 */
float MotionProfile::get_velocity(float time) {
    // accelerate
    if (time < this->motion_time_speeding) return this->motion_acceleration * time;
    // slide
    if (time < this->motion_time_speeding + this->motion_time_sliding) return this->motion_velocity_max;
    // decelerate
    return this->motion_velocity_max - this->motion_acceleration * (time - (this->motion_time_speeding + this->motion_time_sliding));
}

/**
 * Calculates the total time of the motion
 *
 * @return Total time of the motion
 */
float MotionProfile::get_time() {
    return this->motion_time_full;
}



// using namespace math;
// using namespace std;

// static math::InterpolatingMap<double, double> dt_pos;
// static math::InterpolatingMap<double, double> dt_vel;


// int main(){

//     MotionProfile profile(15, 7, 10);

//     ofstream pos;
//     pos.open("pos.txt");
//     ofstream vel;
//     vel.open("vel.txt");

//     for(float t = 0; t < profile.get_time(); t += 0.01f){
//         dt_pos.insert(t, profile.get_distance(t));
//         dt_vel.insert(t, profile.get_velocity(t));
//     }
//     for(float t = 0; t < profile.get_time(); t += 0.01f){
//         pos << dt_pos[t] << endl;
//         vel << dt_vel[t] << endl;
//     }


// }


S_CurveProfile::S_CurveProfile(float distance, float velocity_max, float acceleration, float jerk) : MotionProfile(distance, velocity_max, acceleration){
    this->motion_distance = distance;
    this->motion_velocity_max = velocity_max;
    this->motion_acceleration = acceleration;
    this->motion_jerk = jerk;
    this->motion_time_accelerate = this->motion_acceleration / this->motion_jerk;
    this->t2 = this->motion_velocity_max / this->motion_acceleration;
    this->t3 = this->t2 + this->motion_time_accelerate;
    this->t4 = this->motion_distance / this->motion_velocity_max;
    this->t5 = this->t4 + this->motion_time_accelerate;
    this->t6 = this->t4 + this->t2;
    this->t7 = this->t6 + this->motion_time_accelerate;
    this->motion_time_full = this->t7;
    this->time_section_pointer = 0;
    this->last_dist = 0;
    this->last_velocity = 0;
    this->last_time = 0;
}

unsigned int S_CurveProfile::time_section_selection(float time){
    if(time < this->motion_time_accelerate){
        return 1;
    }
    if(time < this->t2){
        return 2;
    }
    if(time < this->t3){
        return 3;
    }
    if(time < this->t4){
        return 4;
    }
    if(time < this->t5){
        return 5;
    }
    if(time < this->t6){
        return 6;
    }
    if(time < this->t7){
        return 7;
    }
    return 0;
}

float S_CurveProfile::get_distance(float time){
    this->time_section_pointer = time_section_selection(time);

    float current_jerk;
    if(this->time_section_pointer == 1 || this->time_section_pointer == 7) current_jerk = this->motion_jerk;
    else if(this->time_section_pointer == 3 || this->time_section_pointer == 5) current_jerk = -this->motion_jerk;
    else current_jerk = 0;
    float dt = time - this->last_time;

    this->last_time = time;
    //return current_jerk * (1/6.0f) * this->motion_time_accelerate;
    this->last_dist += last_velocity * dt;

    return last_dist;
}


float S_CurveProfile::get_velocity(float time){
    switch(this->time_section_pointer){
        case 1:{
            return this->last_velocity = 0.5f * this->motion_jerk * std::pow(time, 2);
        }
        case 2:{
            return this->last_velocity = this->motion_acceleration * (time - 0.5 * this->motion_time_accelerate);
        }
        case 3:{
            return this->last_velocity = this->motion_acceleration * (time - 0.5 * this->motion_time_accelerate) - 0.5f * this->motion_jerk * std::pow(time - this->t2, 2);
        }
        case 4:{
            return this->last_velocity = this->motion_velocity_max;
        }
        case 5:{
            return this->last_velocity = this->motion_velocity_max - 0.5 * this->motion_jerk * std::pow(time - this->t4, 2);
        }
        case 6:{
            return this->last_velocity = this->motion_acceleration * (this->t2 + this->t5 - time - 0.5 * this->motion_time_accelerate);
        }
        case 7:{
            return this->last_velocity = this->motion_acceleration * (this->t2 + this->t5 - time - 0.5 * this->motion_time_accelerate) + 0.5 * this->motion_jerk * std::pow(time - t6, 2);
        }
    }
    return 0.0f;
}


float S_CurveProfile::get_acceleration(float time){

    switch(this->time_section_pointer){
        case 1:{
            return this->motion_jerk * time;
        }
        case 2:{
            return this->motion_acceleration;
        }
        case 3:{
            return this->motion_acceleration - this->motion_jerk * (time - this->t2);
        }
        case 4:{
            return 0.0f;
        }
        case 5:{
            return -this->motion_jerk * (time - this->t4);
        }
        case 6:{
            return -this->motion_acceleration;
        }
        case 7:{
            return -this->motion_acceleration + this->motion_jerk * (time - this->t6);
        }
    }
    return 0.0f;
}

static math::InterpolatingMap<float, float> S_pos_dt;
static math::InterpolatingMap<float, float> S_vel_dt;
static math::InterpolatingMap<float, float> S_acc_dt;
static math::InterpolatingMap<float, float> N_pos_dt;
static math::InterpolatingMap<float, float> N_vel_dt;


int main(){
    using namespace std;
    S_CurveProfile s_profile(15, 6, 6, 13);
    MotionProfile profile(15, 6, 6);

    ofstream Spos;
    Spos.open("Sposition.txt");
    ofstream Svel;
    Svel.open("Svelocity.txt");
    ofstream Sacc;
    Sacc.open("Sacceleration.txt");
    ofstream Npos;
    Npos.open("Tposition.txt");
    ofstream Nvel;
    Nvel.open("Tvelocity.txt");



    for(float t = 0; t < s_profile.get_time(); t += 0.01){
        // S_pos_dt.insert(t, s_profile.get_distance(t));
        // S_vel_dt.insert(t, s_profile.get_velocity(t));
        // S_acc_dt.insert(t, s_profile.get_acceleration(t));
        Spos << s_profile.get_distance(t) << endl;
        Svel << s_profile.get_velocity(t) << endl;
        Sacc << s_profile.get_acceleration(t) << endl;
        if(t < profile.get_time()){
            Npos << profile.get_distance(t) << endl;
            Nvel << profile.get_velocity(t) << endl;
        }
    }
    Spos.close();
    Svel.close();
    Sacc.close();
    Npos.close();
    Nvel.close();
}






