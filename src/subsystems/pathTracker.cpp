#include "main.h"
#include <math.h>
#include <vector>

namespace pathTracker {
    float arriveDeviation = 3.5;
    float interpolatingDeviation = 0.05;

    
    /**
     * @brief Turn absolute coordinates to local coordinates
     * 
     * @param coord global coordinates to convert 
     * @returns local coordinates 
     */
    Coordinates absoluteToLocalCoordinate(Coordinates& coord) {
        Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
        float xDist = coord.get_x() - positionSI.xPercent; // shoub be -ve
        float yDist = coord.get_y() - positionSI.yPercent; // should be +ve

        // apply rotation matrix
        float newX = yDist*cos(positionSI.theta*M_PI/180.0) - xDist*sin(positionSI.theta*M_PI/180.0);
        // = 0
        float newY = yDist*sin(positionSI.theta*M_PI/180.0) + xDist*cos(positionSI.theta*M_PI/180.0);
        // +

        Odom::debug();
        // printf("x = %f, y = %f, facing = %f —> newX (side) = %f, newY (forward) = %f\n", positionSI.xPercent, positionSI.yPercent, positionSI.theta, newX, newY);
        return Coordinates(newX, newY, positionSI.theta);
    }

    namespace ramsete {

        std::vector<Waypoint> pathCoords;

        float b = 0.6; // rouqhly the proportional term
        float zeta = 0.8; // roughly the damping term

        void configureWaypoint() {
            for (int i = 0; i < pathCoords.size(); i++) {
                float angle; // the angle of the point
                float angle_difference; // use angle difference as velocity
                if (i-1 < 0 && i+1 < pathCoords.size()) {
                    angle = atan2(pathCoords[i+1].get_y() - pathCoords[i].get_y(), pathCoords[i+1].get_x() - pathCoords[i].get_x()) * 180 / pi;
                    angle_difference = 0;
                } else if (i+1 < pathCoords.size()) {
                    float next_angle = atan2(pathCoords[i+1].get_y() - pathCoords[i].get_y(), pathCoords[i+1].get_x() - pathCoords[i].get_x()) * pi / 180;
                    float prev_angle = atan2(pathCoords[i].get_y() - pathCoords[i-1].get_y(), pathCoords[i].get_x() - pathCoords[i-1].get_x()) * pi / 180;

                    if ((next_angle / abs(next_angle)) != (prev_angle / abs(prev_angle)) and ((abs(next_angle) + abs(prev_angle)) > 180)) {
                        angle = ((next_angle + prev_angle) / 2.0) + 180;
                    } else {
                        angle = ((next_angle + prev_angle) / 2.0);
                    }
                    angle_difference = next_angle - angle;
                } else {
                    angle = atan2(pathCoords[i].get_y() - pathCoords[i-1].get_y(), pathCoords[i].get_x() - pathCoords[i-1].get_x()) * 180 / pi;
                    angle_difference = 0;
                }

                pathCoords[i] = Waypoint(pathCoords[i].get_x(), pathCoords[i].get_y(), angle, pathCoords[i].get_linear_vel(), 0.01 * angle_difference * pathCoords[i].get_linear_vel());
            }
        }
        /**
         * @brief Set the path to follow
         * 
         * @param coords the path to follow
         */
        void setPath(std::vector<Waypoint> coords) {
            pathCoords.clear();
            for (int i = 0; i < coords.size(); i++) {
                pathCoords.push_back(coords[i]);
            }
            configureWaypoint();
        }


        /**
         * @brief Get the index of the closest point to the robot
         * 
         * @returns the index of the closest point to the robot
         */
        int closest() {
            float xDist = (pathCoords[0].get_x() - positionSI.xPercent);
            float yDist = (pathCoords[0].get_y() - positionSI.yPercent);
            float dist = sqrt(xDist*xDist + yDist*yDist);
            float idx = 0;
            for (int i = 0; i < pathCoords.size(); i++) {
                float xD = (pathCoords[i].get_x() - positionSI.xPercent);
                float yD = (pathCoords[i].get_y() - positionSI.yPercent);
                float D = sqrt(xD*xD + yD*yD);
                if (D < dist) {
                    dist = D;
                    idx = i;
                }
            }
            return idx;
        }

        void step() {
            // set lookahead point point to the point closest to the robot
            Waypoint lookAheadPoint = pathCoords[closest()]; 

            Coordinates localLookAhead = absoluteToLocalCoordinate(lookAheadPoint);
            
            float e_x = localLookAhead.get_x();
            float e_y = localLookAhead.get_y();
            float e_theta = lookAheadPoint.get_direction() - positionSI.theta;
            
        
            float desired_linearVelocity = lookAheadPoint.get_linear_vel();
            float desired_angularVelocity = lookAheadPoint.get_ang_vel();

            float k = 2 * zeta * sqrt(desired_angularVelocity * desired_angularVelocity + b * desired_linearVelocity * desired_linearVelocity);
            
            float targetLinearVelocity = std::fmin(std::fmax(desired_linearVelocity * cos(e_theta) + k * e_y, -100), 100);
            
            float targetAngularVelocity;

            if (abs(e_theta) != 0) {
                targetAngularVelocity = desired_angularVelocity + k * e_theta + (b*desired_linearVelocity*sin(e_theta)* e_x) / e_theta;
            } else {
                targetAngularVelocity = 0;
            }

            float leftV = std::fmin(std::fmax(targetLinearVelocity + targetAngularVelocity, -200), 200);
            float rightV = std::fmin(std::fmax(targetLinearVelocity - targetAngularVelocity, -200), 200);

            
            Auto::trackVelocityPID(leftV, rightV);
        }


        /**
         * @brief Follow the path with RAMSETE controller
         * 
         */
        void followPath() {
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);

            while (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) >= arriveDeviation) {
                selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
                step();

                pros::delay(20);
            }
        }
    }
}
