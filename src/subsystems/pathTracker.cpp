#include "main.h"
#include <math.h>
#include <vector>

namespace pathTracker {

    Coordinates prevLookAheadPoint = Coordinates(-1,-1,0);
    Coordinates lookAheadPoint = Coordinates(-1,-1,0);
    float arriveDeviation = 3.5;
    float interpolatingDeviation = 0.05;

    
    /**
     * @brief Turn absolute coordinates to local coordinates
     * 
     * @returns local coordinates 
     */
    Coordinates absoluteToLocalCoordinate() {
        Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
        float xDist = lookAheadPoint.get_x() - selfCoordinate.get_x();
        float yDist = lookAheadPoint.get_y() - selfCoordinate.get_y();

        // apply rotation matrix
        float newX = yDist*cos(selfCoordinate.get_direction()*M_PI/180.0) - xDist*sin(selfCoordinate.get_direction()*M_PI/180.0);
        float newY = yDist*sin(selfCoordinate.get_direction()*M_PI/180.0) + xDist*cos(selfCoordinate.get_direction()*M_PI/180.0);

        return Coordinates(newX, newY, positionSI.theta);
    }

    namespace ramsete {

        std::vector<Coordinates> pathCoords;

        float b = 2.5; // rouqhly the proportional term
        float zeta = 0.6; // roughly the damping term
        float smallScalar = 0.5;

        /**
         * @brief Set the path to follow
         * 
         * @param coords the path to follow
         */
        void setPath(std::vector<Coordinates> coords) {
            pathCoords.clear();
            for (int i = 0; i < coords.size(); i++) {
                pathCoords.push_back(coords[i]);
            }
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


        /**
         * @brief Find the look ahead point of the robot
         * 
         * @return 1 for success, -1 for failure 
         */
        int findLookAheadPoint() {
            // Odom::update_odometry();
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
            for (int i = 1; i < pathCoords.size(); i++) {
                Coordinates coord = pathCoords[i];
                Coordinates prevCoord = pathCoords[i-1];

                // printf("%f %f %f %f\n", coord.get_x(), coord.get_y(), selfCoordinate.get_x(), selfCoordinate.get_y());
                // if suitable distance is found
                if (coord.get_distance(selfCoordinate) > lookAheadRadius && 
                prevCoord.get_distance(selfCoordinate) < lookAheadRadius &&
                closest() < i) {

                    // interpolation
                    float prevX = prevCoord.get_x();
                    float prevY = prevCoord.get_y();

                    float currX = coord.get_x();
                    float currY = coord.get_y();

                    float minT = 0;
                    float maxT = 1;

                    float newX = prevX;
                    float newY = prevY;

                    int iterations = 10;

                    // binary approximation
                    for (int z = 0; z < iterations; z++) {
                        float midT = (minT + maxT) / 2.0;
                        newX = prevX * (1 - midT) + currX * midT;
                        newY = prevY * (1 - midT) + currY * midT;

                        lookAheadPoint = Coordinates(newX, newY, 0);

                        float distToSelf = lookAheadPoint.get_distance(selfCoordinate);

                        if (distToSelf < lookAheadRadius - interpolatingDeviation) {
                            minT = midT + interpolatingDeviation;
                        }
                        else if (distToSelf > lookAheadRadius + interpolatingDeviation) {
                            maxT = midT - interpolatingDeviation;
                        }
                        else {
                            // controller.setText(0,0,std::to_string(i)+", " + "(" + std::to_string(newX) + ", " + std::to_string(newY) + ")");
                            return 1;
                        }
                    }



                }
            }
            
            // if reached the end of path
            if (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) < lookAheadRadius) {
                lookAheadPoint = pathCoords[pathCoords.size()-1];
                printf("x=%f, y=%f\n", lookAheadPoint.get_x(), lookAheadPoint.get_y());
                return 1;
            }
            printf("Failed to find lookahead point\n");
            lookAheadPoint = pathCoords[closest()];
            return -1;
        }

        /**
         * @brief Follow the path with RAMSETE controller
         * 
         */
        void followPath() {
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);

            while (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) >= arriveDeviation) {
                // Odom::update_odometry();
                selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
                findLookAheadPoint();

                Coordinates localLookAhead = absoluteToLocalCoordinate();
                
                float e_x = localLookAhead.get_x();
                float e_y = localLookAhead.get_y();
                float e_theta = atan2(e_y, e_x);

                
                float desired_linearVelocity = std::fmin(std::fmax(10*abs(e_x), 20), 120);
                float desired_angularVelocity = std::fmin(std::fmax(0.1*abs(e_theta), 0), pi/2);
                // controller.setText(0,0,std::to_string(desired_linearVelocity) + "," + std::to_string(desired_angularVelocity));
                float k = 2 * zeta * sqrt(desired_angularVelocity * desired_angularVelocity + b * desired_linearVelocity * desired_linearVelocity);
                
                float targetLinearVelocity = std::fmin(std::fmax(desired_linearVelocity * cos(e_theta) + k * e_y, -400), 400);
                
                float targetAngularVelocity;

                if (abs(e_theta) != 0) {
                    targetAngularVelocity = desired_angularVelocity + k * e_theta + (b*desired_linearVelocity*sin(e_theta)* e_x) / e_theta;
                } else {
                    targetAngularVelocity = 0;
                }

                float leftV;
                float rightV;

                leftV = std::fmin(std::fmax(targetLinearVelocity + targetAngularVelocity, -600), 600);
                rightV = std::fmin(std::fmax(targetLinearVelocity - targetAngularVelocity, -600), 600);
                
                printf("%f\n", targetLinearVelocity);
                controller.setText(0,0,std::to_string(pathCoords[pathCoords.size()-1].get_distance(selfCoordinate)));
                Auto::trackVelocityPID(leftV, rightV);
            }
        }
    }

    namespace pure_pursuit {
        std::vector<Coordinates> pathCoords;

        /**
         * @brief Set the path to follow
         * 
         * @param coords the path to follow
         */
        void setPath(std::vector<Coordinates> coords) {
            pathCoords.clear();
            for (int i = 0; i < coords.size(); i++) {
                pathCoords.push_back(coords[i]);
            }
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


        /**
         * @brief Find the look ahead point of the robot
         * 
         * @return 1 for success, -1 for failure 
         */
        int findLookAheadPoint() {
            // Odom::update_odometry();
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
            for (int i = 1; i < pathCoords.size(); i++) {
                Coordinates coord = pathCoords[i];
                Coordinates prevCoord = pathCoords[i-1];

                // if suitable distance is found
                if (coord.get_distance(selfCoordinate) > lookAheadRadius && 
                prevCoord.get_distance(selfCoordinate) < lookAheadRadius &&
                closest() < i) {

                    // interpolation
                    float prevX = prevCoord.get_x();
                    float prevY = prevCoord.get_y();

                    float currX = coord.get_x();
                    float currY = coord.get_y();

                    float minT = 0;
                    float maxT = 1;

                    float newX = prevX;
                    float newY = prevY;

                    int iterations = 10;

                    // binary approximation
                    for (int z = 0; z < iterations; z++) {
                        float midT = (minT + maxT) / 2.0;
                        newX = prevX * (1 - midT) + currX * midT;
                        newY = prevY * (1 - midT) + currY * midT;

                        lookAheadPoint = Coordinates(newX, newY, 0);

                        float distToSelf = lookAheadPoint.get_distance(selfCoordinate);

                        if (distToSelf < lookAheadRadius - interpolatingDeviation) {
                            minT = midT + interpolatingDeviation;
                        }
                        else if (distToSelf > lookAheadRadius + interpolatingDeviation) {
                            maxT = midT - interpolatingDeviation;
                        }
                        else {
                            printf("x=%f, y=%f\n", newX, newY);
                            // controller.setText(0,0,std::to_string(i)+", " + "(" + std::to_string(newX) + ", " + std::to_string(newY) + ")");
                            return 1;
                        }
                    }



                }
            }
            
            // if reached the end of path
            if (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) < lookAheadRadius) {
                lookAheadPoint = pathCoords[pathCoords.size()-1];
                printf("x=%f, y=%f\n", lookAheadPoint.get_x(), lookAheadPoint.get_y());
                return 1;
            }
            printf("Failed to find lookahead point\n");
            lookAheadPoint = pathCoords[closest()];
            return -1;
        }

         /**
         * @brief Follow the path with pure pursuit controller
         * 
         */
        void followPath() {

            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);

            // if not reached the end
            while (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) >= arriveDeviation) {
                // Odom::update_odometry();
                findLookAheadPoint();
                // controller.setText(0,0,"(" + std::to_string(positionSI.xPercent) + ", " + std::to_string(positionSI.yPercent) + ")");

                Coordinates localLookAhead = absoluteToLocalCoordinate();
                // controller.setText(0,0,"(" + std::to_string(localLookAhead.get_x()) + ", " + std::to_string(localLookAhead.get_y()) + ")");

                float curvature = (2*localLookAhead.get_x()) / (lookAheadRadius * lookAheadRadius);
                printf("(%f, %f), %f\n", localLookAhead.get_x(), localLookAhead.get_y(), curvature);

                float targetV = 100;
                float percentTrackWidth = wheeltrackLength.convert(meter) / fieldLength * 100;

                float leftTargetV = targetV * (1+curvature*percentTrackWidth/2);
                float rightTargetV = targetV * (1-curvature*percentTrackWidth/2);

                Auto::trackVelocityPID(leftTargetV, rightTargetV);
                pros::delay(20);
            }   

        }
    }
}
