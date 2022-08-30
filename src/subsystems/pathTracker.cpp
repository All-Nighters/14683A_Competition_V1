#include "main.h"
#include <math.h>
#include <vector>

namespace pathTracker {

    Coordinates prevLookAheadPoint = Coordinates(-1,-1,0);
    Coordinates lookAheadPoint = Coordinates(-1,-1,0);
    float arriveDeviation = 0.05;
    float interpolatingDeviation = 0.05;

    

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

        float b = 1; // rouqhly the proportional term
        float zeta = 0.5; // roughly the damping term
        float smallScalar = 0.01;

        void setPath(std::vector<Coordinates> coords) {
            std::copy(coords.begin(), coords.end(), pathCoords.begin());
        }

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

        int findLookAheadPoint() {
            Odom::update_odometry();
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
            for (int i = 1; i < pathCoords.size(); i++) {
                Coordinates coord = pathCoords[i];
                Coordinates prevCoord = pathCoords[i-1];

                // if suitable distance is found
                if (coord.get_distance(selfCoordinate) > lookAheadPercentage && 
                prevCoord.get_distance(selfCoordinate) < lookAheadPercentage && 
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

                        if (distToSelf < lookAheadPercentage - interpolatingDeviation) {
                            minT = midT + interpolatingDeviation;
                        }
                        else if (distToSelf > lookAheadPercentage + interpolatingDeviation) {
                            maxT = midT - interpolatingDeviation;
                        }
                        else {
                            return 1;
                        }
                    }



                }
            }
            
            // if reached the end of path
            if (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) < lookAheadPercentage) {
                lookAheadPoint = pathCoords[pathCoords.size()-1];
                return 1;
            }
            return -1;
        }

        void followPath() {
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);

            while (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) >= arriveDeviation) {
                findLookAheadPoint();

                Coordinates localLookAhead = absoluteToLocalCoordinate();
                
                float e_x = localLookAhead.get_x();
                float e_y = localLookAhead.get_y();
                float e_theta = atan2(e_y, e_x);

                float desired_linearVelocity = std::fmax(std::fmin(smallScalar * abs(e_x), 10), 500);
                float desired_angularVelocity = std::fmax(std::fmin(smallScalar * abs(e_theta), 1), 5);
                float k = 2 * zeta * sqrt(desired_angularVelocity * desired_angularVelocity + b * desired_linearVelocity * desired_linearVelocity);
                
                float targetLinearVelocity = desired_linearVelocity * cos(e_theta) + k * e_x;
                float targetAngularVelocity;

                if (e_theta != 0) {
                    targetAngularVelocity = desired_angularVelocity + k * e_theta + (b*desired_linearVelocity*sin(e_theta)* e_y) / e_theta;
                } else {
                    targetAngularVelocity = 0;
                }

                float targetLinearRPM = ((targetLinearVelocity / 100 * fieldLength) / (trackingWheelDiameter.convert(meter))) / (2 * M_PI) * 60; // convert percent / s to RPM
                float targetAngularRPM = (targetAngularVelocity * (wheeltrackLength.convert(meter)/2)) / (trackingWheelDiameter.convert(meter)) / (2 * M_PI) * 60;
                Auto::trackVelocityPID(targetLinearRPM + targetAngularRPM, targetLinearRPM - targetAngularRPM);
            }
        }
    }

    namespace pure_pursuit {
        std::vector<Coordinates> pathCoords;

        void setPath(std::vector<Coordinates> coords) {
            std::copy(coords.begin(), coords.end(), pathCoords.begin());
        }

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

        int findLookAheadPoint() {
            Odom::update_odometry();
            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);
            for (int i = 1; i < pathCoords.size(); i++) {
                Coordinates coord = pathCoords[i];
                Coordinates prevCoord = pathCoords[i-1];

                // if suitable distance is found
                if (coord.get_distance(selfCoordinate) > lookAheadPercentage && 
                prevCoord.get_distance(selfCoordinate) < lookAheadPercentage &&
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

                        if (distToSelf < lookAheadPercentage - interpolatingDeviation) {
                            minT = midT + interpolatingDeviation;
                        }
                        else if (distToSelf > lookAheadPercentage + interpolatingDeviation) {
                            maxT = midT - interpolatingDeviation;
                        }
                        else {
                            printf("x=%f, y=%f\n", newX, newY);
                            return 1;
                        }
                    }



                }
            }
            
            // if reached the end of path
            if (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) < lookAheadPercentage) {
                lookAheadPoint = pathCoords[pathCoords.size()-1];
                printf("x=%f, y=%f\n", lookAheadPoint.get_x(), lookAheadPoint.get_y());
                return 1;
            }
            printf("Failed to find lookahead point\n");
            return -1;
        }

        void followPath() {

            Coordinates selfCoordinate = Coordinates(positionSI.xPercent, positionSI.yPercent, positionSI.theta);

            // if not reached the end
            while (pathCoords[pathCoords.size()-1].get_distance(selfCoordinate) >= arriveDeviation) {
                findLookAheadPoint();

                Coordinates localLookAhead = absoluteToLocalCoordinate();

                float curvature = (2*localLookAhead.get_y()) / (lookAheadPercentage * lookAheadPercentage);
                
                float targetV = 400;
                float percentTrackWidth = wheeltrackLength.convert(meter) / fieldLength * 100;

                float leftTargetV = targetV * (1+curvature*percentTrackWidth/2);
                float rightTargetV = targetV * (1-curvature*percentTrackWidth/2);

                Auto::trackVelocityPID(leftTargetV, rightTargetV);
                pros::delay(20);
            }   

        }
    }
}
