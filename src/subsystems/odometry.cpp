/**
 * @file odometry.cpp
 *
 * @brief Basic odometry code
 *
 */


#include "main.h"

std::shared_ptr<OdomChassisController> odomChassis =
	ChassisControllerBuilder()
		.withMotors(
			frontLeftMotorPort,  // Top left
			frontRightMotorPort, // Top right (reversed)
			bottomRightMotorPort, // Bottom right (reversed)
			bottomLeftMotorPort   // Bottom left
		)
		// green gearset, 4 inch wheel diameter, 11.5 inch wheel track
		.withDimensions({AbstractMotor::gearset::blue, (84.0 / 36.0)}, {{wheelDiameter, wheeltrackLength}, imev5BlueTPR})
		.withSensors(
			ADIEncoder{leftEncoderPort[0], leftEncoderPort[1]}, // Left encoder in ADI ports A & B (reversed)
			ADIEncoder{rightEncoderPort[0], rightEncoderPort[1]},  // Right encoder in ADI ports C & D
			ADIEncoder{middleEncoderPort[0], middleEncoderPort[1]} // Middle encoder in ADI ports
		)
		.withOdometry({{trackingWheelDiameter, wheeltrackLength}, quadEncoderTPR})
		.buildOdometry();

namespace Odom {
    void tare_odometry() {
        odomChassis->setState({0_m, 0_m, 0_deg});
		update_odometry();
    }
    void update_odometry() {
        position = odomChassis->getState();
        QLength QxPos = position.x;
        QLength QyPos = position.y;
        QAngle QfaceAng = position.theta;
        positionSI.x = QxPos.convert(meter);
        positionSI.y = QyPos.convert(meter);
		positionSI.xPercent = positionSI.x / fieldLength * 100;
		positionSI.yPercent = positionSI.y / fieldLength * 100;

		positionSI.theta = QfaceAng.convert(degree);
		// printf("%f %f %f\n", positionSI.x, positionSI.y, positionSI.theta);
		// float theta = QfaceAng.convert(degree);

		// // convert absolute angle data to -180 to 180 deg when 0 represents the front
		// if ((int)(theta / 180) % 2 == 1) {
		// 	theta = -180 + (theta - (180 * (int)(theta / 180)));
		// } else {
		// 	theta = theta - (180 * (int)(theta / 180));
		// }
        
		// positionSI.facing = theta;
    }

	void setState(QLength x, QLength y, QAngle ang) {
		odomChassis->setState({x, y, ang});
		update_odometry();
	}
}