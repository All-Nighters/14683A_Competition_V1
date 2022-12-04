#include "main.h"
namespace Odom {

	float WHEEL_RADIUS = wheelDiameter.convert(meter) / 2.0;
	//Starting angle (relative to field) (RADIANS)
	float THETA_START = 0;

	//The starting x and y coordinates of the bot (meters)
	float X_START = 0;
	float Y_START = 0; 

	//Distances of tracking wheels from tracking center (meters)
	float LTrackRadius = wheeltrackLength.convert(meter)/2.0; 
	float RTrackRadius = wheeltrackLength.convert(meter)/2.0; 
	float STrackRadius = middleEncoderDistance.convert(meter);

	float LPos = 0;
	float RPos = 0;
	float SPos = 0;

	float LPrevPos = 0;
	float RPrevPos = 0;
	float SPrevPos = 0;

	float deltaDistL = 0;
	float deltaDistR = 0;
	float deltaDistS = 0;

	float totalDeltaDistL = 0;
	float totalDeltaDistR = 0;

	float currentAbsoluteOrientation = THETA_START;
	float previousTheta = THETA_START;

	float deltaTheta = 0;
	float avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

	float deltaXLocal = 0;
	float deltaYLocal = 0;

	float deltaXGlobal = 0;
	float deltaYGlobal = 0;

	float xPosGlobal = X_START;
	float yPosGlobal = Y_START;


	odomMode odometry_mode = MOTOR_IMU;

	/**
	 * @brief Save global position and rotation to positionSI structure
	 * 
	 */
	void save_results() {
		positionSI.x = xPosGlobal;
		positionSI.xPercent = meterToPercentage(xPosGlobal);

		positionSI.y = yPosGlobal;
		positionSI.yPercent = meterToPercentage(yPosGlobal);

		positionSI.theta = currentAbsoluteOrientation * 180 / pi;
	}

	/**
	 * @brief Loop to run odometry
	 * 
	 * @returns 1 
	 */
	int position_tracking() {
		printf("Position tracking started\n");
		while(1) {
			// get position values
			if (odometry_mode == MOTOR_IMU) {
				LPos = Drivetrain::getLeftPosition();
				RPos = Drivetrain::getRightPosition();
				SPos = midTW.get();
				// printf("%f, %f, %f %f\n", LPos, RPos, SPos, (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2);
			} 
			else if (odometry_mode == LEFTTW_IMU) {
				// LPos = leftTW.get();
				RPos = 0;
				SPos = midTW.get();
			}
			else if (odometry_mode == RIGHTTW_IMU) {
				LPos = 0;
				RPos = rightTW.get();
				SPos = midTW.get();
			}
			else if (odometry_mode == THREEWHEEL) {
				// LPos = leftTW.get();
				RPos = rightTW.get();
				SPos = midTW.get();
			}

			//Calculate distance traveled by tracking each wheel (meters)
			//Converts degrees to radians
			deltaDistL = ((LPos - LPrevPos) * M_PI / 180) * WHEEL_RADIUS;
			deltaDistR = ((RPos - RPrevPos) * M_PI / 180) * WHEEL_RADIUS;
			deltaDistS = ((SPos - SPrevPos) * M_PI / 180) * WHEEL_RADIUS;

			//Update previous values to be used next loop (DEGREES)
			LPrevPos = LPos;
			RPrevPos = RPos;
			SPrevPos = SPos;

			//Total change in each of the L and R encoders since last reset (meters)
			//These are used to calculate the absolute orientation of the bot
			totalDeltaDistL += deltaDistL;
			totalDeltaDistR += deltaDistR;

			//Calculate the current absolute orientation (RADIANS)

			if (odometry_mode == THREEWHEEL) {
				currentAbsoluteOrientation = THETA_START + ((totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius));
			} 
			else if (odometry_mode == LEFTTW_IMU || odometry_mode == RIGHTTW_IMU || odometry_mode == MOTOR_IMU) {
				currentAbsoluteOrientation = THETA_START + ((imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2) * M_PI / 180.0;

				if (isinf(currentAbsoluteOrientation)) {
					currentAbsoluteOrientation = 0;
				}
			}

			//Calculate the change in the angle of the bot (RADIANS)
			deltaTheta = currentAbsoluteOrientation - previousTheta;

			//Update the previous Theta value (RADIANS)  
			previousTheta = currentAbsoluteOrientation;

			//If we didn't turn, then we only translated
			if(deltaTheta == 0) {
				deltaXLocal = deltaDistS;
				if (odometry_mode == THREEWHEEL || odometry_mode == LEFTTW_IMU || odometry_mode == MOTOR_IMU) {
					deltaYLocal = deltaDistL;
				}
				else if (odometry_mode == RIGHTTW_IMU) {
					deltaYLocal = deltaDistR;
				}
			}
			//Else, caluclate the new local position
			else {
			//Calculate the changes in the X and Y values (meters)

			deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) - STrackRadius);
			deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistR / deltaTheta) + RTrackRadius);
			}

			//The average angle of the robot during it's arc (RADIANS)
			avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

			deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
			deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

			//Update global positions
			xPosGlobal += deltaXGlobal;
			yPosGlobal += deltaYGlobal;
			save_results();

			pros::delay(10);

		}
		return 1;
	}

	/**
	 * @brief tare sensor readings
	 * 
	 * @param mode odometry mode
	 */
	void tare_sensors() {
		switch (odometry_mode) {
			case MOTOR_IMU:
				imu_sensor_1.tare();
				imu_sensor_2.tare();
				Drivetrain::tarePosition();
				midTW.reset();
				break;
			case LEFTTW_IMU:
				imu_sensor_1.tare();
				imu_sensor_2.tare();
				// leftTW.reset();
				midTW.reset();
				break;
			case RIGHTTW_IMU:
				imu_sensor_1.tare();
				imu_sensor_2.tare();
				rightTW.reset();
				midTW.reset();
				break;
			case THREEWHEEL:
				// leftTW.reset();
				rightTW.reset();
				midTW.reset();
				break;
		}
	}
	
	/**
	 * @brief Initializes odometry 
	 * 
	 * @param mode odometry mode
	 */
	void init(odomMode mode) {
		odometry_mode = mode;
		tare_sensors();
		
		pros::Task tracking(position_tracking);
	}


	/**
	 * @brief Get odometry mode
	 * 
	 * @return current odometry mode 
	 */
	odomMode getOdomMode() {
		return odometry_mode;
	}

	/**
	 * @brief Prints debug message
	 * 
	 */
	void debug() {
		printf("Odometry: x=%f, y=%f, a=%f\n", positionSI.xPercent, positionSI.yPercent, positionSI.theta);
		if (odometry_mode == THREEWHEEL) {
			// printf("Encoder: l=%f, r=%f, m=%f, imu=%f\n",  leftTW.get(),  rightTW.get(), midTW.get());
		}
		else if (odometry_mode == LEFTTW_IMU) {
			// printf("Encoder: l=%f, m=%f, imu=%f\n",  leftTW.get(), midTW.get(), (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2);
		}
		else if (odometry_mode == RIGHTTW_IMU) {
			printf("Encoder: r=%f, m=%f, imu=%f\n",  rightTW.get(), midTW.get(), (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2);
		}
		else if (odometry_mode == MOTOR_IMU) {
			// printf("Encoder: l=%f, r=%f, m=%f, imu=%f\n",  Drivetrain::getLeftPosition(),  Drivetrain::getRightPosition(), midTW.get(), (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2);
		} else {
			printf("Error: Unspecified odom mode\n");
		}
		
	}

	/**
	 * @brief Set new odometry state
	 * 
	 */
	void set_state(QLength x, QLength y, QAngle angle) {
		tare_sensors();
		X_START = x.convert(meter);
		Y_START = y.convert(meter);
		THETA_START = angle.convert(radian);

		LPos = 0;
		RPos = 0;
		SPos = 0;

		LPrevPos = 0;
		RPrevPos = 0;
		SPrevPos = 0;

		deltaDistL = 0;
		deltaDistR = 0;
		deltaDistS = 0;

		totalDeltaDistL = 0;
		totalDeltaDistR = 0;

		currentAbsoluteOrientation = THETA_START;
		previousTheta = THETA_START;
		deltaTheta = 0;
		avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

		deltaXLocal = 0;
		deltaYLocal = 0;
		deltaXGlobal = 0;
		deltaYGlobal = 0;

		xPosGlobal = X_START;
		yPosGlobal = Y_START;

		save_results();
		printf("Set odom state to (%f, %f, %f)\n", positionSI.xPercent, positionSI.yPercent, positionSI.theta);
	}

	/**
	 * @brief Set new odometry state in percentage
	 * 
	 */
	void set_state(float x, float y, float angle) {
		tare_sensors();
		X_START = percentageToMeter(x);
		Y_START = percentageToMeter(y);
		THETA_START = angle * pi/180;

		LPos = 0;
		RPos = 0;
		SPos = 0;

		LPrevPos = 0;
		RPrevPos = 0;
		SPrevPos = 0;

		deltaDistL = 0;
		deltaDistR = 0;
		deltaDistS = 0;

		totalDeltaDistL = 0;
		totalDeltaDistR = 0;

		currentAbsoluteOrientation = THETA_START;
		previousTheta = THETA_START;
		deltaTheta = 0;
		avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

		deltaXLocal = 0;
		deltaYLocal = 0;
		deltaXGlobal = 0;
		deltaYGlobal = 0;
		
		xPosGlobal = X_START;
		yPosGlobal = Y_START;

		save_results();
		printf("Set odom state to (%f, %f, %f)\n", positionSI.xPercent, positionSI.yPercent, positionSI.theta);
	}
}
