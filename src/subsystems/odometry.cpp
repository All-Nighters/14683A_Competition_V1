#include "main.h"
namespace Odom {

	//CONSTANTS / Hard-Coded Values
	//Radius of tracking wheels in meters
	float WHEEL_RADIUS = trackingWheelDiameter.convert(meter) / 2.0; //was 1.379 //1.43

	//Starting angle (relative to field) (RADIANS)
	float THETA_START = 0;

	//The starting x and y coordinates of the bot (meters)
	//These distances are relative to some point (0,0) on the field
	//Relative to: BOTTOM LEFT CORNER
	float X_START = 0;
	float Y_START = 0; 

	//Distances of tracking wheels from tracking center (meters)
	float LTrackRadius = wheeltrackLength.convert(meter)/2.0; 
	float RTrackRadius = wheeltrackLength.convert(meter)/2.0; 
	float STrackRadius = middleEncoderDistance.convert(meter);

	//Calculated Values (every loop)
	//Angles (DEGREES) *NEEDS TO BE CONVERTED TO RADIANS FOR MATH*
	float LPos = 0;
	float RPos = 0;
	float SPos = 0;

	float LPrevPos = 0;
	float RPrevPos = 0;
	float SPrevPos = 0;

	//Distances traveled by tracking wheels each loop (meters)
	float deltaDistL = 0;
	float deltaDistR = 0;
	float deltaDistS = 0;

	//Distance summations (since last reset)
	float totalDeltaDistL = 0;
	float totalDeltaDistR = 0;

	//The current angle of the bot (RADIANS)
	float currentAbsoluteOrientation = THETA_START;
	//The previous angle of the bot (RADIANS)
	float previousTheta = THETA_START;

	//The change in Theta each loop (RADIANS)
	float deltaTheta = 0;

	//The Average angle Theta (In RADIANS) throughout the arc
	//currentAbsoluteOrientation + (deltaTheta / 2)
	float avgThetaForArc = currentAbsoluteOrientation + (deltaTheta / 2);

	//The changes in the X and Y positions (meters)
	/*These are calculated on a local basis each loop,
	then converted to global position changes */
	float deltaXLocal = 0;
	float deltaYLocal = 0;

	//The X and Y offsets converted from their local forms (meters)
	float deltaXGlobal = 0;
	float deltaYGlobal = 0;

	//The global position of the bot (meters)
	float xPosGlobal = X_START;
	float yPosGlobal = Y_START;


	odomMode odometry_mode = TWOWHEELIMU;

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
		while(1) {
			//Get encoder values (DEGREES)
			
			if (odometry_mode == BASIC) {
				float gear_ratio = 36/60.0;
				LPos = ((LFMotor.getPosition() + LBMotor.getPosition()) / 2.0) * gear_ratio;
				RPos = ((RFMotor.getPosition() + RBMotor.getPosition()) / 2.0) * gear_ratio;
				SPos = -midTW.get();
			} else {
				LPos = 0;
				RPos = rightTW.get();
				SPos = -midTW.get();
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

			if (odometry_mode == THREEWHEEL || odometry_mode == BASIC) {
				currentAbsoluteOrientation = THETA_START + ((totalDeltaDistL - totalDeltaDistR) / (LTrackRadius + RTrackRadius));
			} 
			else if (odometry_mode == TWOWHEELIMU) {
				currentAbsoluteOrientation = THETA_START + ((imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation()) / 2) * M_PI / 180.0;

				if (isinf(currentAbsoluteOrientation)) {
					currentAbsoluteOrientation = 0;
				}
				// currentAbsoluteOrientation = std::fmod(currentAbsoluteOrientation, 2*pi);
			}

			//Calculate the change in the angle of the bot (RADIANS)
			deltaTheta = currentAbsoluteOrientation - previousTheta;

			//Update the previous Theta value (RADIANS)  
			previousTheta = currentAbsoluteOrientation;

			//If we didn't turn, then we only translated
			if(deltaTheta == 0) {
			deltaXLocal = deltaDistS;
			// could be either L or R, since if deltaTheta == 0 we assume they're =
			deltaYLocal = deltaDistL;
			}
			//Else, caluclate the new local position
			else {
			//Calculate the changes in the X and Y values (meters)
			//General equation is:
				//Distance = 2 * Radius * sin(deltaTheta / 2)
			deltaXLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistS / deltaTheta) + STrackRadius);
			deltaYLocal = 2 * sin(deltaTheta / 2.0) * ((deltaDistR / deltaTheta) + RTrackRadius);

			}

			//The average angle of the robot during it's arc (RADIANS)
			avgThetaForArc = currentAbsoluteOrientation - (deltaTheta / 2);

			deltaXGlobal = (deltaYLocal * cos(avgThetaForArc)) - (deltaXLocal * sin(avgThetaForArc));
			deltaYGlobal = (deltaYLocal * sin(avgThetaForArc)) + (deltaXLocal * cos(avgThetaForArc));

			//Wraps angles back around if they ever go under 0 or over 2 pi
			// while(currentAbsoluteOrientation >= 2 * M_PI) {
			// currentAbsoluteOrientation -= 2 * M_PI;
			// }
			
			// while(currentAbsoluteOrientation < 0) {
			// currentAbsoluteOrientation += 2 * M_PI;
			// }

			//Update global positions
			xPosGlobal += deltaXGlobal;
			yPosGlobal += deltaYGlobal;
			save_results();

			pros::delay(10);

		}
		return 1;
	}
	
	/**
	 * @brief Initializes odometry 
	 * 
	 * @param mode odometry mode
	 */
	void init(odomMode mode) {
		odometry_mode = mode;
		imu_sensor_1.tare();
		imu_sensor_2.tare();
		rightTW.reset();
		midTW.reset();
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
		printf("Encoder: l=%f, r=%f, m=%f, imu=%f\n",  leftTW.get(),  rightTW.get(), -midTW.get(), (imu_sensor_1.get_rotation() + imu_sensor_2.get_rotation())/2);
	}

	/**
	 * @brief Set new odometry state
	 * 
	 */
	void set_state(QLength x, QLength y, QAngle angle) {
		X_START = x.convert(meter);
		Y_START = y.convert(meter);
		THETA_START = angle.convert(radian);

		xPosGlobal = X_START;
		yPosGlobal = Y_START;
		currentAbsoluteOrientation = THETA_START;
		save_results();
	}

	/**
	 * @brief Set new odometry state in percentage
	 * 
	 */
	void set_state(float x, float y, float angle) {
		X_START = percentageToMeter(x);
		Y_START = percentageToMeter(y);
		THETA_START = angle*pi/180.0;

		xPosGlobal = X_START;
		yPosGlobal = Y_START;
		currentAbsoluteOrientation = THETA_START;
		save_results();
	}
}
