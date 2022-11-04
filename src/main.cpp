#include "main.h"

/**
 * Moves the piston to push the disks to the flywheel
 *
 */
void trigger() {
	IndexerMotor.moveVoltage(6000);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// tare sensors
	imu_sensor_1.tare();
	imu_sensor_2.tare();

	// Initialize subsystems
	Odom::init(TWOWHEELIMU);
	Odom::set_state(50, 50, 0);
	Flywheel::startControlLoop();
	Flywheel::grapher::start_graphing();
	Gun::init(ACCURATE_MODE);

	// Set motor brake modes
	LFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	LBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	IndexerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Auto::moveDistance(2);
	// Odom::debug();
	// Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], false);

	// Auto::turnAngle(90);
	// Odom::set_state(0.368826057_m, 0.888775_m, 0_deg);


	// Odom::set_state(86.38888888888889, 76.2037037037037, 180);
	// Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], false);

	// Odom::set_state(1.8288_m, 1.8288_m, 0_deg);
	// Auto::faceCoordinate(blueHighGoalPosition_percent[0], blueHighGoalPosition_percent[1], false);
	// Odom::set_state(89.72222222222221, 76.1111111111111, 180);
	// Auto::faceCoordinate(redHighGoalPosition_percent[0], redHighGoalPosition_percent[1], false);
	Autos::run(RED_FIRST_SCORING);
}

void PPTest() {
	Odom::set_state(0_m, 0_m, 0_deg);
	std::vector<Coordinates> pathway;
	
	// insert evenly distributed waypoints from (0, 0) to (40, 0)
	for (int i = 0; i < 11; i++) {
		pathway.push_back(Coordinates(4*i, 0, 0));
	}

	// insert evenly distributed waypoints from (40, 0) to (40, 40)
	for (int i = 0; i < 11; i++) {
		pathway.push_back(Coordinates(40, 4*i, 0));
	}

	// // insert evenly distributed waypoints from (40, 40) to (0, 40)
	// for (int i = 0; i < 11; i++) {
	// 	pathway.push_back(Coordinates(40-4*i, 40, 0));
	// }
	pros::delay(2000);
	pathTracker::ramsete::setPath(pathway);
	pathTracker::ramsete::findLookAheadPoint();
	pathTracker::ramsete::followPath();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// PPTest();
	// odomChassis->turnToAngle(180_deg);
	// Auto::faceAngle(90);
	// controller.setText(0,0,std::to_string(positionSI.theta));
	// Auto::faceAngle(-90);
	// controller.setText(0,0,std::to_string(positionSI.theta));
	// Auto::faceAngle(90);
	// controller.setText(0,0,std::to_string(positionSI.theta));
	// Auto::faceAngle(-90);
	// controller.setText(0,0,std::to_string(positionSI.theta));
	// Auto::faceAngle(0);
	// controller.setText(0,0,std::to_string(positionSI.theta));

	// autonomous();

	printf("Hello Allnighters\n");

	/*
	 set up drive model
	*/
	std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
            .withMotors(
				frontLeftMotorPort,  // Top left
				frontRightMotorPort, // Top right (reversed)
				bottomRightMotorPort, // Bottom right (reversed)
				bottomLeftMotorPort   // Bottom left
			)
            // Green gearset, 4 in wheel diam, 11.5 in wheel track
            .withDimensions({AbstractMotor::gearset::blue, (84.0 / 36.0)}, {{4_in, 13.38_in}, imev5BlueTPR})
			.withMaxVelocity(200)
			.withGains(
				{0.001, 0, 0.00001}, // Distance controller gains
				{0.008, 0, 0.0001}, // Turn controller gains
				{0.001, 0, 0.00001}  // Angle controller gains (helps drive straight)
			)
            .build();
		
	auto xModel = std::dynamic_pointer_cast<XDriveModel>(drive->getModel());
	int round_begin_milliseconds = pros::millis();

	
	/*
	 set up goal coordinates
	*/
	float HighGoalPositionPercent[3];
	if (team == REDTEAM) {
		HighGoalPositionPercent[0] = redHighGoalPosition_percent[0];
		HighGoalPositionPercent[1] = redHighGoalPosition_percent[1];
		HighGoalPositionPercent[2] = redHighGoalPosition_percent[2];
	} else {
		HighGoalPositionPercent[0] = blueHighGoalPosition_percent[0];
		HighGoalPositionPercent[1] = blueHighGoalPosition_percent[1];
		HighGoalPositionPercent[2] = blueHighGoalPosition_percent[2];
	}

	bool shootEnabled = false; // enables shooting

	bool prevShootButtonState = controller.getDigital(ControllerDigital::R2); // record the previous button state


	while (true) {
		// Odom::update_odometry();
		// Odom::debug();

		// float xDist = HighGoalPositionPercent[0]-positionSI.xPercent;
		// float yDist = HighGoalPositionPercent[1]-positionSI.yPercent;
		// float distToGoal = sqrt(xDist*xDist + yDist*yDist);
		// float targetEjectV = clamp(projectile_trajectory::solveVelocity(maxEjectVel, minEjectVel, 0.0001, 20, distToGoal, 45, 0, m, g, p, Av, Ah, Cv, Ch, HighGoalPositionPercent[1], 0.3), minEjectVel, maxEjectVel);

		// flywheel control
		// Flywheel::setLinearEjectVelocity(targetEjectV);

		// locomotion
		// if (Auto::settled) {
			
		// }

		// printf("x=%f, y=%f, a=%f\n", positionSI.x, positionSI.y, positionSI.theta);

		// if ((controller.getAnalog(ControllerAnalog::rightX) != 0 || 
		// 	controller.getAnalog(ControllerAnalog::leftY) != 0 ||
		// 	controller.getAnalog(ControllerAnalog::leftX) != 0 ||
		// 	controller.getDigital(ControllerDigital::R1) ||
		// 	controller.getDigital(ControllerDigital::Y)) && 
		// 	round_begin_milliseconds == 0
		// 	) 
		// {
		// 	round_begin_milliseconds = pros::millis();
		// }

		xModel->xArcade(controller.getAnalog(ControllerAnalog::rightX),
						controller.getAnalog(ControllerAnalog::leftY),
                        controller.getAnalog(ControllerAnalog::leftX)*0.7);

		if (controller.getDigital(ControllerDigital::R1)) {
			RollerMotor.moveVelocity(100);
		} else if (controller.getDigital(ControllerDigital::R2)) {
			RollerMotor.moveVelocity(-100);
		} else {
			RollerMotor.moveVelocity(0);
		}

		// intake
		// if (controller.getDigital(ControllerDigital::B)) {
		// 	Intake::toggle();
		// }

		// aiming
		// if (controller.getDigital(ControllerDigital::down)) {
		// 	if (!Auto::settled) {
		// 		Auto::faceCoordinateAsync(HighGoalPositionPercent[0], HighGoalPositionPercent[1], true);
		// 	}
		// }

		if (controller.getDigital(ControllerDigital::R2)) {
			Gun::shootDisk();
		}

		if (controller.getDigital(ControllerDigital::X)) {
			Gun::reposition();
		}

		// expansion
		if (controller.getDigital(ControllerDigital::Y)) {
			piston.set_value(true); // remember to change the value in autos.cpp
			
		}

		pros::delay(20);
	}
	
}
