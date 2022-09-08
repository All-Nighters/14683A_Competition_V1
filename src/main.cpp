#include "main.h"

/**
 * Moves the piston to push the disks to the flywheel
 *
 */
void trigger() {
	indexer.set_value(true);
	pros::delay(100);
	indexer.set_value(false);
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

	Odom::tare_odometry();

	LFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	LBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

	
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
	// Auto::turnAngle(90);
	Autos::run("minimum");
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

	autonomous();

	printf("Hello Allnighters\n");

	/*
	 set up drive model
	*/
	std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
            .withMotors(
				-1,  // Top left
				2, // Top right (reversed)
				3, // Bottom right (reversed)
				-4   // Bottom left
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
	if (teamColor == 0) {
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
		Odom::update_odometry();

		// float xDist = HighGoalPositionPercent[0]-positionSI.xPercent;
		// float yDist = HighGoalPositionPercent[1]-positionSI.yPercent;
		// float distToGoal = sqrt(xDist*xDist + yDist*yDist);
		// float targetEjectV = clamp(projectile_trajectory::solveVelocity(maxEjectVel, minEjectVel, 0.0001, 20, distToGoal, 45, 0, m, g, p, Av, Ah, Cv, Ch, HighGoalPositionPercent[1], 0.3), minEjectVel, maxEjectVel);

		// flywheel control
		// Flywheel::setLinearEjectVelocity(targetEjectV);

		// locomotion
		// if (!Auto::settled) {
			
		// }

		// printf("x=%f, y=%f, a=%f\n", positionSI.x, positionSI.y, positionSI.theta);

		xModel->xArcade(-controller.getAnalog(ControllerAnalog::rightX),
						-controller.getAnalog(ControllerAnalog::leftY),
                        -controller.getAnalog(ControllerAnalog::leftX));

		if (controller.getDigital(ControllerDigital::R1)) {
			RollerMotor.moveVelocity(200);
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

		// expansion
		if (controller.getDigital(ControllerDigital::R2)) {
			pros::Task shoot(trigger);
		}

		if (controller.getDigital(ControllerDigital::Y) && pros::millis() - round_begin_milliseconds >= 50 * 1000) {
			piston.set_value(false);
		}

		pros::delay(20);
	}
	
}
