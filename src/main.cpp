#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	printf("\nINITIALIZATION STARTED\n");
	printf("======================\n");
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	// // Initialize subsystems
	// Odom::init(BASIC);
	// Odom::set_state(50, 50, 0);
	printf("1. Starting flywheel control loop...");
	Flywheel::startControlLoop();
	printf("OK\n");
	printf("2. Starting flywheel grapher...");
	Flywheel::grapher::start_graphing();
	printf("OK\n");
	// printf("3. Initializing gun...");
	// Gun::init(FORCE_MODE);
	// printf("OK\n");

	// Set motor brake modes
	printf("4. Setting motor breakmode...");
	LFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RFMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	RBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	LBMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	IndexerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	IntakeMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	RollerMotor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	printf("OK\n");
	printf("\nReady to go!\n");
	printf("======================\n\n");
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	printf("\nDISABLED\n");
	printf("======================\n");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	printf("\nCOMPETITION INITIALIZE\n");
	printf("======================\n");
}

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
	printf("\nAUTONOMOUS\n");
	printf("======================\n");
}

void PPTest() {
	Odom::set_state(0_m, 0_m, 0_deg);
	std::vector<Waypoint> pathway;
	
	// insert evenly distributed waypoints from (0, 0) to (40, 0)
	for (int i = 0; i < 11; i++) {
		pathway.push_back(Waypoint(4*i, 0, 150));
	}

	// insert evenly distributed waypoints from (40, 0) to (40, 40)
	for (int i = 0; i < 11; i++) {
		pathway.push_back(Waypoint(40, 4*i, 150));
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

	printf("\nDRIVER CONTROL\n");
	printf("======================\n");


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
		
	int round_begin_milliseconds = pros::millis();
	auto xModel = std::dynamic_pointer_cast<XDriveModel>(drive->getModel());

	
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

	Flywheel::setLinearEjectVelocity(8);

	printf("Driver configuration finished\n");


	/*
	For controls, see globals.hpp
	*/
	while (true) {

		// locomotion
		drive->getModel()->arcade(controller.getAnalog(ForwardAxis), 0.5*controller.getAnalog(TurnAxis));

		// spin roller
		if (controller.getDigital(RollerDownButton)) {
			printf("Rolling down\n");
			RollerMotor.moveVoltage(6000);
		} else if (controller.getDigital(RollerUpButton)) {
			printf("Rolling up\n");
			RollerMotor.moveVoltage(-6000);
		} else {

			// spin intake
			if (controller.getDigital(IntakeButton)) {
				printf("Running Intake\n");
				Intake::turnOn();
			} else {
				Intake::turnOff();
			}
		}

		// high goal facing
		if (controller.getDigital(AimButton)) {
			if (!Auto::settled) {
				printf("Facing high goal\n");
				Auto::faceCoordinateAsync(HighGoalPositionPercent[0], HighGoalPositionPercent[1], true);
			}
		}

		// shoot disk
		if (controller.getDigital(ShootButton)) {
			printf("Shooting disk\n");
			Gun::shootDisk();
		}


		// expansion
		if (pros::millis()-round_begin_milliseconds >= 105 * 1000 && controller.getDigital(ExpansionButton)) {
			printf("Trigger expansion\n");

			// remember to change the value in autos.cpp
			piston1.set_value(true); 
			piston2.set_value(true);
		}

		pros::delay(20);
	}
	
}
