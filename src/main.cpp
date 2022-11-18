#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	GraphicalInterface selector;
	printf("\nINITIALIZATION STARTED\n");
	printf("======================\n");

	// // Initialize subsystems
	Odom::init(BASIC);
	// Odom::set_state(50, 50, 0);
	printf("1. Starting flywheel control loop...");
	Flywheel::startControlLoop();
	printf("OK\n");
	printf("2. Starting flywheel grapher...");
	Flywheel::grapher::start_graphing();
	printf("OK\n");
	printf("3. Initializing gun...");
	Gun::init(FORCE_MODE);
	printf("OK\n");

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
 * Read user configuration of the match. Reads start position, auton strategy, 
 * and team color.
 */
void readSelectorConfiguration() {
	printf("Match Configuration:\n");
	std::string autoName = "";
	
	GraphicalInterface::InterfaceSelector position = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_POSITION);
	GraphicalInterface::InterfaceSelector game_team = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_TEAM);
	GraphicalInterface::InterfaceSelector mode = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_MODE);
	GraphicalInterface::InterfaceSelector game_round = GraphicalInterface::get_selector(GraphicalInterface::InterfaceConfiguration::GAME_ROUND);

	GraphicalInterface::InterfaceSelector select_combinations[13][4] = {
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_SCORE}, // red first scoring
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_SUPPORT}, // red first supportive
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_IDLE}, // red second idle
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_SCORE}, // red second scoring
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_SUPPORT}, // red second supportive
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_RED, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_IDLE}, // red second idle

		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_SCORE}, // red first scoring
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_SUPPORT}, // red first supportive
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_1, GraphicalInterface::SELECTOR_MODE_IDLE}, // red second idle
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_SCORE}, // red second scoring
		{GraphicalInterface::SELECTOR_ROUND_AUTONOMOUS, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_SUPPORT}, // red second supportive
		
		{GraphicalInterface::SELECTOR_ROUND_SKILL, GraphicalInterface::SELECTOR_TEAM_BLUE, GraphicalInterface::SELECTOR_POSITION_2, GraphicalInterface::SELECTOR_MODE_IDLE}, // skill
	};

	AutoProcedure procedure_combinations[13] = {
		RED_FIRST_SCORING,
		RED_FIRST_SUPPORTIVE,
		RED_FIRST_IDLE,
		RED_SECOND_SCORING,
		RED_SECOND_SUPPORTIVE,
		RED_SECOND_IDLE,

		BLUE_FIRST_SCORING,
		BLUE_FIRST_SUPPORTIVE,
		BLUE_FIRST_IDLE,
		BLUE_SECOND_SCORING,
		BLUE_SECOND_SUPPORTIVE,
		BLUE_SECOND_IDLE,

		SKILL
	};

	GraphicalInterface::InterfaceSelector current_config[] = {game_round, game_team, position, mode};

	int procedure_idx = -1;
	if (current_config[0] == GraphicalInterface::SELECTOR_ROUND_SKILL) {
		auto_procedure_running = SKILL;
	} else {
		for (int i = 0; i < sizeof(select_combinations) / sizeof(select_combinations[0]); i++) {
			if (select_combinations[i][0] == current_config[0] &&
				select_combinations[i][1] == current_config[1] &&
				select_combinations[i][2] == current_config[2] &&
				select_combinations[i][3] == current_config[3]
			) {
				procedure_idx = i;
				break;
			}
		}
		auto_procedure_running = procedure_combinations[procedure_idx];
	}	
	if (procedure_idx == 0 || procedure_idx == 1 || procedure_idx == 2) { // red first
		Odom::set_state(86.38888888888889, 76.2037037037037, 180);
	}
	else if (procedure_idx == 3 || procedure_idx == 4 || procedure_idx == 5) { // red second
		Odom::set_state(58.05, 10.41, 90);
	}
	else if (procedure_idx == 6 || procedure_idx == 7 || procedure_idx == 8) { // blue first
		Odom::set_state(10.10, 24.35, 0);
	}
	else if (procedure_idx == 9 || procedure_idx == 10 || procedure_idx == 11) { // blue second
		Odom::set_state(41.9, 90.95, -90);
	}
	else if (procedure_idx == 12) { // skill
		Odom::set_state(0, 0, 0);
	}
	
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
	readSelectorConfiguration();

	// // position #1 auto
	// if (auto_procedure_running == RED_FIRST_SCORING || 
	// 	auto_procedure_running == BLUE_FIRST_SCORING || 
	// 	auto_procedure_running == RED_FIRST_SUPPORTIVE ||
	// 	auto_procedure_running == BLUE_FIRST_SUPPORTIVE 
	// ) {
	// 	// roller
	// 	LFMotor.moveVelocity(-200);
	// 	LBMotor.moveVelocity(-200);
	// 	RFMotor.moveVelocity(-200);
	// 	RBMotor.moveVelocity(-200);
	// 	pros::delay(300);
	// 	LFMotor.moveVelocity(0);
	// 	LBMotor.moveVelocity(0);
	// 	RFMotor.moveVelocity(0);
	// 	RBMotor.moveVelocity(0);
	// }
	
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
	readSelectorConfiguration(); // remember to delete this in production

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

	float prev_error = 0;

	printf("Driver configuration finished\n");

	bool prevButtonState = false;
	bool isSpinning = false;
	bool isEndGame = false;
	// Flywheel::setLinearEjectVelocity(6);

	/*
	For controls, see globals.hpp
	*/
	while (true) {
		Odom::debug();
		

		// // Align to the high goal when pressing the button
		// if (controller.getDigital(AimButton)) {
		// 	printf("Facing high goal\n");
			
		// 	/*
		// 	Converting x and y distance to angle to face
		// 	*/
		// 	float xDist = HighGoalPositionPercent[0] - positionSI.xPercent;
		// 	float yDist = HighGoalPositionPercent[1] - positionSI.yPercent;

		// 	if (abs(xDist) < 0.1 && abs(yDist) < 0.1) {
		// 		return;
		// 	}

		// 	float relativeAngle;

		// 	if (xDist > 0 && yDist > 0) { // first quadrant
		// 		relativeAngle = atan(abs(yDist/xDist)) * 180 / M_PI;
		// 	}
		// 	else if (xDist > 0 && yDist < 0) { // second quadrant
		// 		relativeAngle = -atan(abs(yDist/xDist)) * 180 / M_PI;
		// 	}
		// 	else if (xDist < 0 && yDist < 0) { // third quadrant
		// 		relativeAngle = -180 + (atan(abs(yDist/xDist)) * 180 / M_PI);
		// 	}
		// 	else if (xDist < 0 && yDist > 0) { // fourth quadrant
		// 		relativeAngle = 180 - (atan(abs(yDist/xDist)) * 180 / M_PI);
		// 	}
		// 	else if (xDist == 0 && yDist != 0) {
		// 		relativeAngle = (yDist / abs(yDist))*90;
		// 	}
		// 	else if (xDist != 0 && yDist == 0) {
		// 		relativeAngle = 0;
		// 	} else {
		// 		return;
		// 	}

		// 	// angle to face to aim the goal
		// 	float faceAngle = formatAngle((relativeAngle - positionSI.theta) + aimAngleDeviation);

		// 	/*
		// 	Rotate to angle
		// 	*/
		// 	float target_angle = positionSI.theta + faceAngle;
		// 	prev_error = Auto::directionPIDStep(target_angle, prev_error);
		// } 
		
		// else {
		// 	prev_error = 0;
		// 	// locomotion
		// 	drive->getModel()->arcade(controller.getAnalog(ForwardAxis), 0.5*controller.getAnalog(TurnAxis));
		// }

		drive->getModel()->arcade(controller.getAnalog(ForwardAxis), 0.5*controller.getAnalog(TurnAxis));

		// spin intake
		if (controller.getDigital(IntakeButton)) {
			printf("Running Intake\n");
			Intake::turnOn();
		} else if (controller.getDigital(IntakeButtonRev)) {
			printf("Running Intake\n");
			Intake::turnOnRev();
		} else {
			Intake::turnOff();
		}

		// shoot disk
		if (controller.getDigital(ShootButton)) {
			printf("Shooting disk\n");
			Gun::shootDisk();
		}


		// triple shoot disk
		if (controller.getDigital(TripleShootButton)) {
			printf("Shooting 3 disks\n");
			Gun::shootDisk(3, FORCE_MODE);

			Flywheel::setLinearEjectVelocity(0);
			isSpinning = false;
		}

		if (controller.getDigital(FlywheelStopButton) && !prevButtonState) {
			prevButtonState = true;
			if (isSpinning) {
				Flywheel::setLinearEjectVelocity(0);
				isSpinning = false;
			} else {
				Flywheel::setLinearEjectVelocity(5);
				isSpinning = true;
			}
			
			
		} else if (!controller.getDigital(FlywheelStopButton)) {
			prevButtonState = false;
		}


		if (pros::millis()-round_begin_milliseconds >= 90 * 1000 && !isEndGame) {
			controller.clearLine(0);
			controller.setText(0,0,"Endgame");
			isEndGame = true;
		} else {
			controller.setText(0,0, isSpinning? "SPIN" : "STOP");
		}


		if (pros::millis()-round_begin_milliseconds >= 90 * 1000 && controller.getDigital(ExpansionButton)) {
			printf("Trigger expansion\n");

			// remember to change the value in autos.cpp
			piston1.set_value(true); 
			piston2.set_value(true);
		}

		pros::delay(20);
	}
	
}
