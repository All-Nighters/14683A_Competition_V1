#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
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

	pros::lcd::register_btn1_cb(on_center_button);
	Odom::tare_odometry();
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
void autonomous() {}

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

	printf("Hello Allnighters\n");

	float upper = 8; // 8 m/s maximum
	float lower = 1; // 1m/s minimum
	float step = 0.00001;
	float epoch = 20;
	float xPos = 1.9; // 2m from the launcher
	float yPos = 0.76835; // 0.76835m above ground
	float a = 45; // 45 deg launcher
	float Vh = 0;
	float m = 0.06;
	float g = 9.81;
	float p = 1.225;
	float Av = 0.015393804;
	float Ah = 0.0028;
	float Cv = 5;
	float Ch = 5;
	float launcher_height = 0.1697;




	float targetV = projectile_trajectory::solveVelocity(upper, lower, step, epoch, xPos, a, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height);
	float flywheelV = Flywheel::getCurrentVelocity();

	while (true) {
		// FlywheelMotor1.moveVelocity(600);
		// printf("Actual rpm: %f\n", flywheelV);
		// Flywheel::spinVelocityRPM(2000);
		// Flywheel::setLinearEjectVelocity(6);

		
		// printf("%f\n", targetV);
		// if (targetV == -1) {
		// 	Flywheel::setLinearEjectVelocity(10);
		// } else {
		// 	Flywheel::setLinearEjectVelocity(targetV);
		// }
		// Flywheel::grapher::graph_velocity(3600, flywheelV);
		float upper = 8; // 8 m/s maximum
		float lower = 1; // 1m/s minimum
		float step = 0.00001;
		float epoch = 20;
		float xPos = 2.3; // 2m from the launcher
		float yPos = 0.76835; // 0.76835m above ground
		float a = 45; // 45 deg launcher
		float Vh = 0;
		float m = 0.06;
		float g = 9.81;
		float p = 1.225;
		float Av = 0.015393804;
		float Ah = 0.0028;
		float Cv = 3.25;
		float Ch = 3.25;
		float launcher_height = 0.1697;




		float targetV = projectile_trajectory::solveVelocity(upper, lower, step, epoch, xPos, a, Vh, m, g, p, Av, Ah, Cv, Ch, yPos, launcher_height);
		float flywheelV = Flywheel::getCurrentVelocity();
		Flywheel::setLinearEjectVelocity(targetV);
		Flywheel::grapher::graph_velocity(Flywheel::getExpectRPMFromEjectVelocity(targetV), flywheelV);
		printf("%f\n", targetV);
		pros::delay(20);
	}
	
}
