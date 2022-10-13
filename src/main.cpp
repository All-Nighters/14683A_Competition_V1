#include "main.h"
#include "objects/graphics/graphics.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::Vision vision_sensor(1);
	//auto TEST_SIG = pros::Vision::signature_from_utility(1, 9923, 10415, 10169, -1261, -863, -1062, 11.0, 0);
	auto RED_SIG = pros::Vision::signature_from_utility(1, 5327, 9741, 7534, -905, -405, -655, 2, 0);
	auto BLUE_SIG = pros::Vision::signature_from_utility(2, -2991, -2499, -2745, 10223, 15139, 12681, 4.3, 0);
	/*auto RED_SIG = pros::Vision::signature_from_utility(1, 1987, 6583, 4285, -597, 201, -198, 1.3, 0);
	auto BLUE_SIG = pros::Vision::signature_from_utility(2, -2127, -819, -1473, 7139, 12803, 9971, 2.4, 0);*/
	//vision_sensor.set_signature(1, &TEST_SIG);
	vision_sensor.set_signature(1, &RED_SIG);
	vision_sensor.set_signature(2, &BLUE_SIG);
	lv_obj_t* label = Graphics::draw_text(Coordinates(10, 10, 0), "Default Text");
	while (true) {
		//auto rtn = vision_sensor.get_by_sig(0, 1);
		int color_locations[2];
		color_locations[0] = 0;
		color_locations[1] = 0;
		int size_index = -1;
		while (true) {
			size_index++;
			auto object = vision_sensor.get_by_size(size_index);
			//int object_area = object.width * object.height;
			//if (object_area < 50 * 50 || object.signature == 255) break;
			if (object.width < 50 || object.signature == 255) break;
			if (color_locations[object.signature - 1] > 0) continue;
			color_locations[object.signature - 1] = object.top_coord;
		}
		std::string result = "R=" + std::to_string(color_locations[0]) + " B=" + std::to_string(color_locations[1]);
		printf("R=%d B=%d\n", color_locations[0], color_locations[1]);
		lv_label_set_text(label, result.c_str());
		/*for (int i = 0; i < 1; i++) {
			auto rtn = vision_sensor.get_by_size(i);
			printf("#%d: H=%d SIG=%d LC=%d TC=%d\n", i + 1, rtn.height, rtn.signature, rtn.left_coord, rtn.top_coord);
		}*/
		pros::delay(10);
	}
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
void opcontrol() {}

	bool shootEnabled = false; // enables shooting

	bool prevShootButtonState = controller.getDigital(ControllerDigital::R2); // record the previous button state

	while (true) {
		Odom::update_odometry();
		// Odom::test_odometry();

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

		// if (controller.getDigital(ControllerDigital::R2)) {
		// 	pros::Task shoot(trigger);
		// }

		// expansion
		if (controller.getDigital(ControllerDigital::Y)) {
			piston.set_value(true); // remember to change the value in autos.cpp
			
		}

		pros::delay(20);
	}
	
}
>>>>>>> main
