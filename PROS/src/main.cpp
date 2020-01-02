#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

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
	printf("initializing\n");
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

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
	std::vector<int> m_ports = {19, 12, 11, 20};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	drivetrain.drive_ticks(-3600);

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
	printf("beginning control\n");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor lift_R(5, pros::E_MOTOR_GEARSET_36);
	pros::Motor lift_L(10, pros::E_MOTOR_GEARSET_36, 1);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	pros::Motor claw(21, pros::E_MOTOR_GEARSET_18, 1);

	//master.print(1,1, "" + drivetrain.test());

	while (true) {


		double turn = master.get_analog(ANALOG_LEFT_X);
		double y = master	.get_analog(ANALOG_LEFT_Y);
		double lift = master.get_analog(ANALOG_RIGHT_Y);

		pros::lcd::print(1, "y: %d\n", y);
		pros::lcd::print(2, "turn: %d\n", turn);

		if(turn != 0) {
			drivetrain.turn(turn);
		}
		else {
			drivetrain.drive(y);
		}

		lift_R.move(lift);
		lift_L.move(lift);

		if(master.get_digital(DIGITAL_A)) {
			claw.move(255);
		}
		else if(master.get_digital(DIGITAL_B)) {
			claw.move(-255);
		}
		else {
			claw.move(0);
		}

		pros::delay(2);
		//pros::lcd::clear();
	}
}
