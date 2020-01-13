#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

//preset lift heights in ticks
double floor = 0;
double one_cube = 0;
double two_cube = 0;
double three_cube = 0;
double low_tower = 0;
double mid_tower = 0;
double high_tower = 0;

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
	pros::Motor intake_R(4, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(20, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift_R(1, pros::E_MOTOR_GEARSET_36);
	pros::Motor lift_L(10, pros::E_MOTOR_GEARSET_36, 1);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	intake_R.move(127);
	intake_L.move(127);

	drivetrain.drive_inches(24);
	





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
	pros::Motor intake_R(4, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(20, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift_R(1, pros::E_MOTOR_GEARSET_36);
	pros::Motor lift_L(10, pros::E_MOTOR_GEARSET_36, 1);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	//master.print(1,1, "" + drivetrain.test());

	bool intaking = false;

	while (true) {
		double precision_mult = 1;

		if(master.get_digital(DIGITAL_L2)) {
			precision_mult = .5;
		}


		double x = master.get_analog(ANALOG_LEFT_X);
		double y = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		drivetrain.print_position();

		if(turn != 0) {
			drivetrain.turn(turn);
		}
		else {
			drivetrain.drive(y);
		}

		if(master.get_digital(DIGITAL_UP)) {
			printf("intake!\n");
			intake_R.move(127*precision_mult);
			intake_L.move(127*precision_mult);
			intaking = true;
		}
		else if(master.get_digital(DIGITAL_DOWN)) {
			printf("outtake!\n");
			intake_R.move(-70*precision_mult);
			intake_L.move(-70*precision_mult);
			intaking = false;
		}
		else if(master.get_digital(DIGITAL_LEFT) || !intaking){
			printf("STOP TAKING!\n");
			intake_R.move(0);
			intake_L.move(0);
			intaking = false;
		}

		if(master.get_digital(DIGITAL_R1)) {
			lift_R.move(150*precision_mult);
			lift_L.move(150*precision_mult);
		}
		else if(master.get_digital(DIGITAL_R2)) {
			lift_R.move(-150*precision_mult);
			lift_L.move(-150*precision_mult);
		}
		else {
			lift_R.move(0);
			lift_L.move(0);
		}

		pros::delay(2);
		//pros::lcd::clear();

	}
}
