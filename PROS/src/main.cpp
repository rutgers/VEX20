#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

uint8_t imu_port = 19;
pros::Imu *imu;

//preset lift heights in ticks
double bottom = 0;
double one_cube = 1170;
double two_cube = 2070;
double three_cube = 2435;
double low_tower = 2518;
double mid_tower = 3757;
double high_tower = 3757;

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

void lift_to_position(double pos, pros::Motor lift_r, pros::Motor lift_l) {

	double kp = .0045;
	double ki = 0;
	double kd = -.05;
	double e_t = 50;
	PID r_control(kp, ki, kd, e_t);
	PID l_control(kp, ki, kd, e_t);

	r_control.update_target(pos);
	l_control.update_target(pos);

	double dt = 2;
	double passed_time = 0;
	while(!r_control.check_arrived() && !l_control.check_arrived()) {
		double r_output = r_control.update(lift_r.get_position(), dt);
		double l_output = l_control.update(lift_l.get_position(), dt);

		printf("outpu (r, l)t: (%f, %f)\npassed_time: %f\n", r_output, l_output, passed_time);

		lift_r.move(r_output);
		lift_l.move(l_output);
		passed_time = passed_time + dt;
	}
	lift_r.move(0);
	lift_l.move(0);
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
	imu = new pros::Imu(imu_port);

	imu -> reset();

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
	pros::Motor lift_R(10, pros::E_MOTOR_GEARSET_36);
	pros::Motor lift_L(1, pros::E_MOTOR_GEARSET_36, 1);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	//grabbing first cube
	intake_R.move(127);
	intake_L.move(127);
	drivetrain.drive_inches(24);


	//grab first cube on stack
	lift_to_position(two_cube, lift_R, lift_L);
	drivetrain.drive_inches(6);
	drivetrain.drive_inches(-6);

	//grab second cube on stack
	lift_to_position(one_cube, lift_R, lift_L);
	drivetrain.drive_inches(6);

	intake_R.move(0);
	intake_L.move(0);

	//positioning to place fist cube
	drivetrain.turn_degrees(90);
	drivetrain.drive_inches(12);
	drivetrain.turn_degrees(-90);
	drivetrain.drive_inches(24);
	lift_to_position(low_tower, lift_R, lift_L);

	//placing first cube
	intake_R.move(-70);
	intake_L.move(-70);
	pros::delay(600);
	intake_R.move(0);
	intake_L.move(0);

	//positioning to place second cube
	drivetrain.drive_inches(-42);
	drivetrain.turn_degrees(-90);
	drivetrain.drive_inches(36);
	lift_to_position(mid_tower, lift_R, lift_L);

	//placing second cube
	drivetrain.drive_inches(6);
	intake_R.move(-70);
	intake_L.move(-70);
	pros::delay(600);
	intake_R.move(0);
	intake_L.move(0);







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
	pros::Motor lift_R(10, pros::E_MOTOR_GEARSET_36);
	pros::Motor lift_L(1, pros::E_MOTOR_GEARSET_36, 1);
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

		//drivetrain.print_position();
		printf("Lift positions (r, l): (%f, %f)\n", lift_R.get_position(), lift_L.get_position());

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
			intake_R.move(-127*precision_mult);
			intake_L.move(-127*precision_mult);
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
			lift_R.move_velocity(0);
			lift_L.move_velocity(0);
		}

		pros::delay(2);
		//pros::lcd::clear();

	}
}
