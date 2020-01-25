#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

uint8_t imu_port = 19;
pros::Imu *imu;

bool skills = true;
bool red = true;

//preset lift heights in ticks
double bottom = 100;
double one_cube = 1170;
double two_cube = 1970;
double three_cube = 2435;
double low_tower = 3018;
double mid_tower = 3857;
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

	double kp = .09;
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

		printf("output (r, l)t: (%f, %f)\npassed_time: %f\n", r_output, l_output, passed_time);

		if(abs(r_output) > 1) {
			r_output = r_output/abs(r_output);
		}
		if(abs(l_output) > 1) {
			l_output = l_output/abs(l_output);
		}

		if(r_output < 0) {
			lift_r.move(r_output*80);
			lift_l.move(l_output*80);
		}
		else {
			lift_r.move(r_output*127);
			lift_l.move(l_output*127);
		}

		passed_time = passed_time + dt;

		if(passed_time > 6000) {
			break;
		}

		pros::delay(dt);

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

	pros::ADIDigitalOut piston ('A');
	piston.set_value(false);

	printf("initializing\n");
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	imu = new pros::Imu(imu_port);

	imu -> reset();
	pros::delay(2000);

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
	//Intake Motors
	pros::Motor intake_R(4, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(20, pros::E_MOTOR_GEARSET_18, 1);
	//Lift Motors
	pros::Motor lift_R(1, pros::E_MOTOR_GEARSET_36, 1);
	pros::Motor lift_L(10, pros::E_MOTOR_GEARSET_36);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);



	//Drivetrain Setup
	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	double reverse = 1;
	if(!red) {
		reverse = -1;
	}

	if(skills) {
		intake_R.move(127);
		intake_L.move(127);

		drivetrain.drive_inches(24, 50);
		drivetrain.turn_degrees(90*reverse, imu);

		drivetrain.drive_inches(28, 50);
		pros::delay(500);
		drivetrain.turn_degrees(5*reverse, imu);
		drivetrain.drive_inches(14, 20, 1500);
		drivetrain.drive_inches(-10, 50, 1500);
		drivetrain.drive_inches(15, 20, 1500);
		drivetrain.drive_inches(-10, 20, 1500);
		lift_to_position(mid_tower,lift_R,lift_L);
		drivetrain.drive_inches(36, 50, 1500);

		intake_R.move(-127);
		intake_L.move(-127);
		pros::delay(1250);
		intake_R.move(127);
		intake_L.move(127);



		drivetrain.drive_inches(-15, 50, 5000);
		lift_to_position(bottom,lift_R,lift_L);
		drivetrain.drive_inches(-9, 50, 5000);

		drivetrain.turn_degrees(-138*reverse, imu);
		drivetrain.drive_inches(24*sqrt(2), 50, 5000);
		drivetrain.turn_degrees(90*reverse, imu);
		lift_to_position(low_tower,lift_R,lift_L);
		drivetrain.drive_inches(24, 50, 3000);

		intake_R.move(-127);
		intake_L.move(-127);
		pros::delay(1500);
		intake_R.move(0);
		intake_L.move(0);
		//
		//
		// drivetrain.turn_degrees(90*reverse, imu);
		// drivetrain.drive_inches(24, 50, 5000);
		//
		// intake_R.move(127);
		// intake_L.move(127);

		// drivetrain.drive_inches(10, 50, 2500);
		// drivetrain.drive_inches(-10, 50, 2500);
		drivetrain.drive_inches(-36, 50, 2500);

		drivetrain.turn_degrees(-45*reverse, imu);

		drivetrain.drive_inches(90, 50, 2500);
		drivetrain.turn_degrees(90*reverse, imu);
		drivetrain.drive_inches(40, 50, 5000);
		lift_to_position(mid_tower,lift_R,lift_L);

		drivetrain.drive_inches(20, 50, 2500);

		intake_R.move(-127);
		intake_L.move(-127);
		pros::delay(2500);
		intake_R.move(0);
		intake_L.move(0);

		drivetrain.drive_inches(-20, 50, 2500);


	}
	else {
		// Grab first cube on stack
		intake_R.move(127);
		intake_L.move(127);

		drivetrain.drive_inches(22, 50);
		drivetrain.turn_degrees(90*reverse, imu);

		drivetrain.drive_inches(24, 50);
		drivetrain.turn_degrees(45*reverse, imu);
		drivetrain.drive_inches(10, 30, 2500);

		intake_R.move(-127);
		intake_L.move(-127);
		pros::delay(2500);
		intake_R.move(0);
		intake_L.move(0);

		lift_to_position(one_cube,lift_R,lift_L);

		intake_R.move(-127);
		intake_L.move(-127);
		pros::delay(2500);

		drivetrain.drive_inches(-6, 50);


		// drivetrain.drive_inches(22, 50);
		// lift_to_position(two_cube, lift_R, lift_L);
		// drivetrain.drive_inches(12, 25);
		// lift_to_position(two_cube, lift_R, lift_L);
		// pros::delay(500);
		// lift_to_position(two_cube, lift_R, lift_L);
		// drivetrain.drive_inches(-12, 25);
		//
		// // Grab second cube on stack
		// lift_to_position(one_cube, lift_R, lift_L);
		// drivetrain.drive_inches(16, 20);
		// pros::delay(200);
		//
		// // Angling for placing the first cube
		// drivetrain.turn_degrees(14*reverse, imu);
		// lift_to_position(low_tower, lift_R, lift_L);
		// drivetrain.drive_inches(18, 100, 5000);
		// intake_R.move(0);
		// intake_L.move(0);
		// lift_to_position(low_tower, lift_R, lift_L);
		//
		// // Place the first cube
		// intake_R.move(-127);
		// intake_L.move(-127);
		// pros::delay(1000);
		// intake_R.move(0);
		// intake_L.move(0);
		//
		// //Angling for placing the second cube
		// drivetrain.drive_inches(-14, 50);
		// intake_R.move(127);
		// intake_L.move(127);
		// drivetrain.turn_degrees(-19*reverse, imu);
		// drivetrain.drive_inches(-14, 50);
		// drivetrain.turn_degrees(-90*reverse, imu);
		// lift_to_position(mid_tower, lift_R, lift_L);
		// drivetrain.drive_inches(18, 70);
		// intake_R.move(0);
		// intake_L.move(0);
		// lift_to_position(mid_tower, lift_R, lift_L);
		// drivetrain.drive_inches(6, 70, 3000);
		//
		// //Placing the second cube.
		// intake_R.move(-127);
		// intake_L.move(-127);
		// pros::delay(3000);
		// intake_R.move(0);
		// intake_L.move(0);
		//
		// //Backup and stop
		// drivetrain.drive_inches(-24, 50);

	}








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
	pros::Motor lift_R(1, pros::E_MOTOR_GEARSET_36, 1);
	pros::Motor lift_L(10, pros::E_MOTOR_GEARSET_36);
	lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  pros::ADIDigitalOut piston ('A');
	piston.set_value(false);

	std::vector<int> m_ports = {2, 9, 8, 3};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	//master.print(1,1, "" + drivetrain.test());

	bool intaking = false;

	while (true) {

		//Activating precision mode, which divides the power of all movements by 2
		double precision_mult = 1;
		if(master.get_digital(DIGITAL_L2)) {
			precision_mult = .5;
		}

		//Drivetrain control.
		//Both turning and normal driving inputs are placed on an exponential curve (2^(.06*input)-1)
		//This allows for faster and more precise driving.
		double y = master.get_analog(ANALOG_LEFT_Y);
		y = pow(2,.06*abs(y))*abs(y)/y-1;
		double turn = master.get_analog(ANALOG_RIGHT_X);
		double curved_turn = pow(2,.06*abs(turn))*abs(turn)/turn-1;
		if(abs(turn) > 5 ) {
			drivetrain.turn(curved_turn/2);
		}
		else {
			drivetrain.drive(y);
		}

		//Intake control
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


		//Lift Control
		if(master.get_digital(DIGITAL_R1)) {
			lift_R.move(150*precision_mult);
			lift_L.move(150*precision_mult);
		}
		else if(master.get_digital(DIGITAL_R2)) {
			lift_R.move(-70*precision_mult);
			lift_L.move(-70*precision_mult);
		}
		else {
			lift_R.move(0);
			lift_L.move(0);
			lift_R.move_velocity(0);
			lift_L.move_velocity(0);
		}

		if(master.get_digital(DIGITAL_A)) {
			lift_to_position(low_tower, lift_R, lift_L);
		}
		else if(master.get_digital(DIGITAL_B)) {
			lift_to_position(mid_tower, lift_R, lift_L);
		}

		if(master.get_digital(DIGITAL_X)) {
			piston.set_value(true);
		}
		else if(master.get_digital(DIGITAL_Y)) {
			lift_to_position(100,lift_R,lift_L);

		}
		pros::delay(2);

	}
}
