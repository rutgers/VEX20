#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

double lift_place = 10069;
uint8_t imu_port = 16;
pros::Imu *imu;

//SETTINGS FOR AUTON MODES
bool red = false;
bool skills = true;



void move_lift(pros::Motor lift, double ticks) {

	double kp = .02;
	double ki = 0;
	double kd = 10;
	double e_t = 50;

	PID ctrl(kp, ki, kd, e_t);
	ctrl.update_target(ticks);
	ctrl.update(lift.get_position(),0);

	double dt = 2;
	while(!ctrl.check_arrived())
	{

			double output = ctrl.update(lift.get_position(), dt);
			printf("lift output: %f\n", output);

			if(abs(output) > 1) {
				output = output/abs(output);
			}
			lift.move(output*100);


		printf("loop_over!\n");
		pros::delay(dt);
	}
	printf("done moving!\n");
	lift.move(0);
}

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
 *l to keep execution time for this mode under a few seconds.
 */
void initialize() {
	printf("initializing\n");
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	imu = new pros::Imu(imu_port);
	imu->reset();
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
 * competitionpros::reset(imu_port);-specific initialization routines, such as an autonomous selector
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
 * for non-competitionhttps://www.google.com/search?client=ubuntu&channel=fs&q=call+function+from+pointer&ie=utf-8&oe=utf-8 testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake_R(2, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift(3, pros::E_MOTOR_GEARSET_36, 1);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::ADIDigitalIn lift_stop('A');
	// lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {14, 11, 13, 15};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	int reverse = 1;
	if(!red) {
		reverse = -1;
	}

	//SKILLS
	if(skills) {
		intake_R.move(255);
		intake_L.move(255);
		drivetrain.drive_inches(100, 50, 15000);

		drivetrain.drive(60);
		pros::delay(1000);
		drivetrain.drive(60);

		drivetrain.turn_degrees(-87*reverse, imu);

		drivetrain.drive_inches(29, 40, 5000);

		intake_R.move(-35);
		intake_L.move(-35);120120
		pros::delay(500);
		intake_R.move(0);
		intake_L.move(0);

		move_lift(lift, lift_place);
		pros::delay(1000);

		// Reverse and 180
		drivetrain.drive_inches(-24, 40, 2000);
		drivetrain.turn_degrees(180, imu);
	}
	//NORMAL AUTON
	else {

		intake_R.move(255);
		intake_L.move(255);
		lift.move(0);

		// Grabbing P1
		drivetrain.drive_inches(38, 50, 4000);

		drivetrain.drive(-60);
		pros::delay(2500);
		intake_R.move(0);
		intake_L.move(0);
		drivetrain.drive_inches(10,80);

		// Angling for P2
		drivetrain.turn_degrees(90*reverse, imu, 3000);
		drivetrain.drive_inches(21, 60, 5000);
		drivetrain.turn_degrees(-90*reverse, imu, 3000);

		drivetrain.drive(-60);
		pros::delay(1000);

		intake_R.move(255);
		intake_L.move(255);

		// Grabbing P2
		drivetrain.drive_inches(52, 50, 5000);

		//angling for p3
		drivetrain.turn_degrees(90*reverse, imu, 3000);

		//grabbing p3
		drivetrain.drive_inches(24, 50, 5000);

		//going back to the wall
		drivetrain.drive_inches(-24, 60, 5000);
		drivetrain.turn_degrees(-90*reverse, imu, 3000);
		drivetrain.drive(-60);
		pros::delay(4500);


		//Angling for placing

		drivetrain.drive_inches(6);
		drivetrain.turn_degrees(-96*reverse, imu, 3000);
		drivetrain.drive_inches(29, 40, 5000);

		intake_R.move(-35);
		intake_L.move(-35);
		pros::delay(500);
		intake_R.move(0);
		intake_L.move(0);

		// Placing
		move_lift(lift, lift_place);
		pros::delay(1000);

		// Reverse and 180
		drivetrain.drive_inches(-24, 40, 2000);
		drivetrain.turn_degrees(180, imu);
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
	pros::Motor intake_R(2, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	pros::Motor lift(3, pros::E_MOTOR_GEARSET_36, 1);
	pros::ADIDigitalIn lift_stop('A');
	// lift_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// lift_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {14, 11, 13, 15};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);

	//master.print(1,1, "" + drivetrain.test());

	bool intaking = false;

	while (true) {
		double precision_mult = 1;

		if(master.get_digital(DIGITAL_L2)) {
			precision_mult = .5;
		}

		printf("rotation: %f\n", imu->get_rotation());


		double x = master.get_analog(ANALOG_LEFT_X);
		double y = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		//printf("Lift position: %f\n", lift.get_position());
		//drivetrain.print_position();

		if(turn != 0) {
			drivetrain.turn(turn);
		}
		else {
			drivetrain.drive(y);
		}

		if(master.get_digital(DIGITAL_A)) {
			printf("intake!\n");
			intake_R.move(127*precision_mult);
			intake_L.move(127*precision_mult);
			intaking = true;
		}
		else if(master.get_digital(DIGITAL_B)) {
			printf("outtake!\n");
			intake_R.move(-70*precision_mult);
			intake_L.move(-70*precision_mult);
			intaking = false;
		}
		else if(master.get_digital(DIGITAL_Y) || !intaking){
			printf("STOP TAKING!\n");
			intake_R.move(0);
			intake_L.move(0);
			intaking = false;
		}

		if(master.get_digital(DIGITAL_R1)) {
			intake_R.move(-35);
			intake_L.move(-35);
			pros::delay(500);
			intake_R.move(0);
			intake_L.move(0);
			move_lift(lift, lift_place);
		}


		if(master.get_digital(DIGITAL_UP)) {
			lift.move(150*precision_mult);
		}
		else if(master.get_digital(DIGITAL_DOWN) && !lift_stop.get_value()) {
			lift.move(-150*precision_mult);
		}
		else {
			lift.move(0);
		}

		pros::delay(2);
		//pros::lcd::clear();
	}
}
