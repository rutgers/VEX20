#include "main.h"
#include "Drivetrain.cpp"
#include <vector>
#include <thread>

double lift_place = 11084;
uint8_t imu_port = 10;
pros::Imu *imu;

//SETTINGS FOR AUTON MODES
bool red = false;
bool skills = false;
bool lowering = false;



void move_lift(pros::Motor lift, double ticks, pros::Controller controller, bool op_control = false) {

	// Coefficients for the PID controller
	double kp = .05;
	double ki = 0;
	double kd = 10;
	double e_t = 50;

	// Creating a new instance of a PID controller and update it with new targets
	PID ctrl(kp, ki, kd, e_t);
	ctrl.update_target(ticks);
	ctrl.update(lift.get_position(),0);

	double dt = 2;

	while(lift.get_position() < 7000) {
		lift.move(127);
		if(op_control && controller.get_digital(DIGITAL_L1)) {
			lift.move(0);
			return;
		}
	}


	while(!ctrl.check_arrived())
	{
		double output = ctrl.update(lift.get_position(), dt);
		printf("lift output: %f\n", output);

		if(abs(output) > 1) {
			output = output/abs(output);
		}
		lift.move(output*70);


		printf("loop_over!\n");
		pros::delay(dt);

		if(op_control && controller.get_digital(DIGITAL_L1)) {
			break;
		}
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

	//ROBOT SETUP

	//The controller for this bot
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	//Intake motors
	pros::Motor intake_R(5, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	//Lift Motor and Setup
	pros::Motor lift(6, pros::E_MOTOR_GEARSET_36);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//Stop Button to prevent lift from lowering too far
	pros::ADIDigitalIn lift_stop('H');
	//Setup for the drivetrain
	std::vector<int> m_ports = {14, 11, 16, 20};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);


	//Reverses turning on the autonomous modes if a preset value (red) is set to true.
	//Allows for both blue and red opmodes to be matching and edited at the same time
	int reverse = -1;
	if(!red) {
		reverse = 1;
	}

	//SKILLS
	if(skills) {

		intake_R.move(255);
		intake_L.move(255);
		drivetrain.drive_inches(108, 60, 10000);

		drivetrain.turn_degrees(90*reverse, imu);

		drivetrain.drive360(0,80*reverse, 0);
		pros::delay(2000);
		drivetrain.drive(0);
		pros::delay(250);
		drivetrain.drive_inches(20, 60, 3000);
		drivetrain.drive_inches(20, 30, 2000);

		intake_R.move(0);
		intake_L.move(0);

		move_lift(lift, lift_place, master);
		pros::delay(1000);

		// Reverse and 180
		drivetrain.drive_inches(-6, 40, 2000);
		drivetrain.drive360(0, -80*reverse, 0);
		pros::delay(500);
		drivetrain.drive(0);
		pros::delay(100);
		drivetrain.turn_degrees(90*reverse, imu);

		drivetrain.drive(-60);
		double passed_time = 0;
		while(!lift_stop.get_value()) {
			if(passed_time > 1000) {
				drivetrain.drive(0);
			}
			lift.move(-127);
			pros::delay(2);
			passed_time += 2;
		}
		lift.move(0);

		drivetrain.drive(-60);
		pros::delay(1000);

		intake_R.move(255);
		intake_L.move(255);
		drivetrain.drive_inches(108, 50, 10000);



		drivetrain.turn_degrees(-90*reverse, imu, 2000);
		drivetrain.drive360(0,-80*reverse, 0);
		pros::delay(2000);
		drivetrain.drive(0);
		pros::delay(250);
		drivetrain.drive_inches(20, 30, 2000);

		intake_R.move(-35);
		intake_L.move(-35);
		pros::delay(500);
		intake_R.move(0);
		intake_L.move(0);

		move_lift(lift, lift_place, master);
		pros::delay(1000);

		// Reverse and 180
		drivetrain.drive_inches(-20, 40, 2000);
	}
	//NORMAL AUTON
	else {

		//AUTONOMOUS

		// Grabbing the first set of cubes
		intake_R.move(255);
		intake_L.move(255);
		lift.move(0);
		drivetrain.drive_inches(38, 50, 4000);

		// Returning to the wall
		drivetrain.drive(-60);
		pros::delay(2500);
		intake_R.move(0);
		intake_L.move(0);
		drivetrain.drive_inches(10,80);

		// Angling for the second set of cubes
		drivetrain.turn_degrees(90*reverse, imu, 3000);
		drivetrain.drive_inches(20, 60, 5000);
		drivetrain.turn_degrees(-90*reverse, imu, 6000);

		// Returning to the wall
		drivetrain.drive(-60);
		pros::delay(1000);

		// Grabbing the second set of cubes
		intake_R.move(255);
		intake_L.move(255);

		drivetrain.drive_inches(25, 50, 5000);


		//TEMP AUTON
		// drivetrain.drive_inches(44, 50, 5000);
		//
		// // Angling for the third set of cubes
		// drivetrain.turn_degrees(90*reverse, imu, 3000);
		//
		// // Grabbing the third set of cubes
		// drivetrain.drive_inches(14, 50, 5000);
		//
		// // Going back to the wall
		// drivetrain.turn_degrees(-20*reverse, imu, 3000);
		// drivetrain.drive_inches(-32, 60, 5000);
		// drivetrain.turn_degrees(-70*reverse, imu, 3000);
		drivetrain.drive(-60);
		pros::delay(4500);

		// Angling for stacking
		drivetrain.drive_inches(6);

		move_lift(lift, 3000, master);
		drivetrain.turn_degrees(-90*reverse, imu, 3000);
		drivetrain.drive360(0, -80*reverse, 0);
		pros::delay(1500);
		drivetrain.drive(0);
		pros::delay(250);
		drivetrain.drive_inches(24, 60, 4000);
		drivetrain.drive_inches(16, 30, 4000);


		// Placing the Stack
		intake_R.move(-35);
		intake_L.move(-35);
		pros::delay(500);
		intake_R.move(0);
		intake_L.move(0);
		move_lift(lift, lift_place, master);
		pros::delay(1000);

		// Reverse and 180
		drivetrain.drive_inches(-24, 40, 2000);

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
	//ROBOT SETUP

	//The controller for this bot
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	//Intake motors
	pros::Motor intake_R(5, pros::E_MOTOR_GEARSET_18);
	pros::Motor intake_L(1, pros::E_MOTOR_GEARSET_18, 1);
	//Lift Motor and Setup
	pros::Motor lift(6, pros::E_MOTOR_GEARSET_36);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//Stop Button to prevent lift from lowering too far
	pros::ADIDigitalIn lift_stop('H');
	//Setup for the drivetrain
	std::vector<int> m_ports = {14, 11, 16, 20};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);


// Loop for op control
	while (true) {

		//Activating position modes, which divides the power of all movements by 2
		double precision_mult = 1;
		if(master.get_digital(DIGITAL_DOWN)) {
			precision_mult = .5;
		}


		//Drivetrain control.
		//Both turning and normal driving inputs are placed on an exponential curve (2^(.06*input)-1)
		//This allows for faster and more precise driving.
		double y = master.get_analog(ANALOG_LEFT_Y);
		double x = master.get_analog(ANALOG_LEFT_X);
		//y = pow(2,.06*abs(y))*abs(y)/y-1;
		//x = pow(2,.06*abs(x))*abs(x)/x-1;
		// if(master.get_digital(DIGITAL_R1)) {
		// 	y = y*.3;
		// 	x = x*.3;
		// }
		double turn = master.get_analog(ANALOG_RIGHT_X);
		//double curved_turn = pow(2,.06*abs(turn))*abs(turn)/turn-1;
		drivetrain.drive360(y,x,turn);

		//Control for the intake motors.
		if(master.get_digital(DIGITAL_R1)) {
			intake_R.move(127*precision_mult);
			intake_L.move(127*precision_mult);

		}
		else if(master.get_digital(DIGITAL_R2)) {
			intake_R.move(-70*precision_mult);
			intake_L.move(-70*precision_mult);
		}
		else {
			intake_R.move(10);
			intake_L.move(10);
		}

		if(master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT)) {
			intake_R.move(-35);
			intake_L.move(-35);
			pros::delay(500);
			intake_R.move(0);
			intake_L.move(0);
			move_lift(lift, lift_place, master, true);
		}

		if(master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_UP)) {
			move_lift(lift, lift_place, master, true);
		}

		//Controls for the lift
		//Checks a button to prevent overcompression of the lift
		if(master.get_digital(DIGITAL_L1)) {
			lift.move(150*precision_mult);
			lowering = false;
		}
		else if(master.get_digital(DIGITAL_L2) && !lift_stop.get_value()) {
			lift.move(-150*precision_mult);
			lowering = false;
		}
		else if(lift_stop.get_value()) {
			lowering = false;
			lift.move(0);
		}
		//Allows for the driver to bring the lift to the bottom without need to be holding a button
		else if((master.get_digital(DIGITAL_B) || lowering) && !lift_stop.get_value()) {
			lowering = true;
			lift.move(-150);
		}
		else if(!lowering) {
			lift.move(0);
		}


		pros::delay(2);
		//pros::lcd::clear();
	}
}
