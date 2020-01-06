#include "main.h"
#include "Drivetrain.cpp"
#include <vector>

double lift_place = 9589;

void move_lift(pros::Motor lift, double ticks) {

	double kp = .2;
	double ki = 0;
	double kd = 10;
	double e_t = 100;

	PID ctrl(kp, ki, kd, e_t);
	ctrl.update_target(ticks+lift.get_position());
	ctrl.update(lift.get_position(),0);

	double dt = 2;
	while(!ctrl.check_arrived())
	{

			double output = ctrl.update(lift.get_position(), dt);
			printf("lift output: %f\n", output);
			lift.move(output*127);


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

	printf("STARTING AUTO\n\n\n\n\n\n\n\n\n\n\n\n\n");
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

	//chnge to build each side
	bool red = true;
	dir = -1;

	if(red) {
		dir = 1;
	}

	intake_R.move(255);
	intake_L.move(255);
	lift.move(0);

	// Grabbing P1
	drivetrain.drive_inches(38, 40, 5000);
	//
	// // Grabbing Green Cube
	// drivetrain.turn_degrees(30, 50, 10000);
	// drivetrain.drive_inches(8*sqrt(2), 50, 5000);
	//
	// // Going back to the wall
	// drivetrain.turn_degrees(-30, 50, 10000);
	drivetrain.strafe(-12*dir, 40, 2000);
	drivetrain.drive_inches(10, 40, 2000);
	drivetrain.drive(-60);
	pros::delay(3000);
	intake_R.move(10);
	intake_L.move(10);
	drivetrain.drive_inches(10,80, 5000);

	drivetrain.strafe(12*dir, 40, 3000);
	drivetrain.drive(-60);
	pros::delay(5000);
	drivetrain.drive(0);

	intake_R.move(255);
	intake_L.move(255);

	// Grabbing P2
	drivetrain.drive_inches(42, 50, 5000);

	intake_R.move(10);
	intake_L.move(10);

	//Angling for placing
	drivetrain.drive(-60);
	pros::delay(3500);
	drivetrain.drive_inches(6);
	drivetrain.turn_degrees(-105*dir, 70, 5000);
	drivetrain.drive_inches(33, 50, 5000);

	// Placing
	move_lift(lift, lift_place);
	pros::delay(1000);

	// Reverse and 180
	drivetrain.drive_inches(-24, 40, 2000);
	drivetrain.turn_degrees(180*dir, 80);

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
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	std::vector<int> m_ports = {14, 11, 13, 15};
	Drivetrain drivetrain (m_ports, pros::E_MOTOR_GEARSET_18);



	//master.print(1,1, "" + drivetrain.test());

	bool intaking = false;

	while (true) {
		double precision_mult = 1;

		if(master.get_digital(DIGITAL_DOWN)) {
			precision_mult = .5;
		}


		double drive = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X);

		double drive_right = drive-turn;
		double drive_left = drive+turn;

		drive_right = (pow(2,.067*abs(drive_right))-1)*abs(drive_right)/drive_right;
		drive_left = (pow(2,.067*abs(drive_left))-1)*abs(drive_left)/drive_left;

		if(drive_right < 1 && drive_right > -1) {
			drive_right = 0;
		}
		if(drive_left < 1 && drive_left > -1) {
			drive_left = 0;
		}

		drivetrain.drive_each({drive_right, drive_left, drive_left, drive_right});

		//printf("Lift position: %f\n", lift.get_position());
		//drivetrain.print_position();

		if(master.get_digital(DIGITAL_R1)) {
			printf("intake!\n");
			intake_R.move(127*precision_mult);
			intake_L.move(127*precision_mult);
			//intaking = true;
		}
		else if(master.get_digital(DIGITAL_R2)) {
			printf("outtake!\n");
			intake_R.move(-70*precision_mult);
			intake_L.move(-70*precision_mult);
			//intaking = false;
		}
		else {
			//printf("STOP TAKING!\n");
			intake_R.move(10);
			intake_L.move(10);
			//intaking = false;
		}

		if(master.get_digital(DIGITAL_L1)) {
			lift.move(100*precision_mult);
		}
		else if(master.get_digital(DIGITAL_L2) && !lift_stop.get_value()) {
			lift.move(-150*precision_mult);
		}
		else {
			lift.move(0);
		}

		if(master.get_digital(DIGITAL_LEFT)) {
			drivetrain.turn_degrees(90, 80, 5000);
		}
		else if(master.get_digital(DIGITAL_RIGHT)) {
			drivetrain.turn_degrees(-90, 80, 5000);
		}

		pros::delay(2);
		//pros::lcd::clear();
	}
}
