#include "api.h"
#include "PID.hpp"
#include "Drivetrain.hpp"

//Motors follow counter clockwise order, starting from frontR

static std::list<double> pid_coeffs = {.75, 0, 0}

Drivetrain::Drivetrain(std::list<int> ports, pros::motor_gearset_e gearset)
{
  pros::Motor frontR(ports[0], pros::E_MOTOR_GEARSET_36);
	pros::Motor frontL(ports[1], pros::E_MOTOR_GEARSET_36, 1);
	pros::Motor rearL(ports[2], pros::E_MOTOR_GEARSET_36, 1);
	pros::Motor rearR(ports[3], pros::E_MOTOR_GEARSET_36);



  std::list<PID> pid_controls;
  for(int i = 0; i < 4; i++)
  {
    pid_controls.push_back(PID(pid_coeffs[0],pid_coeffs[1],pid_coeffs[2]))
  }
}

void Drivetrain::set_power(double p)
{

}
