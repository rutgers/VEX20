#include "Drivetrain.hpp"
#include <vector>

//Motors follow counter clockwise order, starting from frontR


Drivetrain::Drivetrain(std::vector<int> m_ports, pros::motor_gearset_e gearset)
{
  std::vector<double> pid_coeffs = {.75, 0, 0};

  // pros::Motor *frontR = new pros::Motor(m_ports[0], gearset);
	// pros::Motor *frontL = new pros::Motor(m_ports[1], gearset, 1);
  // pros::Motor *midL = new pros::Motor(m_ports[2], gearset, 1);
	// pros::Motor *rearL = new pros::Motor(m_ports[3], gearset, 1);
	// pros::Motor *rearR = new pros::Motor(m_ports[4], gearset);
  // pros::Motor *midR = new pros::Motor(m_ports[5], gearset);

  pros::Motor frontR (m_ports[0], gearset);
  pros::Motor frontL (m_ports[1], gearset, 1);
  pros::Motor rearL (m_ports[2], gearset, 1);
  pros::Motor rearR (m_ports[3], gearset);


//   std::vector<pros::Motor *> motors = {frontR, frontL, midL, rearL, rearR, midR};
//
//   std::vector<PID> pid_controls = {};
//   for(int i = 0; i < 6; i++)
//   {
//     pid_controls.push_back(PID(pid_coeffs[0],pid_coeffs[1],pid_coeffs[2]));
//   }
}

// void Drivetrain::set_power(double p)
// {
//   for(int i = 0; i < motors.size(); i++) {
//     motors[i] -> move(p);
//   }
// }
