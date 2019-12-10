#include "api.h"
#include "PID.hpp"

class Drivetrain
{
private:
  pros::Motor rearR;
  pros::Motor rearL;
  pros::Motor frontR;
  pros::Motor frontL;
  std::list<pros::Motor> motors;
  std::list<PID> pid_controls;
  static std::list<double> pid_coeffs;


  //Motors follow counter clockwise order, starting from frontR

public:
  Drivetrain(std::list<int> ports, pros::motor_gearset_e, gearset)

}
