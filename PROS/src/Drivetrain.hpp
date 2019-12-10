#include "api.h"
#include "PID.hpp"
#include <vector>

class Drivetrain
{
private:
  pros::Motor frontR(std::vector<int> , pros::motor_gearset_e);
  pros::Motor frontL(std::vector<int> , pros::motor_gearset_e, bool);
  pros::Motor midL(std::vector<int> , pros::motor_gearset_e, bool);
  pros::Motor rearL(std::vector<int> , pros::motor_gearset_e, bool);
  pros::Motor rearR(std::vector<int> , pros::motor_gearset_e, bool);
  pros::Motor midR(std::vector<int> , pros::motor_gearset_e, bool);


  std::vector<pros::Motor *> motors;
  std::vector<PID> pid_controls;
  std::vector<double> pid_coeffs;

  //Motors follow counter clockwise order, starting from frontR

public:
  Drivetrain(std::vector<int> ports, pros::motor_gearset_e gearset);
  void set_power(double p);

};
