#include "api.h"
#include "PID.hpp"
#include <vector>
#include <math.h>

//Motors follow counter clockwise order, starting from frontR

class Drivetrain
{
private:
  // pros::Motor frontR;
  // pros::Motor frontL(std::vector<int> , pros::motor_gearset_e, bool);
  // pros::Motor rearL(std::vector<int> , pros::motor_gearset_e, bool);
  // pros::Motor rearR(std::vector<int> , pros::motor_gearset_e, bool);


  std::vector<pros::Motor> motors;
  std::vector<PID> pid_controls;
  double kp;
  double ki;
  double kd;
  double e_t;
  double tpr;
  double tpi;
  double wheel_diameter;
  //Motors follow counter clockwise order, starting from frontR

public:
  Drivetrain(std::vector<int> m_ports, pros::motor_gearset_e gearset)
  {
    kp = .2;
    ki = 0;
    kd = 10;
    e_t = 100;
    //green gearbox
    tpr = 900;

    wheel_diameter = 4;
    tpi = tpr/(wheel_diameter*M_PI);
    printf("tpi: %f\n",tpi);


    printf("constructing!\n");
    // pros::Motor *frontR = new pros::Motor(m_ports[0], gearset);
  	// pros::Motor *frontL = new pros::Motor(m_ports[1], gearset, 1);
    // pros::Motor *midL = new pros::Motor(m_ports[2], gearset, 1);
  	// pros::Motor *rearL = new pros::Motor(m_ports[3], gearset, 1);
  	// pros::Motor *rearR = new pros::Motor(m_ports[4], gearset);
    // pros::Motor *midR = new pros::Motor(m_ports[5], gearset);

    pros::Motor frontR (m_ports[0], gearset);
    motors.push_back(frontR);
    pros::Motor frontL (m_ports[1], gearset, 1);
    motors.push_back(frontL);
    pros::Motor rearL (m_ports[2], gearset);
    motors.push_back(rearL);
    pros::Motor rearR (m_ports[3], gearset, 1);
    motors.push_back(rearR);

    printf("%d\n", motors.size());

    for(int i = 0; i < motors.size(); i++)
    {
      PID tmp(kp, ki, kd, e_t);
      pid_controls.push_back( tmp );

    }
  }

  void drive(double p)
  {
    //printf("%d\n",motors.size());

    for(int i = 0; i < motors.size(); i++) {
      motors[i].move(p);
    }

  }

  void turn(double p)
  {
    motors[0].move(-p);
    motors[1].move(p);
    motors[2].move(p);
    motors[3].move(-p);
  }

  void drive_ticks(double ticks)
  {

    printf("ticks: %f\nmotor_pos: %f\n", ticks, motors[0].get_position());
    for(int i = 0; i < motors.size(); i++)
    {
      pid_controls[i].update_target(ticks+motors[i].get_position());
    }

    static double dt = 2;
    static double passed_time = 2;
    while(!check_arrived())
    {

      for(int i = 0; i < motors.size(); i++)
      {
        double output = pid_controls[i].update(motors[i].get_position(), dt);
        double dir = abs(output)/output;
        if(abs(output) >= 1 && passed_time <= 400) {
          output = dir*passed_time/400;
        }
        printf("output: %f\npassed_time: %f\n", output, passed_time);
        if(abs(output) >= 1) {
          output = .7*dir;
        }
        motors[i].move(output*127);
      }

      printf("loop_over!\n");
      passed_time = passed_time+dt;
      pros::delay(dt);
    }
    printf("done moving!\n");
    drive(0);

  }

  void drive_inches(double inches)
  {
    drive_ticks(inches*tpi);
  }
  bool check_arrived()
  {
    bool arrived = true;
    for(int i = 0; i < motors.size(); i++) {
      arrived = arrived && pid_controls[i].check_arrived();
    }
    return arrived;
  }
};
