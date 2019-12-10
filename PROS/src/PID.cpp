#include "PID.hpp"

PID::PID(double p, double i, double d)
{
    setCoeffs(p, i, d);
}

void PID::setCoeffs(double p, double i, double d)
{
    kp = p;
    ki = i;
    kd = d;
}

int PID::get_kp() { return kp; }
int PID::get_ki() { return ki; }
int PID::get_kd() { return kd; }
int PID::get_target() { return target; }

void PID::update_target(double new_target)
{
  target = new_target;
  last_err = 0;
  error_sum = 0;
}

double PID::update(double measure, double dt)
{
  double err = target - measure;
  error_sum += err*dt;
  double output = kp*err + ki*error_sum + kd*(err-last_err)/dt;
  last_err = err;

  return output;
}
