#include "PID.hpp"
#include <cstdio>
#include <cstdlib>

PID::PID(double& p, double& i, double& d, double& e_t)
{
    setCoeffs(p, i, d, e_t);
    update_target(0);
}

void PID::setCoeffs(double p, double i, double d, double e_t)
{
    kp = p;
    ki = i;
    kd = d;
    err_thresh = e_t;
}

double PID::get_kp() { return kp; }
double PID::get_ki() { return ki; }
double PID::get_kd() { return kd; }
double PID::get_target() { return target; }

void PID::update_target(double new_target)
{
  target = new_target;
  last_err = new_target;
  error_sum = 0;
}

double PID::update(double measure, double dt)
{
  double err = target - measure;
  printf("%f\n", err);
  error_sum += err*dt;
  double output = kp*err + ki*error_sum + kd*(err-last_err)/dt;
  last_err = err;

  return output;
}

bool PID::check_arrived()
{
  return abs(last_err) < err_thresh;
}
