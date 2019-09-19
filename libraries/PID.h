#include<webots/Robot.hpp>
#pragma once

class PID{
  public:
   PID(webots::Robot *robotInstance);
   void setGains(float P, float I, float D);
   void setTolerance(float tolerance_units);
   void reset();
   double update(double demand, double measurement);
   void setMax(double new_max);
   void ResetKi();
   void setYawCorrection(bool flag);
   ~PID();
  private:
   float Kp, Ki, Kd;
   webots::Robot *robot_instance;
   float max_output, tolerance;
   float last_demand, last_measurement,last_error, integral_error;
   long last_millis;
   double total;
   bool yawCorrection;
};