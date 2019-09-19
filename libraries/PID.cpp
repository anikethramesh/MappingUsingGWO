#include <webots/Robot.hpp>
#include <cmath>
#include <iostream>
#include "PID.h"

using namespace webots;

PID::PID(Robot *robotInstance){
  max_output=255; 
  total=0;
  last_demand=0; //For storing the previous input
  last_measurement=0; //For storing the last measurement
  last_error=0; //For calculating the derivative term
  integral_error=0; //For storing the integral of the error
  robot_instance = robotInstance;
  last_millis = 1000*robot_instance->getTime();
  total = 0;
  yawCorrection = false;
}

void PID::ResetKi(){
   last_error=0;
   integral_error=0;
}

void PID::setYawCorrection(bool flag){
  yawCorrection = flag;
}
void PID::setGains(float P, float D, float I){
    Kp = P;
    Kd = D;
    Ki = I;
}

void PID::setTolerance(float tolerance_units){
  tolerance = tolerance_units;
}

void PID::reset(){
  last_error = 0;
  integral_error = 0;
  last_millis = 1000*robot_instance->getTime();
}

double PID::update(double demand, double measurement){
    //Calculate how much time (in milliseconds) has passed since the last update call
    long time_now = 1000*robot_instance->getTime();
    float time_delta = (float)(time_now - last_millis);
    last_millis = time_now;  
    //This represents the error term
    float error = demand - measurement;
    if(yawCorrection){
      if(error > M_PI)
         error = error - (2*M_PI);
      else if(error < -M_PI)
         error = error + (2*M_PI);
    }
    //This represents the error derivative
    float error_delta = (last_error - error) / time_delta;  

    //Update storage
    last_demand = demand;
    last_measurement = measurement;
    last_error = error;  
    
    integral_error += (error * time_delta);

    //Add the three components to get the total output
    total = (Kp*error) + (Kd*error_delta) + (Ki*integral_error);
    //Make sure we don't exceed the maximum output
    // total = constrain( total, -max_output, max_output );
    if(total>=max_output)
      total = max_output;
    else if(total<-max_output)
      total = -max_output;
    
    if(std::abs(error)<tolerance)
      total = 0; 
    // std::cout<<"demand: "<<demand<<" measurement: "<<measurement<<" total: "<<total<<std::endl;
    
    return total;   
}

void PID::setMax(double new_max){
    if (new_max > 0)
        max_output = new_max;
    else
      std::cout<<"Max output must be positive";
}

PID::~PID(){
  delete robot_instance;
}