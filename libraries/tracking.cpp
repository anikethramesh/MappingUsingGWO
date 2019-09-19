#include<iostream>
#include<cmath>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include "tracking.h"
#include <limits>
#include <vector>

#define _USE_MATH_DEFINES

using namespace webots;

robot_tracking::robot_tracking(GPS *gps_pointer, InertialUnit *imu_pointer,int timeStep){
  gps_pointer->enable(timeStep);
  imu_pointer->enable(timeStep);
  // std::cout<<"Robot Tracking enabled\n";
  update_currentPosition(gps_pointer,imu_pointer);
}

void robot_tracking::update_currentPosition(GPS *gps_pointer, InertialUnit *imu_pointer){
  const double *gps_val = gps_pointer->getValues();
  const double *imu_val = imu_pointer->getRollPitchYaw();   
  previous_position = current_position;
  current_position.x = *gps_val;
  current_position.z = *(gps_val +2);//This is actually the z coordinate, but will be treated as y for simplicity
  float val = *(imu_val+2);//Yaw from the IMU
  //Inverting the robot. The side with the position sensors ps0 and ps7 face ahead
  if(val>0)
    current_position.heading = val - M_PI;
  else if(val<0)
    current_position.heading = val + M_PI;
}
void robot_tracking::set_targetPosition(Position target){
  target_position = target;
}
void robot_tracking::set_tolerance(double tolerance_position, double tolerance_heading){
  heading_tolerance = tolerance_heading;// In radians 
  position_tolerance = tolerance_position;// In cartesian units
}
bool robot_tracking::if_reachedHeading(double target_heading){
  if (std::abs(current_position.heading - target_heading)<heading_tolerance)
    return true;
  else
    return false;
}

bool robot_tracking::if_reachedTargetHeading(){
  if (std::abs(current_position.heading - target_position.heading)<heading_tolerance)
    return true;
  else
    return false;
}

bool robot_tracking::if_reachedTargetPosition(){
  double distance = std::pow(current_position.x - target_position.x,2) + std::pow(current_position.z - target_position.z,2);
  if(std::pow(distance,0.5)<position_tolerance)
    return true;
  else
    return false;
}