#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#pragma once
//The file with all the definitions has more comments.

class robot_tracking{
  public:
  struct Position{
    double x;
    // double y;
    double z;
    double heading; //In radians
  }current_position,previous_position,target_position;  
  bool printDebuggingInformation;
  robot_tracking(webots::GPS *gps_pointer, webots::InertialUnit *imu_pointer,int timeStep);
  void set_tolerance(double position, double heading);
  void update_currentPosition(webots::GPS *gps_pointer, webots::InertialUnit *imu_pointer);
  void set_targetPosition(Position target);
  // ~robot_tracking();
  private:
  double heading_tolerance, position_tolerance;
  bool if_reachedTargetHeading();
  bool if_reachedHeading(double target_heading);
  bool if_reachedTargetPosition();
};