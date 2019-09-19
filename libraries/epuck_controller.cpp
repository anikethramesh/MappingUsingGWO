 //Webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>

//Standard Headers
#include <limits>
#include <iostream>
#include <stdio.h>
#include <cmath>

//My headers
#include "tracking.h"
#include "PID.h"
#include "epuck_controller.h"

//Macros
#define  _USE_MATH_DEFINES


//Definitions

void Epuck_Controller::wait(int seconds){
  for(int i=0;i<=(seconds*32);i++){
    robot_handler->step(timeStep);
  }
}

double Epuck_Controller::target_euclideanDistance(){
  return pow(pow(robot_tracker.target_position.x - robot_tracker.current_position.x,2)+pow(robot_tracker.target_position.z - robot_tracker.current_position.z,2),0.5);
}

double Epuck_Controller::return_atan2Value(){
  return atan2(robot_tracker.target_position.x - robot_tracker.current_position.x,robot_tracker.target_position.z - robot_tracker.current_position.z);
}

void Epuck_Controller::setup_HeadingControl(float P, float D, float I, float tolerance_units){
  heading_Control.setGains(P,I,D);
  heading_Control.setTolerance(tolerance_units);
  heading_Control.setMax(6.27);
  heading_Control.setYawCorrection(true);
}

void Epuck_Controller::setup_DistanceControl(float P, float D, float I, float tolerance_units){
  distance_Control.setGains(P,I,D);
  distance_Control.setTolerance(tolerance_units);
  distance_Control.setMax(6.27);
}

void Epuck_Controller::setup_DistanceSensors(){
  char psNames[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  for (int i = 0; i < 8; i++){
    ps[i] = robot_handler->getDistanceSensor(psNames[i]);
    ps[i]->enable(timeStep);
    robot_handler->step(timeStep);
    // std::cout<<"Type of Distance Sensor "<<i<<": "<<ps[i]->getType()<<"\n";
  }
}

//Constructor
Epuck_Controller::Epuck_Controller(webots::Robot *robot_handle, webots::Motor *leftMotor,webots::Motor *rightMotor,webots::GPS *gpsHandle, webots::InertialUnit *imuHandle)
:robot_tracker(gpsHandle,imuHandle,(int)robot_handle->getBasicTimeStep()), heading_Control(robot_handle),distance_Control(robot_handle) {
  timeStep = (int)robot_handle->getBasicTimeStep();
  double inf = std::numeric_limits<double>::infinity();
  GPS_Handle = gpsHandle;
  IMU_Handle = imuHandle;
  robot_handler = robot_handle;
  left_Motor = leftMotor;
  right_Motor = rightMotor;
  left_Motor->setPosition(inf);
  right_Motor->setPosition(inf);
  left_Motor->setVelocity(0);
  right_Motor->setVelocity(0);
  GPS_Handle->enable(timeStep);
  IMU_Handle->enable(timeStep);
  robot_handler->step(timeStep);
  setup_DistanceSensors();
  debugging = false;
  robot_handle->step(timeStep);
  robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);
}

void Epuck_Controller::print_currentPosition(){
  std::cout<<"Current position X: "<<robot_tracker.current_position.x<<" Z: "<<robot_tracker.current_position.z<<" heading: "<<(robot_tracker.current_position.heading*180/M_PI)<<"\n";
}

robot_tracking::Position Epuck_Controller::get_currentPosition(){
  return robot_tracker.current_position;
}

double Epuck_Controller::get_currentHeading(){
  return robot_tracker.current_position.heading;
}

void Epuck_Controller::moveMotors(double leftVelocity, double rightVelocity){
    //Constrain left velocity. 6.27 is the max velocity for a wheel
    if(leftVelocity>=6.27)
      left_Motor->setVelocity(6.27);
    else if(leftVelocity<=-6.27)
      left_Motor->setVelocity(-6.27);
    else
      left_Motor->setVelocity(leftVelocity);

    //Constrain right velocity. 6.27 is the max velocity for a wheel
    if(rightVelocity>=6.27)
      right_Motor->setVelocity(6.27);      
    else if(rightVelocity<=-6.27)
      right_Motor->setVelocity(-6.27);
    else
      right_Motor->setVelocity(rightVelocity);
      
    //Update position and make a time step
    robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);
    // Debugging info
    if(debugging)
      print_currentPosition();
    //Time Step 
    if(robot_handler->step(timeStep)!=-1);
}

//Brake the motors
void Epuck_Controller::stopMotors(){
  if(robot_handler->step(timeStep)!= -1){
    left_Motor->setVelocity(0);
    right_Motor->setVelocity(0);    
  }
  if(debugging)
    std::cout<<"Motors Stopped\n";
}

bool Epuck_Controller::isAbvThresh_PS(int psNumber, int threshold){
 //Value should take a value below 8
  int val = ps[psNumber]->getValue();
  return (val>=threshold?true:false);
}

void Epuck_Controller::setTargetPosition(robot_tracking::Position P){
  // robot_tracking::Position P;
  // P.x = x;
  // P.z = z;
  // P.heading = heading;
  robot_tracker.set_targetPosition(P);
}

void Epuck_Controller::headTo(double heading){
  double target_heading = heading;
  //Convert 0-2pi to -pi to pi.
  double velocity = 6.27;
  while(velocity!=0){
    robot_handler->step(timeStep);
    robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);  
    velocity = heading_Control.update(target_heading,robot_tracker.current_position.heading);
    if(debugging)
      std::cout<<"The heading velocity is:"<<velocity<<" \n";
    moveMotors(-velocity,velocity);
    // print_currentPosition();
  }
    stopMotors();
    heading_Control.reset();
}

void Epuck_Controller::moveTo_Target(){
  double goto_heading = return_atan2Value();
  double euc_dist;
  double distance_velocity=0;
  double heading_velocity=0;
  // std::cout<<"Heading is"<<goto_heading<<"\n";
  headTo(goto_heading);
  stopMotors();
  while(distance_velocity!=0){
      if (robot_handler->step(timeStep)!=-1){
      // std::cout<<"Euc Dist: "<<euc_dist<<"\t Heading velocity: "<<heading_velocity<<"Distance Velocity "<<distance_velocity<<" \n";
      robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);
      euc_dist = target_euclideanDistance();
      goto_heading = return_atan2Value();
      distance_velocity = -distance_Control.update(0,euc_dist);
      heading_velocity = heading_Control.update(goto_heading,robot_tracker.current_position.heading);
      moveMotors(distance_velocity-heading_velocity,distance_velocity+heading_velocity);
      if(debugging)
        print_currentPosition();        
    }
  }
  heading_Control.reset();
  distance_Control.reset();  
  stopMotors();
  // std::cout<<"\nmovement stopped\n";
  // goto_heading = robot_tracker.target_position.heading;
  // headTo(goto_heading);
  // std::cout<<"Stop Called \n";
}

void Epuck_Controller::getValues_DistanceSensors(bool printValues){
  // double psValues[8];
  // std::cout<<"Distance Sensor Values"<<std::endl;
  printValues = false;
  for (int i = 0; i < 8 ; i++){
    psValues[i] = ps[i]->getValue();
    if(printValues)
      std::cout<<i<<"th Sensors value"<<psValues[i]<<"\n";
  }
  robot_handler->step(timeStep);
}

void Epuck_Controller::moveForward_BugAlgorithm(int robot_index){
  // int checker = 3;
  double goto_heading = return_atan2Value();
  double euc_dist, initHeading, currentHeading;
  double distance_velocity, heading_velocity;
  headTo(goto_heading);
  bool bugMode = false;
  int emptyThresh, ObsThresh;
  while(robot_handler->step(timeStep)!=1 && (distance_velocity+heading_velocity)!=0){
    getValues_DistanceSensors(false);
    if(psValues[0]>300 || psValues[7]>300 || psValues[6]>300 || psValues[1]>300){
      bugMode = true;
      robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);
      ObsThresh = std::max(std::max(psValues[0],psValues[7]), std::max(psValues[6],psValues[1]));
      initHeading = get_currentHeading();
      while(psValues[2]<ObsThresh){
        if(std::abs(currentHeading - initHeading)>(M_PI/2)){ 
          break;
          bugMode = false;
        }
        moveMotors(-0.5,0.5);
        currentHeading = get_currentHeading();
        getValues_DistanceSensors(true);
      }
      stopMotors();
      getValues_DistanceSensors(true);
      emptyThresh  = (psValues[6] + psValues[5] + psValues[4])/3;
      ObsThresh = psValues[2];
      if(bugMode){
        while(isAbvThresh_PS(2,ObsThresh) || isAbvThresh_PS(1,emptyThresh)){
          euc_dist = target_euclideanDistance();
          distance_velocity = -distance_Control.update(0,euc_dist);
          // int sign = distance_velocity/std::abs(distance_velocity);
          moveMotors(distance_velocity,distance_velocity);
        }
      } 
    }
    else{
    // if(robot_index==checker){    
      // std::cout<<"_________Robot Number___________:"<<robot_index<<"\n";
      // std::cout<<"Inside else\n";
    // }
      robot_tracker.update_currentPosition(GPS_Handle,IMU_Handle);
      euc_dist = target_euclideanDistance();
      distance_velocity = -distance_Control.update(0,euc_dist);
      if(distance_velocity!=0){
        goto_heading = return_atan2Value();      
        heading_velocity = heading_Control.update(goto_heading,robot_tracker.current_position.heading);
      }
      else{
      // if(robot_index==checker){    
        // std::cout<<"_________Robot Number___________:"<<robot_index<<"\n";
        // std::cout<<"heading velocity is zero\n"; 
      // }     
        heading_velocity = 0;
      }
      // std::cout<<"Velocities d,h: "<<distance_velocity<<","<<heading_velocity<<"\n";
      moveMotors(distance_velocity-heading_velocity,distance_velocity+heading_velocity);
      if(debugging)
        print_currentPosition();
    }    
  }
  // if(robot_index==checker){
  // std::cout<<"_________Robot Number___________:"<<robot_index<<"\n";
  // std::cout<<"Reached end\n";} 
  heading_Control.reset();
  distance_Control.reset();  
  stopMotors();
  goto_heading = robot_tracker.target_position.heading;
  headTo(goto_heading); 
  // if(robot_index==checker){
  // std::cout<<"_________Robot Number___________:"<<robot_index<<"\n";
  // std::cout<<"Movement Stopped\n";}
  // std::cout<<"\nmovement stopped\n";  
}

Epuck_Controller::~Epuck_Controller(){
  delete robot_handler;
  delete left_Motor;
  delete right_Motor; 
  delete GPS_Handle;
  delete IMU_Handle;
}
