#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include "tracking.h"
#include "PID.h"
#pragma once
class Epuck_Controller{
  private:   
    //Webots variables
    webots::GPS *GPS_Handle;
    webots::InertialUnit *IMU_Handle;
    webots::Motor *left_Motor;
    webots::Motor *right_Motor;
    webots::DistanceSensor *ps[8];
    webots::Robot *robot_handler;
    //Standard Variables
    int timeStep;
    double psValues[8];
    bool debugging;
    //Custom Variables
    robot_tracking robot_tracker;
    PID heading_Control;
    PID distance_Control;
    //Functions
    double target_euclideanDistance();
    double return_atan2Value();
    //double velocity_Constrain(double value);
    void setup_DistanceSensors();  
  public:
    void setup_HeadingControl(float P, float D, float I, float tolerance_units);
    void setup_DistanceControl(float P, float D, float I, float tolerance_units);
    //Constructor
    Epuck_Controller(webots::Robot *robot_handle,webots::Motor *leftMotor,webots::Motor *rightMotor,webots::GPS *gpsHandle, webots::InertialUnit *imuHandle);
    //Basic movement
    void moveMotors(double leftVelocity, double rightVelocity);
    void stopMotors();
    //Debugging
    void print_currentPosition();
    //Position sensor
    bool isAbvThresh_PS(int psNumber, int threshold);
    //Set the target position
    void setTargetPosition(robot_tracking::Position P);
    //High level movement functions
    void headTo(double heading);
    void moveTo_Target();//Do not use this. This is Shitty!
    //Position Sensor debugging
    void getValues_DistanceSensors(bool printValues);
    //Obstacle Avoidance
    void moveForward_BugAlgorithm(int robot_index); 
    robot_tracking::Position get_currentPosition();
    void wait(int seconds);
    double get_currentHeading();
    //Destructor
    ~Epuck_Controller();      
};