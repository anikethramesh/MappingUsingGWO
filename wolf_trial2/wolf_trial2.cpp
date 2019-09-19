//Webots headers
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Motor.hpp>

//My headers
#include "../../libraries/PID.h"
#include "../../libraries/tracking.h"
#include "../../libraries/epuck_controller.h"
#include "../../libraries/greyWolf.h"
#include "../../libraries/Obstacle.h"

//Standard Headers
#include<iostream>
#include <vector>
#include<fstream>
#include<limits>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <time.h>
#define _USE_MATH_DEFINES

using namespace webots;

bool inObstacleRegion(std::vector<Obstacle> ObstacleArray, robot_tracking::Position givenPosition, int robot_index){
  bool flag = false;
  for(std::vector<Obstacle>::iterator it = ObstacleArray.begin() ; it != ObstacleArray.end(); ++it){
    if(it->checkIfInside(givenPosition.x,givenPosition.z,robot_index))
      flag = true;
  }
  return flag;
}

int main(int argc, char **argv) {
  //Get robot and configuration parameters from custom data
  Robot *robot = new Robot();
  std::vector<Obstacle> ObstacleList;
  // Obstacle horizontalCylinder(-2.76,-2.26,2,1.4,M_PI);
  ObstacleList.emplace_back(1.85,-0.38,0.6);
  ObstacleList.emplace_back(-2.76,3.42,0.9);
  ObstacleList.emplace_back(-1.27,-1.25,0.6,0.6,0);
  ObstacleList.emplace_back(1.36,3.31,0.75,0.3,0); 
  ObstacleList.emplace_back(1.59,-2.82,1,1,0);   
  std::string robotName = robot->getName();
  std::cout<<robotName+".csv"<<"\n";
  std::ofstream myfile(robotName+".csv");
  int timeStep = (int)robot->getBasicTimeStep();
  std::string customData = robot->getCustomData();
  std::vector<std::string> data;
  boost::split(data, customData, boost::is_any_of(","));
  srand(time(0)*(std::stoi(data[0])+1)); 
  //Get Motors
  Motor *left_motor = robot->getMotor("left wheel motor");
  Motor *right_motor = robot->getMotor("right wheel motor");
  //Get GPS and IMU
  GPS *gps = robot->getGPS("gps");
  InertialUnit *imu = robot->getInertialUnit("inertial unit");
  //Setup controller
  Epuck_Controller epuck(robot,left_motor,right_motor,gps,imu);
  epuck.setup_HeadingControl(1,0,0,0.01);
  epuck.setup_DistanceControl(10,0,0,0.05);
  //Emitter and Receiver setup
  Emitter *emitter = robot->getEmitter("emitter");
  Receiver *receiver = robot->getReceiver("receiver");
  //Setup the GreyWolf Object
  robot->step(timeStep);
  GreyWolf wolf(emitter, receiver, timeStep,epuck.get_currentPosition());
  robot->step(timeStep);
  wolf.set_boundaries(5,-5,5,-5);
  wolf.set_swarmSettings(std::stoi(data[0]),std::stoi(data[1]),std::stoi(data[2]));
  wolf.set_individualSettings(std::stof(data[3]),std::stof(data[4]),std::stoi(data[5]));
  //Setup operation
  robot->step(timeStep);
  wolf.update_position(epuck.get_currentPosition());
  robot->step(timeStep);
  //Share the starting positions with all the robots.
  std::cout<<"Sending out init status update\n";
  wolf.init_StatusUpdate(epuck.get_currentPosition());
  epuck.wait(3);
  robot->step(timeStep);  
  wolf.get_allEvents(0);
  // std::cout<<"Finished initialization\n";
  //Setup target
  robot_tracking::Position targetPosition = {2.32,2.32, M_PI/2};
  wolf.set_targetPosition(targetPosition);
  int num_iter = std::stoi(data[2]);
  for(int i=0;i<num_iter;i++){
    // if(std::stoi(data[0])==3){    
      // std::cout<<"_________Iteration Number___________:"<<i<<"\n";
    // }    
    myfile<<i<<",";
    robot_tracking::Position currentPos = epuck.get_currentPosition();
    myfile<<currentPos.x<<","<<currentPos.z<<","<<robot->getTime()<<"\n";
    myfile.flush();
    wolf.update_position(currentPos);
    robot->step(timeStep);
    wolf.clear_eventMessage();
    robot->step(timeStep);
    std::cout<<"_________ROBOT INDEX___________:"<<std::stoi(data[0])<<"\n";    
    std::cout<<"\nIteration Number"<<i<<"\n";
    wolf.emit_currentPosition(i);
    robot->step(timeStep);
    wolf.get_allEvents(i);
    robot->step(timeStep);
    robot_tracking::Position nextPosition = wolf.calculateNextPosition(i);
    robot->step(timeStep);
    // nextPosition = wolf.calculateNextPosition(i);
    // bool checkInside = powerSupply.checkIfInside(nextPosition.x,nextPosition.z,std::stoi(data[0]));
    bool checkInside = inObstacleRegion(ObstacleList,nextPosition,std::stoi(data[0]));
    while(checkInside){
      std::cout<<"_________ROBOT INDEX___________:"<<std::stoi(data[0])<<"\tCalled obstacle check"<<"\n";
      nextPosition = wolf.generateRandomPosition();
      checkInside = inObstacleRegion(ObstacleList,nextPosition,std::stoi(data[0]));
    }
    robot->step(timeStep);
    epuck.setTargetPosition(nextPosition); 
    robot->step(timeStep);    
    epuck.moveForward_BugAlgorithm(std::stoi(data[0]));
    robot->step(timeStep);
    currentPos = epuck.get_currentPosition();
    if(!wolf.check_insideBoundary(currentPos)){
      std::cout<<"_________ROBOT INDEX___________:"<<std::stoi(data[0])<<"\n";
      std::cout<<"Called out of boundary correction. Generating random position\n";
      //If the robot is outside the boundary, the robot returns to the previous position and calculates the next iteration again.
      nextPosition = wolf.generateRandomPosition();
      checkInside = inObstacleRegion(ObstacleList,nextPosition,std::stoi(data[0]));
      while(checkInside){
        std::cout<<"_________ROBOT INDEX___________:"<<std::stoi(data[0])<<"\tCalled obstacle check"<<"\n";
        nextPosition = wolf.generateRandomPosition();
        checkInside = inObstacleRegion(ObstacleList,nextPosition,std::stoi(data[0]));
      }  
      epuck.setTargetPosition(nextPosition);
      robot->step(timeStep);
      epuck.moveForward_BugAlgorithm(std::stoi(data[0]));   
      robot->step(timeStep); 
      i-=1;
    }
    // currentPos = epuck.get_currentPosition();
    // while(powerSupply.checkIfInside(nextPosition.x,nextPosition.z)){
      // std::cout<<"WHILEn";
      // std::cout<<"_________ROBOT INDEX___________:"<<std::stoi(data[0])<<"\tCalled obstacle check"<<"\n";
      // std::cout<<"Called obstacle check"<<"\n";
      // nextPosition = wolf.generateRandomPosition();
      // epuck.setTargetPosition(nextPosition);
    // }

  }
  myfile.close();
  delete robot;
  return 0;
}

