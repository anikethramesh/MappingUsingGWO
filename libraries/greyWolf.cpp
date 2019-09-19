#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include <string>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include<limits>

#include <Eigen/Dense>

#include "tracking.h"
#include "greyWolf.h"

#define _USE_MATH_DEFINES


double calculate_euclideanDistance(double x1, double z1, double x2, double z2){
  return pow(pow(x2 - x1,2)+pow(z2 - z1,2),0.5);
}


GreyWolf::GreyWolf(webots::Emitter *emitter_handle, webots::Receiver *receiver_handle, int timestep, robot_tracking::Position init_pos){
  emitter = emitter_handle;
  receiver = receiver_handle;
  timeStep = timestep;
  receiver->enable(timeStep);
  printDebuggingInfo = false;
  starting_position = init_pos;
  init_information.clear();
  event_message.clear();
  a <<1,1;
  r1<<1,1;
  r2<< 1,1;
  if(printDebuggingInfo){
    std::cout<<"_________ROBOT INDEX___________:"<<robot_index<<"\n";
    std::cout<<starting_position.x<<starting_position.z<<"\n";    
  }
}

void GreyWolf::set_swarmSettings(int index, int num_wolves, int num_iterations){
  robot_index = index;
  numberOfWolves = num_wolves;
  num_iter = num_iterations;
}

void GreyWolf::set_boundaries(int x_upper,int x_lower,int z_upper,int z_lower){
  x_upper_boundary = x_upper;
  x_lower_boundary = x_lower;
  z_upper_boundary = z_upper;
  z_lower_boundary = z_lower;
}

void GreyWolf::set_individualSettings(double explore_factor, double multiplication_factor, int social_number){
  k_exploration = explore_factor;
  k_multiplication = multiplication_factor;
  social_learning_number = social_number;
}

void GreyWolf::emit_currentPosition(int current_iter){
  message.event_id = current_iter;
  message.index = robot_index;
  message.robot_position = my_position;
  message.event_message.assign("Position Update");
  emitter->send(&message, sizeof(Event));
  event_message.push_back(message);
}

void GreyWolf::init_StatusUpdate(robot_tracking::Position starting_position){
  message.event_id = 0;
  message.index = robot_index;
  message.robot_position = my_position;
  message.event_message.assign("Initialisation");
  emitter->send(&message, sizeof(Event));
  init_information.push_back(message);
}

void GreyWolf::update_position(robot_tracking::Position position){
  prev_position = my_position;
  my_position = position;
}

void GreyWolf::set_targetPosition(robot_tracking::Position target){
  target_position = target;
}

void GreyWolf::get_allEvents(int current_iter){
    event_message.clear();
    Event *received_event;    
    while(receiver->getQueueLength() > 0){
      received_event = (Event *) receiver->getData();
      if(received_event->event_message.compare("Initialisation") == 0)
        init_information.push_back(*received_event);
      else if(received_event->event_message.compare("Position Update") == 0){
        // if(received_event->event_id == current_iter)
          event_message.push_back(*received_event);      
      }
      receiver->nextPacket();
    }
    if(printDebuggingInfo){
      std::cout<<"Printing Events\n";
      print_AllEvents();
    }
}



void GreyWolf::print_AllEvents(){
  std::cout<<"_________ROBOT INDEX___________:"<<robot_index<<"\n";
  for(std::vector<Event>::const_iterator i = event_message.begin(); i != event_message.end(); ++i){
    std::cout<<i->event_id<<"\t"<<i->index<<"\t"<<i->event_message<<"\n";
    std::cout<<"Position"<<"\t"<<i->robot_position.x<<"\t"<<i->robot_position.z<<"\t"<<i->robot_position.heading<<"\n";
  }
}

void GreyWolf::clear_initInformation(){
  init_information.clear();
}

void GreyWolf::clear_eventMessage(){
  event_message.clear();
}

double GreyWolf::calculate_EucDist(robot_tracking::Position P1,robot_tracking::Position P2){
  return std::pow(std::pow(P1.x - P2.x,2) + std::pow(P1.z - P2.z,2),0.5);
}

double GreyWolf::generateRandomNumber(double lowerLimit, double upperLimit){
  double rand_num = ((double)rand())/((double)RAND_MAX +1);
  rand_num = lowerLimit + ((upperLimit - lowerLimit)*rand_num);
  return rand_num;
}

void GreyWolf::calculate_Constants(int current_iter){
  double buff = (double)k_exploration*((double)1 - ((double)current_iter/(double)num_iter));
  // std::cout<<"Buff"<<buff;
  a << buff,buff;
  // std::cout<<a<<"\n";
  r1 << generateRandomNumber(0,2),generateRandomNumber(0,2);
  r2 << generateRandomNumber(0,2),generateRandomNumber(0,2);
  if(printDebuggingInfo){
    std::cout<<"current iter"<<current_iter<<" and num iter"<<num_iter<<"\n";
    std::cout<<"_________ROBOT INDEX___________:"<<robot_index<<"\n";
    std::cout<<"R1: "<<r1[0]<<"\t"<<r1[1]<<"\n";
    std::cout<<"R2: "<<r2[0]<<"\t"<<r2[1]<<"\n";
    std::cout<<"A: "<<a[0]<<"\t"<<a[1]<<"\n";        
  }
}
// bool GreyWolf::compareEvents(Event e1, Event e2){
  // return (e1.index<e2.index);
// }

// bool comparePairs(const std::pair<GreyWolf::Event,double> &a, 
              // const std::pair<GreyWolf::Event,double> &b) 
// { 
    // return (a.second < b.second); 
// } 

double GreyWolf::calculate_fitnessOfWolf(int current_iter, robot_tracking::Position wolfStartingPosition, robot_tracking::Position wolfCurrentPosition){
    
    double k_intraSwarmDistance = 1 - (current_iter/num_iter);
    // double k_intraSwarmDistance = 0;
    double intraSwarmDistance = 0, 	wolf_communicationThreshold = 1;
    for(std::vector<Event>::const_iterator i = event_message.begin(); i!= event_message.end(); ++i){
      // if(i->event_id == current_iter && i->index != robot_index){
      if(i->index != robot_index){      
        intraSwarmDistance += wolf_communicationThreshold/calculate_EucDist(wolfCurrentPosition,i->robot_position);
      }
    }
  //Calculate individual coverage
    double coverageIndividual = calculate_EucDist(wolfStartingPosition, wolfCurrentPosition);
    double k_coverageIndividual = 1 - (current_iter/num_iter);
  //Calculate distance from the goal
    double k_distanceFromGoal = 1 + (current_iter/num_iter);
    double distanceFromGoal = calculate_EucDist(target_position,wolfCurrentPosition); 
    
    double fitness = (k_coverageIndividual*coverageIndividual) + (k_intraSwarmDistance*intraSwarmDistance) - (k_distanceFromGoal*distanceFromGoal);
    return fitness;
}

void GreyWolf::calculate_FitnessRanker(int current_iter){
  // fitnessRanker.clear();
  std::sort(init_information.begin(), init_information.end(),[](Event e1, Event e2){ 
    return (e1.index < e2.index);
  });// Fix this!
  for(std::vector<Event>::const_iterator i = event_message.begin(); i != event_message.end(); ++i){
    //Get the corresponding starting position for that event
    robot_tracking::Position startingPosition = init_information[i->index].robot_position;
    //Made change to the first parameter of the fitness function. 
    //Earlier, the current iter was passed as first argument, now the event id is passed.
    double fitness = calculate_fitnessOfWolf(i->event_id, startingPosition, i->robot_position);
    fitnessRanker.push_back(std::make_pair(*i,fitness));
  }
  //Sort the fitnessRanker
  std::sort(fitnessRanker.begin(), fitnessRanker.end(), [](const std::pair<GreyWolf::Event,double> &a, const std::pair<GreyWolf::Event,double> &b){ 
    return (a.second < b.second);
  });
}

robot_tracking::Position GreyWolf::calculateNextPosition(int current_iter){
  bool isOmega = true;
  Eigen::Vector2d target(target_position.x, target_position.z);
  Eigen::Vector2d currentPosition(my_position.x, my_position.z);
  Eigen::Vector2d nextPosition(0,0);
  robot_tracking::Position next_pos;
  fitnessRanker.clear();
  calculate_FitnessRanker(current_iter);
  calculate_Constants(current_iter);
  Eigen::Vector2d A,C,D,buff;
  
  
  
  for (int i=0;i<social_learning_number;i++){
    if(fitnessRanker[i].first.index == robot_index)
      isOmega = false;
  }
  //Calculate A
  A << 2*a[0]*r1[0] , 2*a[1]*r1[1];
  A -= a;
  //Calculate C
  C = 2*r2;
  
  if(isOmega){
    //Calculate D
    D << C[0]*target[0] , C[1]*target[1];
    D -= currentPosition; 
    nextPosition << A[0]*D[0] ,A[1]*D[1];
    nextPosition  = target - nextPosition;
  } 
  else{
    for (int j=0;j<social_learning_number;j++){
      D << C[0]*fitnessRanker[j].first.robot_position.x, C[1]*fitnessRanker[j].first.robot_position.z;
      D -= currentPosition;
      buff <<fitnessRanker[j].first.robot_position.x - A[0]*D[0] , fitnessRanker[j].first.robot_position.z - A[1]*D[1];
      nextPosition+= buff;
    }
    nextPosition *= 1/social_learning_number;
  }
  
  nextPosition = currentPosition + k_multiplication*(nextPosition - currentPosition);
  next_pos.x = nextPosition[0];
  next_pos.z = nextPosition[1];
  next_pos.heading = M_PI/3;

  // std::cout<<"_________POSITION of ROBOT INDEX___________:"<<robot_index<<"\n";
  // std::cout<<"before boundary snap = ("<<next_pos.x<<","<<next_pos.z<<","<<next_pos.heading*(180/M_PI)<<")\n";
  
  if(!check_insideBoundary(next_pos)){
    // std::cout<<"Boundary snap called\n";
    next_pos = snapToBoundary(my_position, next_pos);
  }
  
  // std::cout<<"_________POSITION of ROBOT INDEX___________:"<<robot_index<<"\n";
  // std::cout<<"after boundary snap = ("<<next_pos.x<<","<<next_pos.z<<","<<next_pos.heading*(180/M_PI)<<")\n";  
  
  
  return next_pos;
}

robot_tracking::Position GreyWolf::snapToBoundary(robot_tracking::Position currentPosition, robot_tracking::Position nextPosition){
  // Set boundary points
  double x1 = currentPosition.x , z1 = currentPosition.z;
  double x2 = nextPosition.x , z2 = nextPosition.z;
  double x3,z3,x4,z4;
  double ta,tb;
  double x_new, z_new;
    
  // std::cout<<"_________Before snapping Function of ROBOT INDEX___________:"<<robot_index<<"\n";
  // std::cout<<"X1,Z1= ("<<x1<<","<<z1<<")\n";
  // std::cout<<"X2,Z2= ("<<x2<<","<<z2<<")\n";    

  if(x1>=x_upper_boundary || x1<=x_lower_boundary)
    x1 = round(x1);
  if(z1>=z_upper_boundary || z1<=z_lower_boundary)
    z1 = round(z1);


  //Setup boundaries
  x3 = x_upper_boundary;
  z3 = z_upper_boundary;
  x4 = x_upper_boundary;
  z4 = z_lower_boundary;
  //Calculate constants
  ta = (((z3 - z4)*(x1 - x3)) + ((x4 - x3)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
  tb = (((z1 - z2)*(x1 - x3)) + ((x2 - x1)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));

  if(ta>=0 && ta<=1 && tb<=1 && tb>=0){
    x_new = x1 + ta*(x2 - x1);
    z_new = z1 + ta*(z2 - z1);
  }
  
  else{
    //Setup boundaries
    x3 = x_upper_boundary;
    z3 = z_upper_boundary;
    x4 = x_lower_boundary;
    z4 = z_upper_boundary;
    //Calculate constants
    ta = (((z3 - z4)*(x1 - x3)) + ((x4 - x3)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
    tb = (((z1 - z2)*(x1 - x3)) + ((x2 - x1)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
  
    if(ta>=0 && ta<=1 && tb<=1 && tb>=0){
      x_new = x1 + ta*(x2 - x1);
      z_new = z1 + ta*(z2 - z1);
    }
    else{
      //Setup boundaries
      x3 = x_lower_boundary;
      z3 = z_lower_boundary;
      x4 = x_lower_boundary;
      z4 = z_upper_boundary;
      //Calculate constants
      ta = (((z3 - z4)*(x1 - x3)) + ((x4 - x3)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
      tb = (((z1 - z2)*(x1 - x3)) + ((x2 - x1)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
    
      if(ta>=0 && ta<=1 && tb<=1 && tb>=0){
        x_new = x1 + ta*(x2 - x1);
        z_new = z1 + ta*(z2 - z1);
      }
      else{
        //Setup boundaries
        x3 = x_lower_boundary;
        z3 = z_lower_boundary;
        x4 = x_upper_boundary;
        z4 = z_lower_boundary;
        //Calculate constants
        ta = (((z3 - z4)*(x1 - x3)) + ((x4 - x3)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
        tb = (((z1 - z2)*(x1 - x3)) + ((x2 - x1)*(z1 - z3)))/(((x4 - x3)*(z1 - z2)) - ((x1 - x2)*(z4 - z3)));
      
        if(ta>=0 && ta<=1 && tb<=1 && tb>=0){
          x_new = x1 + ta*(x2 - x1);
          z_new = z1 + ta*(z2 - z1);
        }
        else{
          x_new = nextPosition.x;
          z_new = nextPosition.z;
        }
      }  
    }
  }

  // std::cout<<"_________Before scaling of ROBOT INDEX___________:"<<robot_index<<"\n";
  // std::cout<<"X_new,Z_new= ("<<x_new<<","<<z_new<<")\n";

  //Scale next Position so that it always remains inside the boundary
  //Done to prevent the robot from always flocking to the edges  
  // std::cout<<"_________Difference in positions ROBOT INDEX___________:"<<robot_index<<"\n";
  // std::cout<<"X_diff,Z_diff= ("<<(x_new - currentPosition.x)<<","<<(z_new - currentPosition.z)<<")\n"; 
   
  robot_tracking::Position newPosition;
  newPosition.x = x_new;
  newPosition.z = z_new;
  newPosition.heading = nextPosition.heading;
  
  return newPosition; 
}

robot_tracking::Position GreyWolf::generateRandomPosition(){
  robot_tracking::Position newPos;
  newPos.x = generateRandomNumber(x_lower_boundary,x_upper_boundary);
  newPos.z = generateRandomNumber(z_lower_boundary,z_upper_boundary);
  newPos.heading = 0;
  return newPos; 
}

bool GreyWolf::check_insideBoundary(robot_tracking::Position nextPosition){
  bool x_insideBoundary = false, z_insideBoundary = false;
  if(nextPosition.x> x_lower_boundary && nextPosition.x< x_upper_boundary)
    x_insideBoundary = true;
  if(nextPosition.z> z_lower_boundary && nextPosition.z< z_upper_boundary)
    z_insideBoundary = true;
  
  return (x_insideBoundary && z_insideBoundary);
}

robot_tracking::Position GreyWolf::get_previousPosition(){
  return prev_position;
}