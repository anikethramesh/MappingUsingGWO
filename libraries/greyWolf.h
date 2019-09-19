#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>

#include <string>
#include <vector>
#include <cstdlib>

#include <Eigen/Dense>

#include "tracking.h"
#pragma once

class GreyWolf{
  private:
  //Receiver and Emitter handles
    webots::Emitter *emitter;
    webots::Receiver *receiver;
  //Swarm Setting
    int numberOfWolves;
    //boundary Variables
    int x_upper_boundary, x_lower_boundary, z_upper_boundary, z_lower_boundary;
    int num_iter;
  //Individual Settings
    double k_exploration;
    double k_multiplication;
    int social_learning_number;
    bool printDebuggingInfo;
  //Swarm Information
    robot_tracking::Position my_position, prev_position, target_position, starting_position;
    // bool inMotion;    
    struct Event{
      int event_id;
      int index;
      robot_tracking::Position robot_position;
      std::string event_message;
    }message;
  public:    
    std::vector<Event> init_information, event_message;
    std::vector<std::pair<Event,double> > fitnessRanker;
    double calculate_fitnessOfWolf(int current_iter, robot_tracking::Position wolfStartingPosition, robot_tracking::Position wolfCurrentPosition);
    int robot_index;
    double wolf_fitness;
    int timeStep;
    GreyWolf(webots::Emitter *emitter_handle, webots::Receiver *receiver_handle, int timestep, robot_tracking::Position init_pos);
    void set_swarmSettings(int index, int num_wolves, int num_iterations);
    void set_boundaries(int x_upper,int x_lower,int z_upper,int z_lower);
    void set_individualSettings(double explore_factor, double multiplication_factor, int social_number);
    void emit_currentPosition(int current_iter);
    void update_position(robot_tracking::Position position);
    void get_allEvents(int current_iter);
    void calculate_FitnessRanker(int current_iter);
    void print_AllEvents();
    // robot_tracking::Position calculate_nextPosition();
    void calculate_Constants(int current_iter);
    void set_targetPosition(robot_tracking::Position target);
    double calculate_EucDist(robot_tracking::Position P1,robot_tracking::Position P2);
    void init_StatusUpdate(robot_tracking::Position starting_position);
    void clear_initInformation();
    void clear_eventMessage();
    robot_tracking::Position calculateNextPosition(int current_iter);
    robot_tracking::Position get_previousPosition();
    bool check_insideBoundary(robot_tracking::Position nextPosition);   
    robot_tracking::Position generateRandomPosition();
  protected:
  //Constants
    // float a[2],r1[2],r2[2];
    Eigen::RowVector2d a,r1,r2;
    double generateRandomNumber(double lowerLimit, double upperLimit);
    robot_tracking::Position snapToBoundary(robot_tracking::Position currentPosition, robot_tracking::Position nextPosition);   
    // bool compareEvents(Event e1, Event e2);
    // bool comparePairs(std::vector<std::pair<Event,double> > p1,std::vector<std::pair<Event,double> > p2);
    
    
};