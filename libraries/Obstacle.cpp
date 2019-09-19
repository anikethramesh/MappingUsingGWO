#include <cmath>
#include <iostream>

#include "Obstacle.h"
Obstacle::Obstacle(double x, double z, double r){
  x_centre = x;
  z_centre = z;
  radius = r;
  isCircle = true;
}
//Rectangle
Obstacle::Obstacle(double x, double z, double l, double b, double angle){
  x_centre = x;
  z_centre = z;
  length = l;
  breadth = b;
  heading = angle;
  isCircle = false;
}

bool Obstacle::checkIfInside(double obs_X,double obs_Z, int robot_index){
  //Rectangle Verticles
  // bool isInside;
  if(!isCircle){
    double x1,x2,x3,x4;
    double z1,z2,z3,z4;
    bool buff1,buff2,buff3,buff4;    
    
    //X Coordinates
    x1 = x_centre - (length*std::cos(heading)) - (breadth*std::sin(heading));
    x2 = x_centre + (length*std::cos(heading)) - (breadth*std::sin(heading));
    x3 = x_centre + (length*std::cos(heading)) + (breadth*std::sin(heading));
    x4 = x_centre - (length*std::cos(heading)) + (breadth*std::sin(heading));
    //Y Coordinates
    z1 = z_centre - (length*std::sin(heading)) + (breadth*std::cos(heading));
    z2 = z_centre + (length*std::sin(heading)) + (breadth*std::cos(heading));
    z3 = z_centre + (length*std::sin(heading)) - (breadth*std::cos(heading));
    z4 = z_centre - (length*std::sin(heading)) - (breadth*std::cos(heading));
    
    //z1 - a, z4 - b, z3 - d
    //am*ab
    
    buff1 = (obs_X - x1)*(x4 - x1) + (obs_Z - z1)*(z4- z1) > 0;
    buff2 = (obs_X- x1)*(x4 - x1) + (obs_Z - z1)*(z4- z1) < ((x4 - x1)*(x4 - x1)) + ((z4 - z1)*(z4-z1));
    buff3 = (obs_X - x1)*(x3- x1) + (obs_Z- z1)*(z3 - z1) > 0;
    buff4 = (obs_X - x1)*(x3 - x1) + (obs_Z - z1)*(z3 - z1) < ((x3 - x1)*(x3 - x1)) + ((z3 - z1)*(z3-z1));

    std::cout<<"_________ROBOT INDEX___________:"<<robot_index<<"\n";
    std::cout<<"Given Point: ("<<obs_X<<","<<obs_Z<<")\n";
    std::cout<<"P1: ("<<x1<<","<<z1<<")\n";
    std::cout<<"P2: ("<<x2<<","<<z2<<")\n";
    std::cout<<"P3: ("<<x3<<","<<z3<<")\n";
    std::cout<<"P4: ("<<x4<<","<<z4<<")\n";            
    
    return (buff1&&buff2&&buff3&&buff4);
  }
  else{
    double buff;
    buff = std::sqrt(std::pow(obs_X-x_centre,2)+std::pow(obs_Z-z_centre,2));
    std::cout<<"_________ROBOT INDEX___________:"<<robot_index<<"\n";
    std::cout<<"Given Point: ("<<obs_X<<","<<obs_Z<<")\n";
    std::cout<<"Pipe Centre: ("<<x_centre<<","<<z_centre<<")\n";    
    std::cout<<"Radius is"<<radius<<"\n";
    std::cout<<"Distance is"<<buff<<"\n";
    return(buff<=radius);
  }
}