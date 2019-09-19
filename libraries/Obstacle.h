#include <cmath>
#include <iostream>

class Obstacle{
  private:
    double x_centre, z_centre;
    //Circle
    double radius;
    //Rectangle or Cylinder
    double length,breadth,heading;
    std::string objectType;
    bool isCircle;
  public:
    //Circle
    Obstacle(double x, double z, double r);
    //Rectangle
    Obstacle(double x, double z, double l, double b, double angle);
    bool checkIfInside(double obs_X,double obs_Z, int robot_index);
};