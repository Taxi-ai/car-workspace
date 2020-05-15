#ifndef CAR_H
#define CAR_H

class Car
{
public:
  // all measurements in m && m/s
  double x{60};
  double y{0};
  float yaw{0}; // TODO make sure IMU return it as rad or deg and edit where we use yaw 
  double speed{0};
  float acc{0};
  // current car lane
  int lane{0};
  void updateCarState(float speed, float acc, float duration);
};
#endif