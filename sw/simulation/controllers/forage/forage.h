#ifndef FORAGE_H
#define FORAGE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

class forage: public Controller
{
  vector<float> motion_p; // Probability of motion
  uint moving_timer; // Timer measuring how long a robot has been moving
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
  bool holds_food;
public:
  forage();
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*FORAGE_H*/
