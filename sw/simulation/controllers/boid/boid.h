#ifndef BOID_H
#define BOID_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

#define COMMAND_LOCAL 1

class boid: public Controller
{
public:
  boid(): Controller() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*BOID_H*/
