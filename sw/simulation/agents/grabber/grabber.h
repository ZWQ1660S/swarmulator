#ifndef GRABBER_H
#define GRABBER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"
#define COMMAND_LOCAL 1  // use COMMAND_LOCAL for local commands

using namespace std;

class grabber: public Agent
{
public:
  grabber(int i, vector<float> state, float tstep);
  vector<float> state_update(vector<float> state);
  void animation();
};

#endif /*GRABBER_H*/
