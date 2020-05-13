#ifndef AGENT_THREAD_H
#define AGENT_THREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include <chrono>

#include "settings.h"
#include "randomgenerator.h"
#include "environment.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include AGENT_INCLUDE // from makefile

/**
 * Update the agent simulation
 * @param ID The ID of the agent/robot
 */
void run_agent_simulation_step(const int &ID)
{
  while (program_running) {
    // Update the position of the agent in the simulation
    // auto start = chrono::steady_clock::now();
    int t_wait = (int)1e6 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));

    mtx_env.lock();
    mtx.lock_shared();
    vector<float> s_0 = s.at(ID)->state;
    vector<float> s_n = s.at(ID)->state_update(s_0); // State update
    mtx.unlock_shared();
    mtx_env.unlock();

    /****** Wall physics engine ********/
    // Check if hitting a wall
    vector<float> test = s_n;
    float r_temp, ang_temp, vx_temp, vy_temp;
    cart2polar(s_n[2], s_n[3], r_temp, ang_temp); // direction of velocity
    polar2cart(r_temp, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
    test[0] += vx_temp;
    test[1] += vy_temp;
    if (!environment.sensor(ID, s_0, test, ang_temp)) { // No wall --> Update the dynamics
      mtx.lock();
      s.at(ID)->state = s_n;
    } else { // Wall! --> Kill the dynamics
      mtx.lock();
      s.at(ID)->state[2] = 0.0; // v_x
      s.at(ID)->state[3] = 0.0; // v_y
      s.at(ID)->state[4] = 0.0; // a_x
      s.at(ID)->state[5] = 0.0; // a_y
      s.at(ID)->controller->moving = false; // Not moving
    }
    if (ID == 0) {
      simtime_seconds += param->simulation_realtimefactor() * t_wait / 1e6;
    }
    mtx.unlock();
    /**********************************/
    // auto end = chrono::steady_clock::now();
    // auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();

    // Wait according to defined frequency (subtract execution time)
    this_thread::sleep_for(chrono::microseconds(t_wait));
  }
}

/**
 * Generates new agent + simulation thread at given position x0 y0
 *
 * @param ID The ID of the agent/robot
 * @param x Initial position of the agent in x
 * @param y Initial position of the agent in y
 */
void create_new_agent(const int &ID, const vector<float> &states)
{
  mtx.lock();
  // Initiate a new agent
  s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
#ifdef ESTIMATOR
  pr.extend(); // Extend estimator so that the new agent can also contribute
#endif
  mtx.unlock();

  // Info message
  stringstream ss;
  ss << "Robot " << ID << " initiated";
  terminalinfo::info_msg(ss.str());

  thread agent(run_agent_simulation_step, ID); // Initiate the thread that controls the agent
  agent.detach(); // Detach thread so that it runs independently
}
#endif /*AGENT_THREAD_H*/
