#include "pfsm_exploration.h"
#include "draw.h"
#include "auxiliary.h"
#include <algorithm> // std::find

pfsm_exploration::pfsm_exploration(): t(8)
{
  string p = param->policy();
  if (!strcmp(p.c_str(), "")) {
    for (size_t i = 0; i < 256; i++) {
      policy[i] = {0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125};
    }
  } else {
    policy = read_matrix(p);
  }
  timelim = 5. * param->simulation_updatefreq();
  moving_timer = rg.uniform_int(0, timelim);
  vmean = 0.5; // Adjustment velocity
  moving = false;
  selected_action = 3;
  st = 0;
}

void pfsm_exploration::action_motion(const int &selected_action, float r, float t, float &v_x, float &v_y)
{
  // float m = 1.;
  float ang[8] = {0., M_PI_4, M_PI_2, 3 * M_PI_4, M_PI, 5.*M_PI_4, 3 * M_PI_2, 7.*M_PI_4};
  // vector<float> ang = {-1.0, -0.7, -0.3, -0.1, 0.1, 0.3, 0.7, 1.0};
  // cout << selected_action << " " << ang[selected_action] << endl;
  polar2cart(vmean, t + ang[selected_action], v_x, v_y);
}

void pfsm_exploration::state_action_lookup(const int ID, uint state_index)
{
  vector<float> p = policy[state_index];
  // fmat<float>::print(1, 8, p, "p");
  if (!moving) {
    selected_action = rg.discrete_int(p);
  }
}

void pfsm_exploration::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0.0;
  v_y = 0.0;
  get_lattice_motion_all(ID, v_x, v_y);

  vector<bool> state;
  vector<int> temp;
  t.assess_situation(ID, state, temp);
  if (st != bool2int(state) || !moving) { // on state change
    st = bool2int(state);
#ifdef ESTIMATOR
    uint a = 0;
    if (moving) {a = selected_action + 1;}
    pr.update(ID, st, a);
#endif
    state_action_lookup(ID, st);
    float r, t;
    cart2polar(s[ID]->state[2], s[ID]->state[3], r, t);
    action_motion(selected_action, r, t, vx_ref, vy_ref);
  }
  increase_counter_to_value(moving_timer, timelim, 1);

  moving = true;
  v_x += vx_ref;
  v_y += vy_ref;

  wall_avoidance(ID, v_x, v_y);
}
