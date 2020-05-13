#include "forage.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "math.h"

forage::forage() : Controller()
{
  moving = false;
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);
  // motion_p = {P1, P2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // motion_p = {0.991355, 0.984845, 0.007304, 0.000783, 0.004238, 0.001033, 0.007088};
  string p = param->policy();
  if (!strcmp(p.c_str(), "")) {
    motion_p = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    // motion_p = {1.0, 0.0, 0.0, 0.0, 1.0, 0.14, 0.14};
    // motion_p = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  } else {
    motion_p = read_array(p);
  }
  timelim = 2.0 * param->simulation_updatefreq();
  moving_timer = rg.uniform_int(0, timelim);
  vmean = 0.5;
  holds_food = false;
}

void forage::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = min(r.size(), motion_p.size());
#ifdef ESTIMATOR
    int a;
    if (moving) {a = 1;}
    else {a = 0;}
    pr.update(ID, st, a); // pr update
#endif

    if (rg.bernoulli(1.0 - motion_p[st])) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      v_x_ref = vmean;
      v_y_ref = rg.gaussian_float(0.0, 0.5);
      moving = true;
    }
  }
  increase_counter_to_value(moving_timer, timelim, 1);

  uint8_t ID_food;
  bool t = o.sense_food(ID, ID_food);
  if (t && !holds_food) {
    environment.grab_food(ID_food);
    holds_food = true;
    terminalinfo::info_msg("grabbed food", ID);
  }

  if (holds_food) {
    float br, bt;
    o.beacon(ID, v_x_ref, v_y_ref);
    if (v_x_ref < rangesensor) {
      holds_food = false;
      terminalinfo::info_msg("released food", ID);
    }
    cout << v_y_ref << endl;
    v_x_ref = vmean;
  }

  wall_avoidance_t(ID, v_x_ref, v_y_ref);

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}
