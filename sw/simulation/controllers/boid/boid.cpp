#include "boid.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"

void boid::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{

  v_x = 0;
  v_y = 0;
  v_x = 0.5;
  get_lattice_motion_all(ID, v_x, v_y, 5);

  vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
  float avg_psi = 0.;

  for (size_t i = 0; i < closest.size(); i++) {
    avg_psi += wrapToPi_f(s[closest[i]]->state[6]) / closest.size();
  }

  v_y = avg_psi - wrapToPi_f(s[ID]->state[6]);

  wall_avoidance_t(ID, v_x, v_y);
}
