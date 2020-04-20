#ifndef PAGERANK_ESTIMATOR_H
#define PAGERANK_ESTIMATOR_H
#include <stdio.h>
#include <eigen3/Eigen/Sparse>

using namespace std;

class pagerank_estimator
{
public:
  vector<uint> H, E, A, des, pr, s_k, s_kp1;
  uint n_states;
  float fitness;

  /**
   * @brief Construct a new pagerank estimator object.
   *
   */
  pagerank_estimator(uint s_size)
  {
    n_states = s_size;
    H.assign(pow(n_states, 2), 0);
    E.assign(pow(n_states, 2), 0);
    A.assign(pow(n_states, 2), 0);
    des.assign(n_states, 0);
    pr.assign(n_states, 0);
  };

  /**
   * @brief Destroy the pagerank estimator object
   *
   */
  ~pagerank_estimator() {};

  /**
   * @brief Initialize the estimator
   *
   * @param s_size Discrete size of the state-space
   */
  void init();

  /**
   * Extend the state vector by 1.
   * This is needed every time we add a new robot, which can now contribute to making a better estimate.
   *
   */
  void extend(void);

  /**
   * @brief Update the G matrix estimate and the desired state estimate
   *
   * @param ID ID of the agent making the update call
   * @param s State of the agent in state-action matrix
   * @param a Action that the agent will take in the state-action matrix (use a=0 for no action, and number all other actions starting from 1)
   */
  void update(const uint &ID, const int &s, const uint &a);

  /**
   * Update the G matrix estimates
   *
   * @param ID ID of the agent making the update call
   * @param action The number of the action
   */
  void update_G(const uint &ID, const uint &action);

  /**
   * Update the estimate of the desired states
   *
   */
  void update_des(void);

  /**
   * Print the matrices to the terminal. Can be useful for debugging.
   *
   */
  void print(void);

  /**
   * Save the matrices to CSV files.
   *
   */
  void save(void);
};

#endif /*PAGERANK_ESTIMATOR_H*/