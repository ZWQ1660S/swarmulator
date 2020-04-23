#include "pagerank_estimator.h"
#include "main.h"
#include "fmat.h"
#include <fstream>
#include <iostream>
#include "fitness_functions.h"

#include <algorithm>
#include <iostream>
#include <vector>

void pagerank_estimator::init()
{
  s_k.assign(nagents, 0);
  s_kp1.assign(nagents, 0);
  fitness = 0;
  fitness_max = 0;
}

void pagerank_estimator::extend(void)
{
  if (s.size() > nagents) {
    s_k.resize(s.size(), 0);
    s_kp1.resize(s.size(), 0);
    s_k.assign(s.size(), 0);
    s_kp1.assign(s.size(), 0);
  }
}

void pagerank_estimator::update(const uint &ID, const int &st, const uint &a)
{
  s_kp1[ID] = st; // update state
  update_G(ID, a); // update G = f(H,E,A) estimate
  update_des(); // update des
  s_k[ID] = st; // update state
}

void pagerank_estimator::update_G(const uint &ID, const uint &action)
{
  // Update H or E depending on whether the change was your own
  if (action > 0) {
    H[s_kp1[ID] + s_k[ID] * n_states] += 1; // col + row * row_length
    A[s_kp1[ID] + s_k[ID] * n_states] = action;
  } else {
    E[s_kp1[ID] + s_k[ID] * n_states] += 1; // col + row * row_length
  }
  pr[s_kp1[ID]] += 1; // Update pr vector
}

void pagerank_estimator::update_des(void)
{
  fitness = evaluate_fitness();
  if (fitness >= fitness_max) {
    fitness_max = fitness;
    for (uint i = 0; i < n_states; i++) {
      des[i] = std::count(s_kp1.begin(), s_kp1.end(), i);
    }
  }
  fmat<uint>::print(1, n_states, des, "fitness");
}

void pagerank_estimator::print(void)
{
  cout << "********************" << endl;
  fmat<uint>::print(n_states, n_states, H, "H");
  fmat<uint>::print(n_states, n_states, E, "E");
  fmat<uint>::print(n_states, n_states, A, "A");
  fmat<uint>::print(1, n_states, pr, "pr");
  fmat<uint>::print(1, n_states, des, "des");
}

void pagerank_estimator::save(void)
{
  fmat<uint>::write_to_csv("logs/E_" + identifier + ".csv", E, n_states, n_states);
  fmat<uint>::write_to_csv("logs/H_" + identifier + ".csv", H, n_states, n_states);
  fmat<uint>::write_to_csv("logs/A_" + identifier + ".csv", A, n_states, n_states);
  fmat<uint>::write_to_csv("logs/des_" + identifier + ".csv", des, 1, n_states);
  fmat<uint>::write_to_csv("logs/pr_" + identifier + ".csv", pr, 1, n_states);
}