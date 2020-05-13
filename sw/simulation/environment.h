#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// #include <stdio.h>
// #include <iostream>
#include <stdint.h>
#include <vector>
#include <mutex>
using namespace std;

class Environment
{
  vector<vector<float>> walls;
public:
  vector<vector<float>> food;
  vector<float> beacon;

  /**
  * @brief Construct a new Environment object
  *
  */
  Environment();

  /**
   * @brief Destroy the Environment object
   *
   */
  ~Environment() {};

  /**
   * @brief Define the initial obstacle list according to the list in conf/environments/.txt
   * You can indicate obstacle list in the conf/parameters.xml file, under <environment>
   * Make sure the file exists!
   */
  void define_walls(void);

  /**
   * @brief Define the initial obstacle list according to the list in conf/environments/.txt
   * You can indicate obstacle list in the conf/parameters.xml file, under <environment>
   * Make sure the file exists!
   */
  void define_food(uint64_t n);


  void define_beacon(float x, float y);

  /**
   * @brief Returns a point within the environment.
   *
   * @return float
   */
  vector<float> start(void);

  /**
   * @brief Returns the furthers point from (0,0) in the environment, used for initialization so that
   * the robots can be initialized in the same spot.
   *
   * @return float
   */
  float limits(void);

  /**
   * @brief Add a new wall to the list, going from (x0,y0) to (x1,y1).
   * This is used to interactively create walls in the animation, just right click with the mouse!
   *
   * @param x0 Initial x
   * @param y0 Initial y
   * @param x1 Final x
   * @param y1 Final y
   */
  void add_wall(float x0, float y0, float x1, float y1);

  /**
   * Senses whether the next state will go through a wall
   *
   * @param ID Robot to consider
   * @param s_n Next state
   * @param s Current state
   * @return true if the lines intersect, meaning that it will go through a wall, so that we can handle it.
   * @return false if the lines do not intersect, so that the robot will not go through a wall and can act normally.
   */
  bool sensor(const uint8_t ID, vector<float> s_n, vector<float> s, float &angle);

  void grab_food(uint64_t food_ID);

  /**
   * Function used to draw all the walls in the animation. It is called by the animation thread.
   *
   */
  void animate(void);
};

#endif /*ENVIRONMENT_H*/
