#ifndef UTILE
#define UTILE

#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <ctime>
#include <algorithm>
#include <cstdlib>
#include "local_maze.hpp"
#include "gladiator.h"

/**
 * @brief Return the difference between two angles in radians
 * @param from : the first angle
 * @param to : the second angle
 * @return the difference between the two angles
*/
float AngleDiffRad(float from, float to);

/**
 * @brief Return the modulo of a by 2*PI in the range [-PI, PI]
 * @param a : the angle
 * @return the modulo of a by 2*PI
*/
float modulo2pi(float a);

/**
 * @brief Return true if the point (x, y) is in the maze and false otherwise
 * @param x : the x coordinate of the point
 * @param y : the y coordinate of the point
 * @param max_maze_size : the maximum coef of the maze /!\ not the size of the maze (integer between 6 and 13)
 * @param gladiator : the gladiator object
 * @return true if the point (x, y) is in the maze and false otherwise
*/
bool isInMaze(float x, float y, uint8_t max_maze_size, Gladiator* gladiator);

/**
 * @brief Return the distance between two points
 * @param x1 : the x coordinate of the first point
 * @param y1 : the y coordinate of the first point
 * @param x2 : the x coordinate of the second point
 * @param y2 : the y coordinate of the second point
 * @return the distance between the two points
*/
float computeDistance(float x1, float y1, float x2, float y2);

#endif