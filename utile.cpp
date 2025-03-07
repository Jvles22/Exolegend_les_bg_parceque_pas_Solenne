#include "utile.hpp"
#include <math.h>

float AngleDiffRad(float from, float to){
  //Return the difference between two angles in radians
  return atan2(sin(to - from), cos(to - from));
}

float modulo2pi(float a){
    //Return the modulo of a by 2*PI in the range [-PI, PI]
    if (a > M_PI) return (a - 2*M_PI);
    if (a < -M_PI) return (a + 2*M_PI);
    return a;
}

bool isInMaze(float x, float y, uint8_t max_maze_size, Gladiator* gladiator){
    //Return true if the point (x, y) is in the maze and false otherwise
    //max_maze_size is the maximum coef of the maze /!\ not the size of the maze (integer between 6 and 13)
    float max = (max_maze_size+1)*gladiator->maze->getSquareSize();
    float min = (start_max_size - max_maze_size)*gladiator->maze->getSquareSize();

    if(x > max || x < min || y > max || y < min) return false;
    return true;
}

float computeDistance(float x1, float y1, float x2, float y2){
    return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}



