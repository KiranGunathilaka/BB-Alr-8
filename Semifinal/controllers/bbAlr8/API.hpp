#ifndef API_H
#define API_H

#include <string>
#include "bbAlr8.hpp"
#include "sensorManager.hpp"
#include "motors.hpp"
#include "solver.hpp"

class BbAlr8;

void API_moveForward();
void API_turnLeft();
void API_turnRight();
void API_turn180();


bool API_wallFront();
bool API_wallRight();
bool API_wallLeft();

std::array<std::array<unsigned int, MAZE_SIZE>, MAZE_SIZE> exploreMaze();

int8_t API_getColour();
bool API_is_RedNode(int x, int y);
bool API_is_OrangeNode(int x, int y);
bool API_is_YellowNode(int x, int y);
bool API_is_SurvivorNode(int x, int y);
void API_add_OrangeNode(int x, int y);
void API_add_YellowNode(int x, int y);
void API_add_RedNode(int x, int y);

void API_detectAndAddSurvivor(int x , int y);

#endif // API_H