#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "math.h"


namespace Config
{
    // Time configuration
    static const int TIME_STEP = 32; // milliseconds

    // Hardware configuration
    static const int NUM_SENSORS = 8;
    static const int NUM_LEDS = 10;

    // Sensors
    static const int STABILIZING_THRESHOLD = 50;
    static const int SENSOR_MAX = 4095;
    static const int SENSOR_MIN = 0;

    static const int MAX_DISTANCE = 35; // cm
    static const int MIN_DISTANCE = 0;

    static const double IDEAL_WALL_DISTANCE = 9; // Desired distance from wall in cm
    static const int MAX_WALL_DISTANCE = 25;
    static const double MAX_STEERING = 1.5; // Maximum steering adjustment
    static const double MAX_INTEGRAL = 1.0; // Anti-windup limit

    static const double Kp = 0.07;
    static const double Ki = 0.01;
    static const double Kd = 0.11;

    // Motors
    const int TIME_90_TURN = 1600;
    const int TIME_PER_CELL = 1850; 

    static constexpr double MAX_SPEED = 10.0;
    constexpr double BASE_SPEED = Config::MAX_SPEED * 0.7;
    constexpr double TURN_SPEED = Config::MAX_SPEED * 0.5; 

    // Floodfill
    const int MAZE_LENGTH = 10;
    const int MAZE_WIDTH = 10;
    
    const std::pair<int, int> cellOrder[4] = { {7, 6}, {5, 0}, {7, 3}, {8, 4} }; //{x, y}


    typedef enum Heading
    {
        NORTH,
        EAST,
        SOUTH,
        WEST
    } Heading;

    typedef enum Action
    {
        LEFT,
        FORWARD,
        RIGHT,
        IDLE
    } Action;

    // Threshold values for detecting walls
    const float L_WALL_THRESHOLD = 25;
    const float F_WALL_THRESHOLD = 18;
    const float R_WALL_THRESHOLD = 25;

    const float COS10 = 0.98480775301;

}
#endif