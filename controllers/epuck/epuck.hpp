// epuck.hpp
#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include "sensor_manager.hpp"
#include "floodfill.hpp"
#include "motors.hpp"
#include "config.hpp"
#include "API.hpp"
#include "solver.hpp"

struct Position {
    int x_mapped, y_mapped;   // Mapped grid coordinates
};

class Epuck : public webots::Supervisor {
public:
    Epuck();
    virtual ~Epuck();
    void run();
    Position recordOwnPosition();
    SensorManager sensorManager;
    Motors motors;
    Floodfill floodfill;

    bool reachedColor = false;

    Config::Heading heading;
    Position position;

    bool iswallFront();
    bool iswallRight();
    bool iswallLeft();

    void turnToHeading(Config::Heading targetHeading);


    double sensorValues[Config::NUM_SENSORS] = {0};

    void moveForward(int cells, double *sensorValues);
     void turnLeft();
    void turnRight();
    void turn180();

    
private:
   
    webots::PositionSensor *leftPosSensor;
    webots::PositionSensor *rightPosSensor;
    
    webots::LED *leds[Config::NUM_LEDS];
    
    webots::Node *selfNode;  // Added to store robot node reference

    void initDevices();
   
    //void moveForward(int cells, double *sensorValues);
    void faceNorth();
};

#endif 
