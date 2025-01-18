#include "API.hpp"
#include <iostream>
#include <thread>
#include <chrono>


bool API_wallFront(Epuck& epuck) {
    return epuck.iswallFront();
}

bool API_wallRight(Epuck& epuck) {
    return epuck.iswallRight();
}

bool API_wallLeft(Epuck& epuck) {
    return epuck.iswallLeft();
}

void API_moveForward(Epuck& epuck, double* sensorValues) {
    epuck.moveForward(1, sensorValues);
}

void API_turnRight(Epuck& epuck) {
    epuck.turnRight();
}

void API_turnLeft(Epuck& epuck) {
    epuck.turnLeft();
}

void go(Epuck& epuck, double* sensorValues) {
    int counter = 1;
    while (true) {
        Config::Action nextMove = solver(epuck);
        std::cout << counter << " Next Move: " << nextMove << std::endl;

        switch (nextMove) {
            case Config::Action::FORWARD:
                API_moveForward(epuck, sensorValues);
                //epuck.turnToHeading(epuck.heading); // Ensure heading correction occurs after motion completes
                break;

            case Config::Action::RIGHT:
                API_turnRight(epuck);
                //epuck.turnToHeading(epuck.heading); 
                break;

            case Config::Action::LEFT:
                API_turnLeft(epuck);
                //epuck.turnToHeading(epuck.heading); 
                break;

            case Config::Action::IDLE:
                break;

            default:
                break;
        }

        // Wait for robot stabilization before checking the next move
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        if (nextMove == Config::Action::IDLE) {
            break;
        }

        counter++;
    }
}
