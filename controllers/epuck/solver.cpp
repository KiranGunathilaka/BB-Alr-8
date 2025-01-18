#include "solver.hpp"
#include <iostream>


//Position Epuck.position;


Config::Action NextAction(Epuck& epuck){
    int least_distance = 300;   // just some large number, none of the distances will be over 300
    Config::Action optimal_move = Config::Action::IDLE;

    //Config::Heading heading = epuck.heading;

    // bool iswallFront = API_wallFront(epuck);
    // bool iswallRight = API_wallRight(epuck);
    // bool iswallLeft = API_wallLeft(epuck);

    // std::cout << "Front Wall: " << iswallFront << std::endl;
    // std::cout << "Right Wall: " << iswallRight << std::endl;
    // std::cout << "Left Wall: " << iswallLeft << std::endl;

    


    if(epuck.heading == Config::Heading::NORTH){
        if(API_wallFront(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1);
            optimal_move = Config::Action::FORWARD;
        }
        if(API_wallRight(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped);
            optimal_move = Config::Action::RIGHT;
        }
        if(API_wallLeft(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped);
            optimal_move = Config::Action::LEFT;
        }
    }
    else if(epuck.heading == Config::Heading::EAST){
        if(API_wallFront(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped);
            optimal_move = Config::Action::FORWARD;
        }
        if(API_wallRight(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1);
            optimal_move = Config::Action::RIGHT;
        }
        if(API_wallLeft(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1);
            optimal_move = Config::Action::LEFT;
        }
    }
    else if(epuck.heading == Config::Heading::SOUTH){
        if(API_wallFront(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1);
            optimal_move = Config::Action::FORWARD;
        }
        if(API_wallRight(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped);
            optimal_move = Config::Action::RIGHT;
        }
        if(API_wallLeft(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped + 1, epuck.position.y_mapped);
            optimal_move = Config::Action::LEFT;
        }
    }
    else if(epuck.heading == Config::Heading::WEST){
        if(API_wallFront(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped - 1, epuck.position.y_mapped);
            optimal_move = Config::Action::FORWARD;
        }
        if(API_wallRight(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped + 1);
            optimal_move = Config::Action::RIGHT;
        }
        if(API_wallLeft(epuck) == 0 && epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1) < least_distance){
            least_distance = epuck.floodfill.getCellCost(epuck.position.x_mapped, epuck.position.y_mapped - 1);
            optimal_move = Config::Action::LEFT;
        }
    }
    // handles dead ends (when there's no walls in front, to the right or to the left)
    if (least_distance == 300) {
        optimal_move = Config::Action::RIGHT;
    }
    return optimal_move;
}

void UpdatePosition(Epuck& epuck, Config::Action NextAction){
    if(NextAction != Config::Action::FORWARD){
        return;
    }

    switch (epuck.heading)
    {
    case Config::Heading::NORTH:
        epuck.position.y_mapped += 1;
        break;
    case Config::Heading::EAST:
        epuck.position.x_mapped += 1;
        break;
    case Config::Heading::SOUTH:
        epuck.position.y_mapped -= 1;
        break;
    case Config::Heading::WEST:
        epuck.position.x_mapped -= 1;
        break;
    default:
        break;
    }
}

void updateHeading(Epuck& epuck, Config::Action NextAction){
    if(NextAction == Config::Action::FORWARD || NextAction == Config::Action::IDLE){
        return;
    }

    else if(NextAction == Config::Action::RIGHT){
        switch (epuck.heading)
        {
        case Config::Heading::NORTH:
            epuck.heading = Config::Heading::EAST;
            break;
        case Config::Heading::EAST:
            epuck.heading = Config::Heading::SOUTH;
            break;
        case Config::Heading::SOUTH:
            epuck.heading = Config::Heading::WEST;
            break;
        case Config::Heading::WEST:
            epuck.heading = Config::Heading::NORTH;
            break;
        default:
            break;
        }
    }

    else if(NextAction == Config::Action::LEFT){
        switch (epuck.heading)
        {
        case Config::Heading::NORTH:
            epuck.heading = Config::Heading::WEST;
            break;
        case Config::Heading::EAST:
            epuck.heading = Config::Heading::NORTH;
            break;
        case Config::Heading::SOUTH:
            epuck.heading = Config::Heading::EAST;
            break;
        case Config::Heading::WEST:
            epuck.heading = Config::Heading::SOUTH;
            break;
        default:
            break;
        }
    }
}
int GoToColorIdX = 0;
Config::Action solver(Epuck& epuck){
    

    int X = epuck.position.x_mapped;
    int Y = epuck.position.y_mapped;

    // Print world and mapped coordinates
    std::cout << "Current grid coordinates: x=" << X
              << ", y=" << Y << std::endl;

    if(!epuck.reachedColor && epuck.floodfill.getCellCost(X,Y) == 0){
        epuck.reachedColor = true;
    }
    else if(epuck.reachedColor && epuck.floodfill.getCellCost(X,Y) == 0){
        epuck.reachedColor = false;
         GoToColorIdX++;
         std::cout << "Red" << std::endl;
    }

    if(GoToColorIdX == 5){
        return Config::Action::IDLE;
    }

    epuck.floodfill.floodMaze(X , Y , Config::cellOrder[GoToColorIdX].first, Config::cellOrder[GoToColorIdX].second);
    //epuck.floodfill.printCosts(); 

    

    Config::Action nextAction = NextAction(epuck);

    updateHeading(epuck, nextAction);
    UpdatePosition(epuck, nextAction);

    return nextAction;
}