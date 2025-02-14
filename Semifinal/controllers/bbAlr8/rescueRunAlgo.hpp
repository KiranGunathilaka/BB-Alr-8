#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <limits>    // For std::numeric_limits
#include <algorithm> // For std::find, std::reverse, std::next_permutation
struct Point
{
    int x, y;
    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point &other) const
    {
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
    }
};

struct PathInfo
{
    std::vector<Point> path;
    int distance;
};

class RescueRunAlgo
{
public:
    void findOptimalRoute();
    RescueRunAlgo();
    ~RescueRunAlgo();
    void setMaze(const std::vector<std::vector<int>> &maze);
    void setDefaults();
    void setRedNodes(const std::vector<Point> &redNodes);
    void setOrangeNodes(const std::vector<Point> &orangeNodes);
    void setYellowNodes(const std::vector<Point> &yellowNodes);
    void setStartPoint(const Point &startPoint);
    void setSurvivors(const std::vector<Point> &survivors);

    // Add getter for the calculated path
    const std::vector<Point> &getOptimalPath() const { return optimalPath; }
    bool hasPathCalculated() const { return !optimalPath.empty(); }

    // Add movement commands structure
    enum class Command
    {
        MOVE_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_180,
        WAIT
    };

    struct Movement
    {
        Command command;
        Point nextPosition;
    };

    Movement getNextMovement(const Point &currentPos, const Point &nextPos, int currentHeading);

    // Maze data and constants
    const int MAZE_S = 20; // MAZE_SIZE
    const int NORTH = 8;   // 1000
    const int SOUTH = 4;   // 0100
    const int EAST = 2;    // 0010
    const int WEST = 1;    // 0001
    // Maze data as 2D array
    // North = 8, South = 4, East = 2, West = 1

    std::vector<std::vector<int>> maze = {
        {5, 4, 12, 12, 6, 5, 6, 5, 12, 12, 14, 13, 4, 12, 6, 5, 4, 12, 4, 6},
        {11, 3, 7, 5, 10, 3, 11, 9, 6, 5, 12, 6, 9, 6, 9, 2, 9, 6, 3, 11},
        {5, 8, 2, 1, 12, 10, 5, 4, 2, 9, 6, 3, 5, 2, 7, 9, 6, 3, 9, 6},
        {11, 5, 10, 9, 12, 6, 11, 1, 8, 12, 10, 1, 10, 11, 1, 4, 10, 11, 5, 10},
        {5, 10, 5, 12, 12, 8, 6, 9, 12, 12, 6, 1, 14, 5, 10, 3, 5, 4, 10, 7},
        {3, 5, 10, 5, 12, 6, 1, 4, 4, 4, 10, 9, 12, 8, 6, 9, 10, 1, 4, 10},
        {3, 1, 14, 3, 7, 9, 0, 2, 1, 8, 12, 12, 14, 5, 2, 7, 7, 1, 10, 7},
        {3, 1, 12, 2, 9, 4, 8, 2, 9, 6, 5, 6, 5, 0, 2, 3, 1, 10, 13, 2},
        {1, 2, 13, 10, 5, 10, 5, 8, 6, 3, 3, 1, 10, 11, 1, 2, 1, 12, 12, 2},
        {1, 8, 6, 5, 10, 13, 0, 14, 1, 2, 3, 11, 5, 12, 8, 8, 8, 6, 5, 2},
        {9, 6, 3, 9, 12, 6, 11, 5, 0, 2, 1, 6, 9, 12, 12, 12, 6, 3, 11, 3},
        {7, 3, 3, 5, 6, 9, 12, 10, 3, 9, 10, 1, 12, 4, 4, 6, 9, 10, 5, 10},
        {3, 3, 3, 3, 3, 5, 6, 13, 10, 13, 6, 3, 5, 2, 11, 3, 5, 12, 8, 6},
        {9, 10, 3, 3, 9, 10, 9, 4, 12, 12, 10, 1, 8, 10, 5, 0, 2, 5, 6, 3},
        {5, 12, 10, 9, 4, 4, 14, 9, 12, 6, 5, 10, 5, 4, 8, 2, 1, 10, 9, 10},
        {1, 6, 5, 4, 8, 2, 7, 5, 6, 3, 3, 5, 10, 1, 6, 1, 8, 6, 13, 6},
        {3, 6, 10, 5, 12, 12, 2, 3, 3, 11, 3, 9, 6, 1, 0, 10, 5, 8, 12, 10},
        {3, 1, 12, 10, 7, 5, 8, 10, 9, 12, 10, 13, 8, 2, 1, 6, 1, 14, 5, 14},
        {1, 8, 12, 12, 10, 9, 6, 5, 6, 5, 4, 6, 5, 8, 10, 3, 9, 6, 9, 6},
        {9, 12, 12, 12, 12, 12, 8, 8, 8, 10, 9, 8, 10, 13, 12, 8, 12, 8, 12, 10}};
    // std::vector<Point> redNodes = {{3, 7}, {15, 9}, {15, 15}}; // {y, x}
    // std::vector<Point> orangeNodes = {{2, 6}, {3, 6}, {4, 6}, {2, 7}, {4, 7}, {2, 8}, {3, 8}, {4, 8}, {14, 8}, {15, 8}, {16, 8}, {14, 9}, {16, 9}, {14, 10}, {15, 10}, {16, 10}, {14, 14}, {15, 14}, {16, 14}, {14, 15}, {16, 15}, {14, 16}, {15, 16}, {16, 16}};
    // std::vector<Point> yellowNodes = {};
    // std::vector<Point> survivors = {{19, 6}, {18, 17}, {0, 11}};


    // const int MAZE_S = 20; // MAZE_SIZE
    // const int NORTH = 8;   
    // const int SOUTH = 2;   
    // const int EAST = 4;    
    // const int WEST = 1;  

    // std::vector<std::vector<int>> maze = {}; // maze[y][x]
    std::vector<Point> redNodes = {}; // {y, x}
    std::vector<Point> orangeNodes = {};
    std::vector<Point> yellowNodes = {};
    std::vector<Point> survivors = {};
    
    Point startPoint = {10, 0};

private:
    std::vector<Point> optimalPath;



    std::vector<std::vector<int>> convertNESWtoNSEW(const std::vector<std::vector<int>> &neswMatrix);
    PathInfo findShortestPath(Point start, Point end);
    bool canMove(int x, int y, int direction);
    std::vector<std::vector<int>> calculateAllPairDistances();
    PathInfo solveTSP(const std::vector<std::vector<int>> &distances);
    void log(const std::string &text);
};