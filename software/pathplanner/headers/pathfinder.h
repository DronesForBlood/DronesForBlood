
#include <utility>
#include <vector>
#include <iostream>
#include <math.h>

#include "node.h"

class Pathfinder
{
public:
    Pathfinder();

private:
    void solve(std::size_t initRow, std::size_t initCol);
    void calculateNeighbors(std::size_t row, std::size_t col, std::vector<Node*> &neighborNodes);
    void generateMap();
    void printMap();
    void printCostMap();
    void printPathToDestination(std::size_t row, std::size_t col);

private:
    std::vector<std::vector<Node>> map;

};
