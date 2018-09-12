#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <utility>
#include <vector>
#include <iostream>
#include <math.h>
#include <memory>
#include <unistd.h>

#include "nodecollection.h"

class Pathfinder
{
public:
    Pathfinder();

private:
    void solve(std::size_t initRow, std::size_t initCol);
    void calculateNeighbors(std::size_t row, std::size_t col, std::vector<std::shared_ptr<Node>> &neighborNodes);
    void generateMap();
    void divideIntoCollections();
    void getSlice(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, NodeCollection &collection);

    void printMap();
    void printCostMap();
    void printPathMap(std::size_t row, std::size_t col);
    void makePathToDestination(std::size_t row, std::size_t col);

private:
    std::vector<std::vector<std::shared_ptr<Node>>> map;
    std::vector<NodeCollection> nodeCollections;
    std::vector<std::pair<std::size_t, std::size_t>> path;

    const std::size_t mapSize = 2000;
};

#endif // PATHFINDER_H
