#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <utility>
#include <vector>
#include <iostream>
#include <math.h>
#include <memory>
#include <unistd.h>

#include "headers/nodecollection.h"

class MapGenerator
{
public:
    MapGenerator();
    void generateTestMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections);


private:
    void calculateNeighbors(std::size_t row, std::size_t col, std::vector<std::shared_ptr<Node>> &neighborNodes, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map);
    void divideIntoCollections(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections);
    void getSlice(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, NodeCollection &collection);

private:
    // Note to self: DO NOT MAKE SUPER LARGER MAPS.
    const std::size_t mapSizeRow = 800;
    const std::size_t mapSizeCol = 1600;
    std::size_t gridRows = 40;
    std::size_t gridCols = 80;
};

#endif // MAPGENERATOR_H
