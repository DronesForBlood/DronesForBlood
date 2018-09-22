#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <utility>
#include <vector>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <memory>
#include <unistd.h>

#include "headers/nodecollection.h"

class MapGenerator
{
public:
    MapGenerator();
    void generateMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections, std::pair<double,double> startCoord, std::pair<double,double> endCoord, double distanceBetweenNodes, double width, double padLength);
    void generateMapTest(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections);


private:
    void divideIntoCollections(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections);

    void calculateNeighborsTest(std::size_t row, std::size_t col, std::vector<std::shared_ptr<Node>> &neighborNodes, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map);
    void divideIntoCollectionsTest(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections);
    void getSliceTest(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, NodeCollection &collection);

    double calcMeterDistanceBetweensCoords(std::pair<double,double> startCoord, std::pair<double,double> endCoord);
    std::pair<double,double> calcShiftedCoord(std::pair<double,double> coord, double dxMeters, double dyMeters);
    std::pair<double,double> calcNormalVector(std::pair<double,double> startCoord, std::pair<double,double> endCoord);

private:
    const double pi = 3.14159265359;
    const double radiusEarthMeters = 6371000.0;
    // Note to self: DO NOT MAKE SUPER LARGE MAPS.
    //*
    std::size_t nodesPrCollectionAxis = 25;

    const std::size_t mapSizeRow = 800;
    const std::size_t mapSizeCol = 1600;
    std::size_t gridRows = 800/25;
    std::size_t gridCols = 1600/25;
    /*/
    const std::size_t mapSizeRow = 400;
    const std::size_t mapSizeCol = 400;
    std::size_t gridRows = 20;
    std::size_t gridCols = 20;
    //*/
};

#endif // MAPGENERATOR_H
