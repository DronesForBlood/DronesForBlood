#ifndef WATCHZONE_H
#define WATCHZONE_H

#include <vector>
#include <iostream>
#include <ctime>

#include "headers/node.h"
#include "headers/global/geofunctions.h"

class WatchZone
{
public:
    WatchZone();

    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double,double>> polygonCoords);
    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double,double>> polygonCoords, time_t epochValidFrom, time_t epochValidTo);

    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double,double> circleMidpointCoord, double radius);
    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double,double> circleMidpointCoord, double radius, time_t epochValidFrom, time_t epochValidTo);

    ~WatchZone();

    std::vector<std::shared_ptr<Node>> getNodesInArea();

private:
    void findNodesInAreaPolygon();
    void findNodesInAreaCircle();

private:
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::vector<std::shared_ptr<Node>> nodesInArea;

    int internalServerID;
    std::string name;

    bool usePolygon;
    bool dynamicZone;

    std::vector<std::pair<double,double>> polygonCoordinates;

    std::pair<double,double> circleMidpointCoordinate;
    double meterRadius;

    std::time_t validFrom;
    std::time_t validTo;

    double penalty = 1000;

};

#endif // WATCHZONE_H
