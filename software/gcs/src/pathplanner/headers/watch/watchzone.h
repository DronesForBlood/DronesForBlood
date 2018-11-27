#ifndef WATCHZONE_H
#define WATCHZONE_H

#include <vector>
#include <iostream>
#include <ctime>
#include <thread>
#include <chrono>

#include "headers/node.h"
#include "headers/global/geofunctions.h"
#include "headers/visualizer.h"

class WatchZone
{
public:
    WatchZone();

    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double,double>> polygonCoords, std::shared_ptr<Visualizer> aVisualizer);
    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double,double>> polygonCoords, std::shared_ptr<Visualizer> aVisualizer, time_t epochValidFrom, time_t epochValidTo);

    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double,double> circleMidpointCoord, double radius, std::shared_ptr<Visualizer> aVisualizer);
    WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double,double> circleMidpointCoord, double radius, std::shared_ptr<Visualizer> aVisualizer, time_t epochValidFrom, time_t epochValidTo);

    ~WatchZone();

    std::vector<std::shared_ptr<Node>> getNodesInArea();
    bool checkLineIntersect(std::pair<double,double> p1, std::pair<double,double> p2);

private:
    void findNodesInAreaPolygon();
    void findNodesInAreaCircle();
    bool linesIntersects(std::pair<double,double> start1, std::pair<double,double> end1, std::pair<double,double> start2, std::pair<double,double> end2);
    void handleDynamicVisualization();

private:
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::shared_ptr<Visualizer> visualizer;
    std::vector<std::shared_ptr<Node>> nodesInArea;
    std::shared_ptr<std::thread> dynamicThread;

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
    bool stopThread = false;
    bool threadStopped = true;

};

#endif // WATCHZONE_H
