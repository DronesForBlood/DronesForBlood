#ifndef WATCHDRONE_H
#define WATCHDRONE_H

#include <iostream>
#include <ctime>
#include <memory>

#include "headers/node.h"
#include "headers/global/geofunctions.h"
#include "headers/global/coordconverter.h"

struct WaypointData {
    std::pair<double,double> position;
    double altitude;
    double headingAngle;
    double velocity;
    int epochETA;
};

struct EllipseData {
    double majorR;
    double minorR;
    double a;
    double b;
    std::pair<double,double> centre;
};

class WatchDrone
{
public:
    WatchDrone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp);
    ~WatchDrone();

    std::string getID() { return droneID; }
    void updateCurrentPosition(std::pair<double,double> position, double altitude, double headingAngle, double velocity);
    void setNextWaypoint(std::pair<double,double> position, double altitude, double headingAngle, double velocity, int ETA);

    WaypointData getCurrentPosition() {return currentPosition;}
    WaypointData getNextWaypoint() {return nextWaypoint;}

    bool checkLineIntersect(std::pair<double,double> p1, std::pair<double,double> p2);

private:
    bool isInsideEllipse(std::pair<double,double> point);

private:
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::vector<std::shared_ptr<Node>> nodesInArea;

    std::string droneID;
    std::string name;
    int UAVOperationStatus;
    int epochTrackingEntry;
    int GPSTimestamp;

    EllipseData currentEllipse;
    WaypointData currentPosition;
    WaypointData nextWaypoint;

};

#endif // WATCHDRONE_H
