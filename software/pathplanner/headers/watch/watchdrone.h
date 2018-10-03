#ifndef WATCHDRONE_H
#define WATCHDRONE_H

#include <iostream>
#include <ctime>

struct WaypointData {
    std::pair<double,double> position;
    double altitude;
    double headingAngle;
    double velocity;
    int epochETA;
};

class WatchDrone
{
public:
    WatchDrone(std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp);
    ~WatchDrone();

    void updateCurrentPosition(std::pair<double,double> position, double altitude, double headingAngle, double velocity);
    void setNextWaypoint(std::pair<double,double> position, double altitude, double headingAngle, double velocity, int ETA);

    WaypointData getCurrentPosition() {return currentPosition;}
    WaypointData getNextWaypoint() {return nextWaypoint;}

private:


private:
    std::string droneID;
    std::string name;
    int UAVOperationStatus;
    int epochTrackingEntry;
    int GPSTimestamp;

    WaypointData currentPosition;
    WaypointData nextWaypoint;

};

#endif // WATCHDRONE_H
