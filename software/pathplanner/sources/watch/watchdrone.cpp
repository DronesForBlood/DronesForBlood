
#include "headers/watch/watchdrone.h"

WatchDrone::WatchDrone(std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp)
{
    droneID = aDroneID;
    name = aName;
    UAVOperationStatus = operationStatus;
    epochTrackingEntry = trackingEntry;
    GPSTimestamp = aGPSTimestamp;
}

WatchDrone::~WatchDrone()
{

}

void WatchDrone::updateCurrentPosition(std::pair<double, double> position, double altitude, double headingAngle, double velocity)
{
    currentPosition.position = position;
    currentPosition.altitude = altitude;
    currentPosition.headingAngle = headingAngle;
    currentPosition.velocity = velocity;
    currentPosition.epochETA = int(std::time(nullptr));
}

void WatchDrone::setNextWaypoint(std::pair<double, double> position, double altitude, double headingAngle, double velocity, int ETA)
{
    nextWaypoint.position = position;
    nextWaypoint.altitude = altitude;
    nextWaypoint.headingAngle = headingAngle;
    nextWaypoint.velocity = velocity;
    nextWaypoint.epochETA = ETA;
}
