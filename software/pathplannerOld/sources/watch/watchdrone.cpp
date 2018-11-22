
#include "headers/watch/watchdrone.h"

WatchDrone::WatchDrone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp)
{
    map = aMap;

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

    currentEllipse.majorR = GeoFunctions::calcMeterDistanceBetweensCoords(nextWaypoint.position, currentPosition.position) + 500;
    currentEllipse.minorR = currentEllipse.majorR / 10;

    if(currentEllipse.minorR > 500)
        currentEllipse.minorR = 500;
    else if(currentEllipse.minorR < 100)
        currentEllipse.minorR = 100;
    currentEllipse.minorR = 500;

    currentEllipse.a = pow(currentEllipse.majorR / 2., 2);
    currentEllipse.b = pow(currentEllipse.minorR / 2., 2);

    currentEllipse.centre.first = nextWaypoint.position.first + (position.first - nextWaypoint.position.first) / 2;
    currentEllipse.centre.second = nextWaypoint.position.second + (position.second - nextWaypoint.position.second) / 2;

    std::vector<std::shared_ptr<Node>> oldNodesInArea = nodesInArea;

    nodesInArea.clear();

    std::vector<std::pair<size_t,size_t>> nodesToKeep;

    for(auto &it : *map.get())
        for(auto &it2 : it) {
            if(isInsideEllipse(it2->getWorldCoordinate())) {
                double distanceToPoint = GeoFunctions::calcMeterDistanceBetweensCoords(it2->getWorldCoordinate(), currentPosition.position);
                int ETA = distanceToPoint / currentPosition.velocity;
                ETA += currentPosition.epochETA;

                it2->addToColor(0,0,255);

                it2->addDynamicPenalty(droneID, 10000, ETA, ETA);

                nodesInArea.push_back(it2);

                nodesToKeep.push_back(it2->getNodeIndex());
            }
        }

    for(auto node : oldNodesInArea) {
        bool found = false;
        for(auto &keepers : nodesToKeep) {
            if(node->getNodeIndex() == keepers) {
                found = true;
                break;
            }
        }
        if(!found)
            node->removeDynamicPenalty(droneID);
        node->addToColor(0,0,-255);
    }
}

void WatchDrone::setNextWaypoint(std::pair<double, double> position, double altitude, double headingAngle, double velocity, int ETA)
{
    nextWaypoint.position = position;
    nextWaypoint.altitude = altitude;
    nextWaypoint.headingAngle = headingAngle;
    nextWaypoint.velocity = velocity;
    nextWaypoint.epochETA = ETA;
}

bool WatchDrone::checkLineIntersect(std::pair<double, double> p1, std::pair<double, double> p2)
{
    // NOT OPTIMAL
    double distance = GeoFunctions::calcMeterDistanceBetweensCoords(p1, p2);
    int points = int(distance / 50);
    std::pair<double, double> checkPoint = p1;
    double incrementX = (p2.first - p1.first) / double(points);
    double incrementY = (p2.second - p1.second) / double(points);
    for(int i = 0; i < points; i++) {
        checkPoint.first += incrementX;
        checkPoint.second += incrementY;
        if(isInsideEllipse(checkPoint))
            return true;
    }

    return false;
}

bool WatchDrone::isInsideEllipse(std::pair<double, double> point)
{
    double angle = GeoFunctions::calcAngle(currentPosition.position, nextWaypoint.position) / 180 * PI;

    std::pair<double, double> centre = currentEllipse.centre;

    std::pair<double, double> pointUTM;
    std::pair<double, double> centreUTM;

    std::string UTMZone1;
    RobotLocalization::NavsatConversions::LLtoUTM(point.first, point.second, pointUTM.first, pointUTM.second, UTMZone1);

    std::string UTMZone2;
    RobotLocalization::NavsatConversions::LLtoUTM(centre.first, centre.second, centreUTM.first, centreUTM.second, UTMZone2);
\
    double a = currentEllipse.a;
    double b = currentEllipse.b;
    double first = pow(((pointUTM.first - centreUTM.first) * cos(angle) + (pointUTM.second - centreUTM.second) * sin(angle)), 2) / a;
    double second = pow(((pointUTM.first - centreUTM.first) * sin(angle) - (pointUTM.second - centreUTM.second) * cos(angle)), 2) / b;

    if(first + second <= 1)
        return true;

    return false;
}
