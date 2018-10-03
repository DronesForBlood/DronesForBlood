
#include "headers/watch/watchzone.h"

WatchZone::WatchZone()
{

}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double, double> > polygonCoords)
{
    map = aMap;

    polygonCoordinates = polygonCoords;
    usePolygon = true;
    dynamicZone = false;

    findNodesInAreaPolygon();
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double, double> > polygonCoords, time_t epochValidFrom, time_t epochValidTo)
{
    map = aMap;

    polygonCoordinates = polygonCoords;
    usePolygon = true;
    dynamicZone = true;

    validFrom = epochValidFrom;
    validTo = epochValidTo;

    findNodesInAreaPolygon();
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double, double> circleMidpointCoord, double radius)
{
    map = aMap;

    circleMidpointCoordinate = circleMidpointCoord;
    meterRadius = radius;
    usePolygon = false;
    dynamicZone = false;

    findNodesInAreaCircle();
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double, double> circleMidpointCoord, double radius, time_t epochValidFrom, time_t epochValidTo)
{
    map = aMap;

    circleMidpointCoordinate = circleMidpointCoord;
    meterRadius = radius;
    usePolygon = true;
    dynamicZone = true;

    validFrom = epochValidFrom;
    validTo = epochValidTo;

    findNodesInAreaCircle();
}

WatchZone::~WatchZone()
{

}

std::vector<std::shared_ptr<Node> > WatchZone::getNodesInArea()
{
    return nodesInArea;
}

void WatchZone::findNodesInAreaPolygon()
{
    nodesInArea.clear();
    for(auto &it : *map.get())
        for(auto &it2 : it)
            if(GeoFunctions::pointIsInsidePolygon(polygonCoordinates, it2->getWorldCoordinate()))
                nodesInArea.push_back(it2);
}

void WatchZone::findNodesInAreaCircle()
{
    nodesInArea.clear();
    for(auto &it : *map.get())
        for(auto &it2 : it)
            if(GeoFunctions::calcMeterDistanceBetweensCoords(circleMidpointCoordinate, it2->getWorldCoordinate()) < meterRadius)
                nodesInArea.push_back(it2);
}
