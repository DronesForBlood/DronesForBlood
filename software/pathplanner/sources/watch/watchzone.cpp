
#include "headers/watch/watchzone.h"

WatchZone::WatchZone()
{

}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double, double> > polygonCoords, std::shared_ptr<Visualizer> aVisualizer)
{
    map = aMap;
    visualizer = aVisualizer;

    polygonCoordinates = polygonCoords;
    usePolygon = true;
    dynamicZone = false;

    findNodesInAreaPolygon();

    for(auto it : nodesInArea)
        it->addToColor(255, 0, 0);
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::vector<std::pair<double, double> > polygonCoords, std::shared_ptr<Visualizer> aVisualizer, time_t epochValidFrom, time_t epochValidTo)
{
    map = aMap;
    visualizer = aVisualizer;

    polygonCoordinates = polygonCoords;
    usePolygon = true;
    dynamicZone = true;

    validFrom = epochValidFrom;
    validTo = epochValidTo;

    findNodesInAreaPolygon();


    for(auto it : nodesInArea) {
        it->addDynamicPenalty(10000, int(epochValidFrom), int(epochValidTo));
        it->addToColor(100, 0, 0);
    }

    /*
    for(auto &it : *map.get())
        for(auto &it2 : it) {
            it2->setStable(false);
            it2->unlockNodeReady();
        }
        */

    threadStopped = false;
    dynamicThread = std::shared_ptr<std::thread>(new std::thread(&WatchZone::handleDynamicVisualization,this));
    dynamicThread->detach();
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double, double> circleMidpointCoord, double radius, std::shared_ptr<Visualizer> aVisualizer)
{
    map = aMap;
    visualizer = aVisualizer;

    circleMidpointCoordinate = circleMidpointCoord;
    meterRadius = radius;
    usePolygon = false;
    dynamicZone = false;

    findNodesInAreaCircle();

    for(auto it : nodesInArea)
        it->addToColor(255, 0, 0);
}

WatchZone::WatchZone(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap, std::pair<double, double> circleMidpointCoord, double radius, std::shared_ptr<Visualizer> aVisualizer, time_t epochValidFrom, time_t epochValidTo)
{
    map = aMap;
    visualizer = aVisualizer;

    circleMidpointCoordinate = circleMidpointCoord;
    meterRadius = radius;
    usePolygon = false;
    dynamicZone = true;

    validFrom = epochValidFrom;
    validTo = epochValidTo;

    findNodesInAreaCircle();

    for(auto it : nodesInArea) {
        it->addDynamicPenalty(10000, validFrom, validTo);
        it->addToColor(100, 0, 0);
    }

    threadStopped = false;
    dynamicThread = std::shared_ptr<std::thread>(new std::thread(&WatchZone::handleDynamicVisualization,this));
    dynamicThread->detach();
}

WatchZone::~WatchZone()
{
    stopThread = true;
    while(!threadStopped)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::vector<std::shared_ptr<Node> > WatchZone::getNodesInArea()
{
    return nodesInArea;
}

bool WatchZone::checkLineIntersect(std::pair<double, double> p1, std::pair<double, double> p2)
{
    if(usePolygon) {
        if(linesIntersects(p1, p2, polygonCoordinates.front(), polygonCoordinates.back()))
            return true;

        for(size_t i = 0; i < polygonCoordinates.size() - 1; i++)
            if(linesIntersects(p1, p2, polygonCoordinates[i], polygonCoordinates[i+1]))
                return true;
    }
    else {
        // NOT OPTIMAL
        double distance = GeoFunctions::calcMeterDistanceBetweensCoords(p1, p2);
        int points = int(distance / 50);
        std::pair<double, double> checkPoint = p1;
        double incrementX = (p2.first - p1.first) / double(points);
        double incrementY = (p2.second - p1.second) / double(points);
        for(int i = 0; i < points; i++) {
            checkPoint.first += incrementX;
            checkPoint.second += incrementY;
            if(GeoFunctions::calcMeterDistanceBetweensCoords(checkPoint, circleMidpointCoordinate) < meterRadius)
                return true;
        }
    }
    return false;
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

bool WatchZone::linesIntersects(std::pair<double, double> start1, std::pair<double, double> end1, std::pair<double, double> start2, std::pair<double, double> end2)
{
    double ax = end1.first - start1.first;     // direction of line a
    double ay = end1.second - start1.second;     // ax and ay as above

    double bx = start2.first - end2.first;     // direction of line b, reversed
    double by = start2.second - end2.second;     // really -by and -by as above

    double dx = start2.first - start1.first;   // right-hand side
    double dy = start2.second - start1.second;

    double det = ax * by - ay * bx;

    if (det > -0.00000001 && det < 0.00000001)
        return false;

    double r = (dx * by - dy * bx) / det;
    double s = (ax * dy - ay * dx) / det;

    return !(r < 0 || r > 1 || s < 0 || s > 1);
}

void WatchZone::handleDynamicVisualization()
{
    int timeUntilStart = validFrom - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    while(timeUntilStart > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeUntilStart = validFrom - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if(stopThread) {
            threadStopped = true;
            return;
        }
    }

    for(auto it : nodesInArea)
       it->addToColor(100, 0, 0);

    int timeUntilStop = validTo - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while(timeUntilStop > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeUntilStop = validTo - std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if(stopThread) {
            threadStopped = true;
            return;
        }
    }

    for(auto it : nodesInArea)
        it->addToColor(-200, 0, 0);

    threadStopped = true;
}
