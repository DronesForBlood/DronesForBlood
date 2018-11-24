
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

MapController::~MapController()
{

}

void MapController::addPreMapPenaltyOfAreaCircle(std::pair<double, double> position, double radius, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    PreMapPenaltyCircle temp;
    temp.position = position;
    temp.radius = radius;
    temp.penalty = penalty;
    temp.epochValidFrom = epochValidFrom;
    temp.epochValidTo = epochValidTo;
    preMapPenaltyCircles.push_back(temp);
}

void MapController::addPreMapPenaltyOfAreaPolygon(std::vector<std::pair<double, double> > polygonCoordinates, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    PreMapPenaltyArea temp;
    temp.polygonCoordinates = polygonCoordinates;
    temp.penalty = penalty;
    temp.epochValidFrom = epochValidFrom;
    temp.epochValidTo = epochValidTo;
    preMapPenaltyAreas.push_back(temp);
}

void MapController::generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord, double distanceBetweenNodes, double width, double padLength)
{
    MapGenerator generator;
    generator.generateMap(map, nodeCollections, startCoord, endCoord, distanceBetweenNodes, width, padLength);

    visualizer = std::make_shared<Visualizer>(map, int(map->size()), int(map->at(0).size()));

    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);

    std::cout << "generateNewMap circles" << std::endl;
    for(auto &it : preMapPenaltyCircles) {
        std::cout << "generateNewMap begin" << std::endl;
        std::cout << "it.position: " << it.position.first << " " << it.position.second << std::endl;
        std::cout << "it.radius: " << it.radius << std::endl;
        std::cout << "it.penalty: " << it.penalty << std::endl;
        std::cout << "it.epochValidFrom: " << it.epochValidFrom << std::endl;
        std::cout << "it.epochValidTo: " << it.epochValidTo << std::endl;
        updatePenaltyOfAreaCircle(it.position, it.radius, it.penalty, it.epochValidFrom, it.epochValidTo);
        std::cout << "generateNewMap done" << std::endl;
    }

    std::cout << "generateNewMap areas" << std::endl;
    for(auto &it : preMapPenaltyAreas)
        updatePenaltyOfAreaPolygon(it.polygonCoordinates, it.penalty, it.epochValidFrom, it.epochValidTo);

    std::cout << "generateNewMap done?" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mapReady = true;
}

std::pair<std::size_t, std::size_t> MapController::getMapSize()
{
    return std::pair<std::size_t, std::size_t>(map->size(), map->at(0).size());
}

/* Start the solver
 * It waits to ensure that the solver is in fact ready
 */
void MapController::startSolver(std::pair<double, double> worldCoord) // takes start coordinate 
{
    //std::cout << "Scalar: " << std::is_scalar<std::pair<std::size_t, std::size_t>>::value << std::endl;

    currentHeading = std::make_shared<std::pair<std::size_t, std::size_t>>(getClosestNodeIndex(worldCoord));
    initPosition = *currentHeading.get();

    solver.setInitialPosition(currentHeading);
    solver.startSolver();
}

void MapController::setCurrentHeading(std::pair<double, double> headingCoord)
{
    currentHeading = std::make_shared<std::pair<std::size_t, std::size_t>>(getClosestNodeIndex(headingCoord));

    for(auto it = currentPath.rbegin(); it != currentPath.rend(); ++it) {
        if(*currentHeading.get() == *it) {
            solver.setCurrentHeading(currentHeading);
            return;
        }
    }

    std::cout << "SETTING AS INIT!" << std::endl;
    solver.setInitialPosition(currentHeading);
}

void MapController::setCurrentPosition(std::pair<double, double> currentCoord)
{
    //visualizer->printCurrentPositionImage(getClosestNodeIndex(currentCoord));
    visualizer->setCurrentPosition(getClosestNodeIndex(currentCoord));
    currentPosition = currentCoord;
}

bool MapController::updateDrone(std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp, std::pair<double, double> position, std::pair<double, double> nextPosition)
{
    map->at(getClosestNodeIndex(position).first).at(getClosestNodeIndex(position).second)->addToColor(255,0,0);
    //map->at(getClosestNodeIndex(nextPosition).first).at(getClosestNodeIndex(nextPosition).second)->addToColor(255,255,255);
    std::pair<double, double> midPoint(nextPosition.first + (position.first - nextPosition.first) / 2, nextPosition.second + (position.second - nextPosition.second) / 2);
    //map->at(getClosestNodeIndex(midPoint).first).at(getClosestNodeIndex(midPoint).second)->addToColor(255,255,255);

    std::shared_ptr<WatchDrone> drone = std::make_shared<WatchDrone>(map, aDroneID, aName, operationStatus, trackingEntry, aGPSTimestamp);


    bool exists = false;
    for(auto &it : watchDrones)
        if(it->getID() == aDroneID) {
            drone = it;
            exists = true;
            break;
        }

    if(!exists)
        watchDrones.push_back(drone);

    drone->setNextWaypoint(nextPosition, 1, 1, 1, 1);

    solver.pauseSolver();
    drone->updateCurrentPosition(position, 100, 10, 100);
    solver.resumeSolver();

    bool intersectsWithFlightPath = drone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

    if(intersectsWithFlightPath)
        return checkIfIntersectionIsDangerous();
    return false;
}

bool MapController::updatePenaltyOfAreaCircle(std::pair<double, double> position, double radius, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    std::shared_ptr<WatchZone> zone;

    bool staticZone = (epochValidFrom < 0 || epochValidTo < 0);

    if(staticZone) {
        zone = std::make_shared<WatchZone>(map, position, radius, visualizer);
        std::vector<std::shared_ptr<Node>> nodes = zone->getNodesInArea();
        solver.updatePenaltyOfNodeGroup(nodes, penalty);
    }
    else {
        std::cout << "Test 1" << std::endl;
        solver.pauseSolver();
        std::cout << "Test 2" << std::endl;
        zone = std::make_shared<WatchZone>(map, position, radius, visualizer, epochValidFrom, epochValidTo);
        std::cout << "Test 3" << std::endl;
        solver.resumeSolver();
        std::cout << "Test 4" << std::endl;
    }

    watchZones.push_back(zone);

    bool intersectsWithFlightPath = false;
    if(currentHeading)
        intersectsWithFlightPath = zone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

    return intersectsWithFlightPath;
}

bool MapController::updatePenaltyOfAreaPolygon(std::vector<std::pair<double,double>> polygonCoordinates, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    double minDistance = 10000;
    bool closerThanMinDistance = false;
    for(auto &it : polygonCoordinates) {
        double distanceInit = GeoFunctions::calcMeterDistanceBetweensCoords(initPosition, it);
        double distanceGoal = GeoFunctions::calcMeterDistanceBetweensCoords(goalPosition, it);

        if(distanceInit < minDistance || distanceGoal < minDistance) {
            closerThanMinDistance = true;
            break;
        }
    }

    if(!closerThanMinDistance)
        return false;


    std::cout << "Area Test 1" << std::endl;
    solver.pauseSolver();
    std::shared_ptr<WatchZone> zone;

    if(epochValidFrom < 0 || epochValidTo < 0)
        zone = std::make_shared<WatchZone>(map, polygonCoordinates, visualizer);
    else
        zone = std::make_shared<WatchZone>(map, polygonCoordinates, visualizer, epochValidFrom, epochValidTo);

    watchZones.push_back(zone);
    std::cout << "Area Test 2" << std::endl;

    std::vector<std::shared_ptr<Node>> nodes = zone->getNodesInArea();

    std::cout << "nodes: " << nodes.size() << std::endl;
    std::cout << "Area Test 2.1" << std::endl;
    solver.resumeSolver();
    if(epochValidFrom < 0 || epochValidTo < 0) {
        std::cout << "Area Test 2.2" << std::endl;
        if(nodes.size() > 0)
            solver.updatePenaltyOfNodeGroup(nodes, penalty);
        std::cout << "Area Test 2.3" << std::endl;
    }

    std::cout << "Area Test 3" << std::endl;
    bool intersectsWithFlightPath = false;
    if(currentHeading)
        intersectsWithFlightPath = zone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

    std::cout << "Area Test 4" << std::endl;
    return intersectsWithFlightPath;
}

bool MapController::getPathToDestination(std::vector<std::pair<double, double> > &path)
{
    path.clear();
    std::vector<std::pair<std::size_t, std::size_t> > nodePath;

    bool succes = makePathToDestination(goalPosition, nodePath);

    if(succes) {
        for(const auto &nodeIndex : nodePath)
            path.push_back(map->at(nodeIndex.first).at(nodeIndex.second)->getWorldCoordinate());

        PathShortener pathShortener;
        std::vector<std::pair<double, double> > shortPathCoords;
        pathShortener.shortenPath(path, shortPathCoords, 1);

        std::vector<std::pair<std::size_t, std::size_t> > shortPath;

        for(auto &it : shortPathCoords)
            shortPath.push_back(getClosestNodeIndex(it));

        //visualizer->printPathImage(nodePath, *currentHeading.get());
        //visualizer->printShortPathImage(shortPath, *currentHeading.get());
        visualizer->setCurrentPath(nodePath);
        visualizer->setCurrentShortPath(shortPath);
        visualizer->setCurrentHeading(*currentHeading.get());

        currentPath = nodePath;
        currentShortPath = shortPath;
        path = shortPathCoords;

    }

    return succes;
}

std::pair<std::size_t, std::size_t> MapController::getClosestNodeIndex(std::pair<double, double> worldCoord)
{
    int currentI = int(map->size()/2);
    int currentJ = int(map->at(0).size()/2);

    std::pair<double, double> nodeCoord = map->at(size_t(currentI)).at(size_t(currentJ))->getWorldCoordinate();

    double smallestDistance = GeoFunctions::calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

    int newI = 0;
    int newJ = 0;
    bool repeat = true;
    while(repeat) {
        repeat = false;
        for(int i = -1; i < 2; i++) {
            for(int j = -1; j < 2; j++) {
                if(i == 0 && j == 0)
                    continue;

                if(isInsideMap(currentI + i, currentJ + j)) {
                    nodeCoord = map->at(std::size_t(currentI + i)).at(std::size_t(currentJ + j))->getWorldCoordinate();
                    double distance = GeoFunctions::calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

                    if(distance < smallestDistance) {
                        smallestDistance = distance;
                        newI = currentI + i;
                        newJ = currentJ + j;
                        repeat = true;
                    }
                }
            }
        }
        if(repeat) {
            currentI = newI;
            currentJ = newJ;
        }

    }

    std::pair<std::size_t, std::size_t> closestNodeIndex(currentI,currentJ);
    return closestNodeIndex;
}


/* Get the path to the node at location row, col.
 */
bool MapController::makePathToDestination(std::pair<std::size_t,std::size_t> pos, std::vector<std::pair<std::size_t, std::size_t> > &path)
{
    std::shared_ptr<Node> currentNode = map->at(pos.first).at(pos.second);
    if(!currentNode->getUpdated() ||
       !currentNode->getStable()  ||
       !currentNode->getPointerToSource()) {
        return false;
    }

    for(auto it = path.begin(); it != path.end(); ++it)
        if(pos == *it) {
            std::cout << "INFINITE ROUTE DUE TO CYCLE. THIS SHOULD NEVER HAPPEN." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            return false;
        }

    path.push_back(pos);
    std::pair<std::size_t,std::size_t> sourcePos = currentNode->getSourceIndex();

    if(sourcePos == *currentHeading.get())
        return true;

    return makePathToDestination(sourcePos, path);
}

bool MapController::isInsideMap(int row, int col)
{
    if(row < 0 || col < 0)
        return false;

    if(row >= int(map->size()))
        return false;

    if(col >= int(map->at(std::size_t(row)).size()))
        return false;

    return true;
}

bool MapController::checkIfIntersectionIsDangerous()
{
    std::cout << "Checking for danger" << std::endl;
    bool onCurrentPath = false;
    for(auto it = currentPath.rbegin(); it != currentPath.rend(); ++it) {
        if(!onCurrentPath && *it == getClosestNodeIndex(currentPosition))
            onCurrentPath = true;

        if(onCurrentPath) {
            double distanceToNode = GeoFunctions::calcMeterDistanceBetweensCoords(currentPosition, (*map)[it->first][it->second]->getWorldCoordinate());
            if((*map)[it->first][it->second]->checkIfNodeIsInDangerZone(distanceToNode)) {
                std::cout << "DANGER" << std::endl;
                return true;
            }
        }

        if(*currentHeading.get() == *it)
            return false;
    }
    return false;
}
