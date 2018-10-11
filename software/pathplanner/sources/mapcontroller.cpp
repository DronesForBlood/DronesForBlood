
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

MapController::~MapController()
{

}

void MapController::generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord, double distanceBetweenNodes, double width, double padLength)
{
    MapGenerator generator;
    generator.generateMap(map, nodeCollections, startCoord, endCoord, distanceBetweenNodes, width, padLength);

    visualizer = std::make_shared<Visualizer>(map, int(map->size()), int(map->at(0).size()));

    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);

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
void MapController::startSolver(std::pair<double, double> worldCoord)
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

bool MapController::updatePenaltyOfAreaCircle(std::pair<double, double> position, double radius, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    solver.pauseSolver();
    std::shared_ptr<WatchZone> zone;

    if(epochValidFrom < 0 || epochValidTo < 0)
        zone = std::make_shared<WatchZone>(map, position, radius, visualizer);
    else
        zone = std::make_shared<WatchZone>(map, position, radius, visualizer, epochValidFrom, epochValidTo);

    watchZones.push_back(zone);

    std::vector<std::shared_ptr<Node>> nodes = zone->getNodesInArea();

    solver.resumeSolver();
    if(epochValidFrom < 0 || epochValidTo < 0)
        solver.updatePenaltyOfNodeGroup(nodes, penalty);


    bool intersectsWithFlightPath = zone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

    return intersectsWithFlightPath;
}

bool MapController::updatePenaltyOfAreaPolygon(std::vector<std::pair<double,double>> polygonCoordinates, double penalty, time_t epochValidFrom, time_t epochValidTo)
{
    solver.pauseSolver();
    std::shared_ptr<WatchZone> zone;

    if(epochValidFrom < 0 || epochValidTo < 0)
        zone = std::make_shared<WatchZone>(map, polygonCoordinates, visualizer);
    else
        zone = std::make_shared<WatchZone>(map, polygonCoordinates, visualizer, epochValidFrom, epochValidTo);

    watchZones.push_back(zone);

    std::vector<std::shared_ptr<Node>> nodes = zone->getNodesInArea();

    solver.resumeSolver();
    if(epochValidFrom < 0 || epochValidTo < 0)
        solver.updatePenaltyOfNodeGroup(nodes, penalty);

    std::cout << "DONE" << std::endl;


    bool intersectsWithFlightPath = zone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

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
        pathShortener.shortenPath(path, shortPathCoords, 100);

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
