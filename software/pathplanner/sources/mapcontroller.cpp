
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

MapController::~MapController()
{

}

<<<<<<< HEAD
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

    for(auto &it : preMapPenaltyCircles)
        updatePenaltyOfAreaCircle(it.position, it.radius, it.penalty, it.epochValidFrom, it.epochValidTo);

    for(auto &it : preMapPenaltyAreas)
        updatePenaltyOfAreaPolygon(it.polygonCoordinates, it.penalty, it.epochValidFrom, it.epochValidTo);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    mapReady = true;
=======
void MapController::generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    MapGenerator generator;
    generator.generateMap(map, nodeCollections, startCoord, endCoord, 100, 10000, 5000);
    pathImage = cv::Mat(int(map->size()), int(map->at(0).size()), CV_8UC3, cv::Scalar(0, 0, 0));
    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);
>>>>>>> develop
}

std::pair<std::size_t, std::size_t> MapController::getMapSize()
{
    return std::pair<std::size_t, std::size_t>(map->size(), map->at(0).size());
}

/* Start the solver
 * It waits to ensure that the solver is in fact ready
 */
<<<<<<< HEAD
void MapController::startSolver(std::pair<double, double> worldCoord) // takes start coordinate 
=======
void MapController::startSolver(std::pair<double, double> worldCoord)
>>>>>>> develop
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

<<<<<<< HEAD
    for(auto it = currentPath.rbegin(); it != currentPath.rend(); ++it) {
        if(*currentHeading.get() == *it) {
            solver.setCurrentHeading(currentHeading);
=======
    /*
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    iterationCounter++;
    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
    std::cout << "Time2: " << timeCounter/iterationCounter << "  " << iterationCounter << std::endl;
    */

    for(auto it = currentPath.rbegin(); it != currentPath.rend(); ++it) {
        if(*currentHeading.get() == *it) {
            solver.setCurrentHeading(currentHeading);


>>>>>>> develop
            return;
        }
    }

    std::cout << "SETTING AS INIT!" << std::endl;
    solver.setInitialPosition(currentHeading);
}

<<<<<<< HEAD
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
        solver.pauseSolver();
        zone = std::make_shared<WatchZone>(map, position, radius, visualizer, epochValidFrom, epochValidTo);
        solver.resumeSolver();
    }

    watchZones.push_back(zone);

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

    bool intersectsWithFlightPath = zone->checkLineIntersect(currentPosition, map->at(currentHeading->first).at(currentHeading->second)->getWorldCoordinate());

    return intersectsWithFlightPath;
=======
void MapController::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(col), int(row)));
    color->val[2] = 255;
    solver.updatePenaltyOfNode(row, col, penalty);
}

void MapController::updatePenaltyOfNodeGroup(std::vector<std::pair<std::size_t, std::size_t> > &positions, double penalty)
{
    for(std::pair<std::size_t, std::size_t> &pos : positions) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(pos.second), int(pos.first)));
        color->val[2] = 255;
    }

    solver.updatePenaltyOfNodeGroup(positions, penalty);
>>>>>>> develop
}

bool MapController::getPathToDestination(std::vector<std::pair<double, double> > &path)
{
    path.clear();
    std::vector<std::pair<std::size_t, std::size_t> > nodePath;

    bool succes = makePathToDestination(goalPosition, nodePath);

<<<<<<< HEAD
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
=======
    if(succes)
        printPathImage(nodePath);

    for(const auto &nodeIndex : nodePath)
        path.push_back(map->at(nodeIndex.first).at(nodeIndex.second)->getWorldCoordinate());

    return succes;
}

void MapController::printPathImage(std::vector<std::pair<std::size_t, std::size_t>> &path)
{
    std::size_t currentPositionIndex = 0;
    for(std::size_t i = 0; i < currentPath.size(); i++) {
        if(currentPath[i] == *currentHeading.get()) {
            currentPositionIndex = i;
            break;
        }
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(currentPath[i].second), int(currentPath[i].first)));
        color->val[0] = 0;
        color->val[1] = 100;
    }

    if(currentPositionIndex != 0)
        for(std::size_t i = currentPositionIndex; i < currentPath.size(); i++) {
            cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(currentPath[i].second), int(currentPath[i].first)));

            color->val[0] = 255;
            color->val[1] = 0;
        }

    for(std::size_t i = 0; i < path.size(); i++) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(int(path[i].second), int(path[i].first)));
        color->val[0] = 0;
        color->val[1] = 255;
    }

    currentPath = path;

    cv::imshow("image", pathImage);
    cv::waitKey(1);
>>>>>>> develop
}

std::pair<std::size_t, std::size_t> MapController::getClosestNodeIndex(std::pair<double, double> worldCoord)
{
<<<<<<< HEAD
    int currentI = int(map->size()/2);
    int currentJ = int(map->at(0).size()/2);

    std::pair<double, double> nodeCoord = map->at(size_t(currentI)).at(size_t(currentJ))->getWorldCoordinate();

    double smallestDistance = GeoFunctions::calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

=======
    std::pair<double, double> nodeCoord = map->at(0).at(0)->getWorldCoordinate();
    double smallestDistance =  calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

    int currentI = 0;
    int currentJ = 0;
>>>>>>> develop
    int newI = 0;
    int newJ = 0;
    bool repeat = true;
    while(repeat) {
        repeat = false;
        for(int i = -1; i < 2; i++) {
            for(int j = -1; j < 2; j++) {
                if(i == 0 && j == 0)
                    continue;

<<<<<<< HEAD
                if(isInsideMap(currentI + i, currentJ + j)) {
                    nodeCoord = map->at(std::size_t(currentI + i)).at(std::size_t(currentJ + j))->getWorldCoordinate();
                    double distance = GeoFunctions::calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);
=======
                if(currentI + i >= 0 && currentI + i < int(map->size()) && currentJ + j >= 0 && currentJ + j < int(map->at(0).size())) {
                    nodeCoord = map->at(std::size_t(currentI + i)).at(std::size_t(currentJ + j))->getWorldCoordinate();
                    double distance = calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);
>>>>>>> develop

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

<<<<<<< HEAD
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
=======
double MapController::calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    startCoord.first = startCoord.first * PI / 180.;
    endCoord.first = endCoord.first * PI / 180.;
    startCoord.second = startCoord.second * PI / 180.;
    endCoord.second = endCoord.second * PI / 180.;

    if(startCoord.first - endCoord.first < 0.000000001 && startCoord.second - endCoord.second < 0.000000001)
        return 0;

    double distance_radians = acos(sin(startCoord.first) * sin(endCoord.first) + cos(startCoord.first) * cos(endCoord.first) * cos(startCoord.second - endCoord.second));
    double distanceMeters = distance_radians * RADIUS_EARTH_METERS;
    return distanceMeters;
>>>>>>> develop
}
