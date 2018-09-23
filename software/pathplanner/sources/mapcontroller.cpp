
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

MapController::~MapController()
{
    threadRunning = false;

    while(!threadClosed)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void MapController::generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    MapGenerator generator;
    generator.generateMap(map, nodeCollections, startCoord, endCoord, 100, 10000, 5000);
    pathImage = cv::Mat(map->size(), map->at(0).size(), CV_8UC3, cv::Scalar(0, 0, 0));
    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);
}

/* Generates a simple test map of size mapSize x mapSize (see header).
 * One node is generated for each field on the map.
 * All the nodes are then given a pointer to their neighboring nodes.
 * The nodes are then split into square collections.
 */
void MapController::generateTestMap()
{
    MapGenerator generator;
    generator.generateMapTest(map, nodeCollections);
    pathImage = cv::Mat(map->size(), map->at(0).size(), CV_8UC3, cv::Scalar(0, 0, 0));
    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);
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

    /*
    threadRunning = true;
    threadClosed = false;
    mapStatusThread = std::shared_ptr<std::thread>(new std::thread(&MapController::mapStatusUpdater,this));
    mapStatusThread->detach();
    //*/

}

void MapController::setCurrentHeading(std::pair<double, double> headingCoord)
{
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    iterationCounter++;

    currentHeading = std::make_shared<std::pair<std::size_t, std::size_t>>(getClosestNodeIndex(headingCoord));

    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
    //std::cout << "Time2: " << timeCounter/iterationCounter << "  " << iterationCounter << std::endl;

    for(auto it = currentPath.rbegin(); it != currentPath.rend(); ++it) {
        if(*currentHeading.get() == *it) {
            solver.setCurrentHeading(currentHeading);
            //std::cout << "Srsly.. 2" << std::endl;
            return;
        }
    }

    std::cout << "SETTING AS INIT!" << std::endl;
    solver.setInitialPosition(currentHeading);
}

void MapController::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(col, row));
    color->val[2] = 255;
    solver.updatePenaltyOfNode(row, col, penalty);
}

void MapController::updatePenaltyOfNodeGroup(std::vector<std::pair<std::size_t, std::size_t> > &positions, double penalty)
{
    for(std::pair<std::size_t, std::size_t> &pos : positions) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(pos.second, pos.first));
        color->val[2] = 255;
    }

    solver.updatePenaltyOfNodeGroup(positions, penalty);
}

bool MapController::getPathToDestination(std::vector<std::pair<double, double> > &path)
{
    path.clear();
    std::vector<std::pair<std::size_t, std::size_t> > nodePath;

    bool succes = makePathToDestination(goalPosition.first, goalPosition.second, nodePath);

    if(succes)
        printPathImage(nodePath);

    for(std::pair<std::size_t, std::size_t> &nodeIndex : nodePath)
        path.push_back(map->at(nodeIndex.first).at(nodeIndex.second)->getWorldCoordinate());

    return succes;
}


/* Prints the current status of the map with info about node row, col.
 */
void MapController::printMapStatus()
{
    std::vector<std::pair<std::size_t, std::size_t>> path;
    makePathToDestination(goalPosition.first, goalPosition.second, path);

    if(path.back().first != currentHeading->first || path.back().second != currentHeading->second) {
        std::cout << "No path exists from node [" << currentHeading->first << ", " << currentHeading->second << "] to [" << goalPosition.first << ", " << goalPosition.second << "] in map of size " << map->size() << "x" << map->at(0).size() << std::endl;
        return;
    }

    double pathCost = map->at(goalPosition.first).at(goalPosition.second)->getCost();

    if(solver.getMapStable())
        std::cout << "Map stable: True" << std::endl;
    else
        std::cout << "Map stable: False" << std::endl;

    std::cout << "Computation time: " << solver.getCurrentComputationTime() << std::endl;
    std::cout << "Path from node [" << currentHeading->first << ", " << currentHeading->second << "] to [" << goalPosition.first << ", " << goalPosition.second << "] in map of size " << map->size() << "x" << map->at(0).size() << std::endl;
    std::cout << "Length: " << path.size() << std::endl;
    std::cout << "Cost:   " << pathCost << std::endl;

}

void MapController::printPathImage(std::vector<std::pair<std::size_t, std::size_t>> &path)
{
    std::size_t currentPositionIndex = 0;
    for(std::size_t i = 0; i < currentPath.size(); i++) {
        if(currentPath[i] == *currentHeading.get()) {
            currentPositionIndex = i;
            break;
        }
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(currentPath[i].second, currentPath[i].first));
        color->val[0] = 0;
        color->val[1] = 100;
        //color->val[2] = 0;
    }

    if(currentPositionIndex != 0)
        for(std::size_t i = currentPositionIndex; i < currentPath.size(); i++) {
            cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(currentPath[i].second, currentPath[i].first));

            color->val[0] = 255;
            color->val[1] = 0;
            //color->val[2] = 0;
        }

    for(std::size_t i = 0; i < path.size(); i++) {
        cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(path[i].second, path[i].first));
        color->val[0] = 0;
        color->val[1] = 255;
        //color->val[2] = 0;
    }

    currentPath = path;

    cv::imshow("image", pathImage);
    cv::waitKey(1);
}

void MapController::mapStatusUpdater()
{
    while(threadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        std::cout << "INIT COST: " << map->at(initPosition.first).at(initPosition.second)->getCost() << std::endl;

        //*
        double biggestCost = 0;
        for(int i = 0; i < map->size(); i++) {
            for(int j = 0; j < map->at(i).size(); j++) {
                double cost = map->at(i).at(j)->getCost() + map->at(i).at(j)->getPenalty();
                if(cost > biggestCost)
                    biggestCost = cost;
            }
        }

        for(int i = 0; i < map->size(); i++) {
            for(int j = 0; j < map->at(i).size(); j++) {
                double cost = map->at(i).at(j)->getCost() + map->at(i).at(j)->getPenalty();

                cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(j, i));

                color->val[2] = cost / biggestCost * 255.;
            }
        }
        //*/
        /*
        for(int i = 0; i < map->size(); i++) {
            for(int j = 0; j < map->at(i).size(); j++) {
                cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(j, i));
                if(map->at(i).at(j)->getPenalty() > 1)
                    color->val[2] = 255.;
                else
                    color->val[2] = 0;
            }
        }
        //*/
        std::cout << "DONE" << std::endl;


    }
    threadClosed = true;
}

std::pair<std::size_t, std::size_t> MapController::getClosestNodeIndex(std::pair<double, double> worldCoord)
{
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    iterationCounter++;


    std::pair<double, double> nodeCoord = map->at(0).at(0)->getWorldCoordinate();
    double smallestDistance =  calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

    //*
    int currentI = 0;
    int currentJ = 0;
    int newI = 0;
    int newJ = 0;
    bool repeat = true;
    while(repeat) {
        repeat = false;
        for(int i = -1; i < 2; i++) {
            for(int j = -1; j < 2; j++) {
                if(i == 0 && j == 0)
                    continue;

                if(currentI + i >= 0 && currentI + i < map->size() && currentJ + j >= 0 && currentJ + j < map->at(0).size()) {
                    nodeCoord = map->at(currentI + i).at(currentJ + j)->getWorldCoordinate();
                    double distance = calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);
                    //std::cout << "Dist: " << distance << std::endl;
                    if(distance < smallestDistance) {
                        //std::cout << "Distl: " << distance << std::endl;
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
    //std::cout << currentI << " " << currentJ << std::endl;
    std::pair<std::size_t, std::size_t> closestNodeIndex(currentI,currentJ);
    return closestNodeIndex;
    //*/

    /*
    std::cout << "Start" << std::endl;
    std::pair<std::size_t, std::size_t> closestNodeIndex(0,0);
    for(std::size_t i = 0; i < map->size(); i++) {
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            nodeCoord = map->at(i).at(j)->getWorldCoordinate();
            //double distance = sqrt(pow(worldCoord.first - nodeCoord.first, 2) + pow(worldCoord.second - nodeCoord.second, 2));
            double distance = calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);
            //std::cout << "Dist: " << smallestDistance << std::endl;
            if(distance < 100 && distance < smallestDistance) {
                smallestDistance = distance;
                closestNodeIndex.first = i;
                closestNodeIndex.second = j;
                std::cout << "Got one" << std::endl;
            }
        }
    }
    //*/

    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
    std::cout << "Time2: " << timeCounter/iterationCounter << "  " << iterationCounter << std::endl;

    std::cout << "Close: " << closestNodeIndex.first << " " << closestNodeIndex.second << std::endl;
    return  closestNodeIndex;
}


/* Get the path to the node at location row, col.
 */
bool MapController::makePathToDestination(std::size_t row, std::size_t col, std::vector<std::pair<std::size_t, std::size_t> > &path)
{
    if(!map->at(row).at(col)->getUpdated()) {
        return false;
    }

    if(!map->at(row).at(col)->getStable()){
        return false;
    }

    if(!map->at(row).at(col)->getPointerToSource()){
        return false;
    }

    std::pair<std::size_t, std::size_t> pos = map->at(row).at(col)->getSelfIndex();
    path.push_back(pos);

    std::pair<std::size_t,std::size_t> sourcePos = map->at(row).at(col)->getSourceIndex();

    if(sourcePos == *currentHeading.get())
        return true;

    /*if(row == currentHeading->first && col == currentHeading->second){
        return false;
    }*/

    for(int i = 0; i < path.size() - 1; i++)
        if(pos == path[i]) {
            std::cout << "INFINITE ROUTE DUE TO CYCLE. THIS SHOULD NEVER HAPPEN." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            return false;
        }

    return makePathToDestination(sourcePos.first, sourcePos.second, path);
}

double MapController::calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    startCoord.first = startCoord.first * pi / 180.;
    endCoord.first = endCoord.first * pi / 180.;
    startCoord.second = startCoord.second * pi / 180.;
    endCoord.second = endCoord.second * pi / 180.;

    if(startCoord.first - endCoord.first < 0.000000001 && startCoord.second - endCoord.second < 0.000000001)
        return 0;

    double distance_radians = acos(sin(startCoord.first) * sin(endCoord.first) + cos(startCoord.first) * cos(endCoord.first) * cos(startCoord.second - endCoord.second));
    double distanceMeters = distance_radians * radiusEarthMeters;
    return distanceMeters;
}
