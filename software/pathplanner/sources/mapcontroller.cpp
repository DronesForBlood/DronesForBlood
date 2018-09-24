
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

MapController::~MapController()
{

}

void MapController::generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    MapGenerator generator;
    generator.generateMap(map, nodeCollections, startCoord, endCoord, 100, 10000, 5000);
    pathImage = cv::Mat(int(map->size()), int(map->at(0).size()), CV_8UC3, cv::Scalar(0, 0, 0));
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
}

void MapController::setCurrentHeading(std::pair<double, double> headingCoord)
{
    currentHeading = std::make_shared<std::pair<std::size_t, std::size_t>>(getClosestNodeIndex(headingCoord));

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


            return;
        }
    }

    std::cout << "SETTING AS INIT!" << std::endl;
    solver.setInitialPosition(currentHeading);
}

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
}

bool MapController::getPathToDestination(std::vector<std::pair<double, double> > &path)
{
    path.clear();
    std::vector<std::pair<std::size_t, std::size_t> > nodePath;

    bool succes = makePathToDestination(goalPosition, nodePath);

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
}

std::pair<std::size_t, std::size_t> MapController::getClosestNodeIndex(std::pair<double, double> worldCoord)
{
    std::pair<double, double> nodeCoord = map->at(0).at(0)->getWorldCoordinate();
    double smallestDistance =  calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

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

                if(currentI + i >= 0 && currentI + i < int(map->size()) && currentJ + j >= 0 && currentJ + j < int(map->at(0).size())) {
                    nodeCoord = map->at(std::size_t(currentI + i)).at(std::size_t(currentJ + j))->getWorldCoordinate();
                    double distance = calcMeterDistanceBetweensCoords(worldCoord, nodeCoord);

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
}
