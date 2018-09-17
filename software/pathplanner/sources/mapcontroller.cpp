
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::make_shared<std::vector<std::vector<std::shared_ptr<Node>>>>();
}

/* Generates a simple test map of size mapSize x mapSize (see header).
 * One node is generated for each field on the map.
 * All the nodes are then given a pointer to their neighboring nodes.
 * The nodes are then split into square collections.
 */
void MapController::generateTestMap()
{
    MapGenerator generator;
    generator.generateTestMap(map, nodeCollections);
    pathImage = cv::Mat(map->size(), map->at(0).size(), CV_8UC3, cv::Scalar(0, 0, 0));
    solver.setMap(map);
    solver.setNodeCollections(nodeCollections);
}

/* Start the solver
 * It waits to ensure that the solver is in fact ready
 */
void MapController::startSolver(std::pair<std::size_t, std::size_t> position)
{
    currentHeading = std::shared_ptr<std::pair<std::size_t, std::size_t>>(new std::pair<std::size_t, std::size_t>(position));
    solver.setInitialPosition(currentHeading);
    solver.startSolver();


}

void MapController::setCurrentHeading(std::pair<std::size_t, std::size_t> heading)
{
    currentHeading = std::shared_ptr<std::pair<std::size_t, std::size_t>>(new std::pair<std::size_t, std::size_t>(heading));

    for(std::size_t i = 0; i < currentPath.size(); i++)
        if(heading == currentPath[i]) {
            solver.setCurrentHeading(currentHeading);
            return;
        }

    solver.setInitialPosition(currentHeading);
}

void MapController::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    cv::Vec3b *color = &pathImage.at<cv::Vec3b>(cv::Point(col, row));
    color->val[0] = 0;
    color->val[1] = 0;
    color->val[2] = 255;
    solver.updatePenaltyOfNode(row, col, penalty);
}

bool MapController::getPathToDestination(std::vector<std::pair<std::size_t, std::size_t> > &path)
{
    path.clear();
    bool succes = makePathToDestination(goalPosition.first, goalPosition.second, path);

    if(succes)
        printPathImage(path);

    return succes;
}


/* Prints the current status of the map with info about node row, col.
 */
void MapController::printMapStatus()
{
    std::vector<std::pair<std::size_t, std::size_t>> path;
    getPathToDestination(path);

    if(path.back().first != currentPosition->first || path.back().second != currentPosition->second) {
        std::cout << "No path exists from node [" << currentPosition->first << ", " << currentPosition->second << "] to [" << goalPosition.first << ", " << goalPosition.second << "] in map of size " << map->size() << "x" << map->at(0).size() << std::endl;
        return;
    }

    double pathCost = map->at(goalPosition.first).at(goalPosition.second)->getCost();

    if(solver.getMapStable())
        std::cout << "Map stable: True" << std::endl;
    else
        std::cout << "Map stable: False" << std::endl;

    std::cout << "Computation time: " << solver.getCurrentComputationTime() << std::endl;
    std::cout << "Path from node [" << currentPosition->first << ", " << currentPosition->second << "] to [" << goalPosition.first << ", " << goalPosition.second << "] in map of size " << map->size() << "x" << map->at(0).size() << std::endl;
    std::cout << "Length: " << path.size() << std::endl;
    std::cout << "Cost:   " << pathCost << std::endl;

}


/* Prints the map using the costs of the nodes.
 */
void MapController::printCostMap()
{
    std::cout << "Cost map:" << std::endl;
    for(std::size_t i = 0; i < map->size(); i++) {
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            std::cout << int(map->at(i).at(j)->getCost()) << " ";
        }
        std::cout << "\n";
    }
}


/* Prints the path to the node at location row, col.
 */
void MapController::printPathMap()
{
    std::vector<std::vector<char>> pathMap;
    for(std::size_t i = 0; i < map->size(); i++) {
        std::vector<char> temp;
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            temp.push_back('+');
        }
        pathMap.push_back(temp);
    }

    std::vector<std::pair<std::size_t, std::size_t>> path;
    getPathToDestination(path);

    if(path.back().first != currentPosition->first || path.back().second != currentPosition->second) {
        std::cout << "No path exists from node [" << currentPosition->first << ", " << currentPosition->second << "] to [" << goalPosition.first << ", " << goalPosition.second << "] in map of size " << map->size() << "x" << map->at(0).size() << std::endl;
        return;
    }

    for(std::size_t i = 0; i < path.size(); i++)
        pathMap[path[i].first][path[i].second] = 'X';

    for(std::size_t i = 0; i < pathMap.size(); i++) {
        for(std::size_t j = 0; j < pathMap[i].size(); j++)
            std::cout << pathMap[i][j];
        std::cout << std::endl;
    }
}

void MapController::printMap()
{
    for(std::size_t i = 0; i < map->size(); i++) {
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            std::cout << map->at(i).at(j)->getPosition().first << map->at(i).at(j)->getPosition().second << " ";
        }
        std::cout << "\n";
    }
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
    while(true) {
        //printPathImage();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}


/* Get the path to the node at location row, col.
 */
bool MapController::makePathToDestination(std::size_t row, std::size_t col, std::vector<std::pair<std::size_t, std::size_t> > &path)
{
    if(!map->at(row).at(col)->getUpdated())
        return false;

    if(!map->at(row).at(col)->getStable())
        return false;

    if(!map->at(row).at(col)->getPointerToSource())
        return false;

    std::pair<std::size_t, std::size_t> pos = map->at(row).at(col)->getPosition();
    path.push_back(pos);

    std::pair<std::size_t,std::size_t> sourcePos = map->at(row).at(col)->getSourceIndex();

    if(sourcePos.first == currentHeading->first && sourcePos.second == currentHeading->second)
        return true;

    if(row == currentHeading->first && col == currentHeading->second)
        return false;

    for(int i = 0; i < path.size() - 1; i++)
        if(pos == path[i]) {
            std::cout << "INFINITE ROUTE DUE TO CYCLE. THIS SHOULD NEVER HAPPEN." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            int crash = 0;
            int now = 1/crash;
            crash = now;
            return false;
        }

    return makePathToDestination(sourcePos.first, sourcePos.second, path);
}
