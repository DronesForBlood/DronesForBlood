
#include "headers/mapcontroller.h"

MapController::MapController()
{
    map = std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>>(new std::vector<std::vector<std::shared_ptr<Node>>>);
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
void MapController::startSolver()
{
    solver.startSolver();
    setCurrentHeading(*currentPosition.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //mapStatusThread = std::shared_ptr<std::thread>(new std::thread(&MapController::mapStatusUpdater,this));
    //mapStatusThread->detach();
}

/* Sets the current position of the map.
 * The position MUST be on the current path as of now.
 */
void MapController::setCurrentPosition(std::pair<std::size_t, std::size_t> position)
{
    currentPosition = std::shared_ptr<std::pair<std::size_t, std::size_t>>(new std::pair<std::size_t, std::size_t>(position));
    solver.setCurrentPosition(currentPosition);
}

void MapController::setCurrentHeading(std::pair<std::size_t, std::size_t> heading)
{
    currentHeading = std::shared_ptr<std::pair<std::size_t, std::size_t>>(new std::pair<std::size_t, std::size_t>(heading));
    solver.setCurrentHeading(currentHeading);
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
    std::cout << "Making path" << std::endl;
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
    if(!map->at(row).at(col)->getUpdated()) {
        std::cout << "Done making path 0" << std::endl;
        return false;
    }

    //std::cout << "Begin: " << path.size() << std::endl;
    if(!map->at(row).at(col)->getPointerToSource()) {
        std::cout << "Done making path 1" << std::endl;
        return false;
    }

    //std::cout << "Begin 2" << std::endl;
    std::pair<std::size_t, std::size_t> pos = map->at(row).at(col)->getPosition();
    //std::cout << "Begin 3" << std::endl;



    path.push_back(pos);
    //std::cout << "Begin 4" << std::endl;

    std::pair<std::size_t,std::size_t> sourcePos = map->at(row).at(col)->getSourceIndex();

    //std::cout << "Begin 5" << std::endl;
    //std::cout << "Row:    " << row << " " << col << std::endl;
    //std::cout << "Head:   " << currentHeading->first << " " << currentHeading->second << std::endl;
    //std::cout << "Source: " << sourcePos.first << " " << sourcePos.second << std::endl;

    if(sourcePos.first == currentHeading->first && sourcePos.second == currentHeading->second) {
        //std::cout << "POS: " << pos.first << " " << pos.second << std::endl;
        std::cout << "Done making path 2" << std::endl;
        return true;
    }

    //std::cout << "Begin 6" << std::endl;
    if(row == currentHeading->first && col == currentHeading->second) {
        std::cout << "Done making path 3" << std::endl;
        return false;
    }

    for(int i = 0; i < path.size() - 1; i++)
        if(pos == path[i]) {
            std::cout << "Current pos: " << currentHeading->first << " " << currentHeading->second << std::endl;
            for(int j = 0; j < path.size(); j++)
                std::cout << "L path: " << path[j].first << " " << path[j].second << std::endl;
            std::cout << "Done making path 4" << std::endl;
            return false;
        }

    //std::cout << "Begin 7" << std::endl;
    return makePathToDestination(sourcePos.first, sourcePos.second, path);
}
