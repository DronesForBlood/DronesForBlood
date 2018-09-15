#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include <utility>
#include <vector>
#include <iostream>
#include <math.h>
#include <memory>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


#include "headers/pathfinder.h"
#include "headers/mapgenerator.h"

class MapController
{
public:
    MapController();
    void generateTestMap();
    void setGoalPosition(std::pair<std::size_t,std::size_t> goal) {goalPosition = goal;}
    void startSolver();
    void setCurrentPosition(std::pair<std::size_t,std::size_t> position);
    void setCurrentHeading(std::pair<std::size_t,std::size_t> heading);
    void updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty);
    bool getMapStable() {return  solver.getMapStable();}
    void getPathToDestination(std::vector<std::pair<std::size_t, std::size_t> > &path);

    void printMapStatus();
    void printCostMap();
    void printPathMap();
    void printMap();

    void printPathImage();


private:
    [[ noreturn ]] void mapStatusUpdater();
    void makePathToDestination(std::size_t row, std::size_t col, std::vector<std::pair<std::size_t, std::size_t> > &path);

private:
    cv::Mat pathImage;
    std::shared_ptr<std::thread> mapStatusThread;
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::vector<NodeCollection> nodeCollections;
    std::pair<std::size_t,std::size_t> goalPosition;
    std::shared_ptr<std::pair<std::size_t,std::size_t>> currentPosition;
    std::shared_ptr<std::pair<std::size_t,std::size_t>> currentHeading;
    std::vector<std::pair<std::size_t, std::size_t>> currentPath;
    Pathfinder solver;
};

#endif // MAPCONTROLLER_H
