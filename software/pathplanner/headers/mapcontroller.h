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

#include "headers/defines.h"
#include "headers/pathfinder.h"
#include "headers/mapgenerator.h"

class MapController
{
public:
    MapController();
    ~MapController();
    void generateMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord);
    std::pair<std::size_t, std::size_t> getMapSize();
    void setGoalPosition(std::pair<double,double> goalCoord) {goalPosition = getClosestNodeIndex(goalCoord);}
    void startSolver(std::pair<double, double> worldCoord);
    void setCurrentHeading(std::pair<double, double> headingCoord);
    void updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty);
    void updatePenaltyOfNodeGroup(std::vector<std::pair<std::size_t, std::size_t> > &positions, double penalty);
    bool getPathToDestination(std::vector<std::pair<double, double> > &path);

    void printPathImage(std::vector<std::pair<std::size_t, std::size_t> > &path);


private:
    std::pair<std::size_t, std::size_t> getClosestNodeIndex(std::pair<double, double> worldCoord);
    bool makePathToDestination(std::pair<std::size_t,std::size_t> pos, std::vector<std::pair<std::size_t, std::size_t> > &path);
    double calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord);

private:
    cv::Mat pathImage;
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::vector<NodeCollection> nodeCollections;
    std::pair<std::size_t,std::size_t> goalPosition;
    std::pair<std::size_t,std::size_t> initPosition;
    std::shared_ptr<std::pair<std::size_t,std::size_t>> currentHeading;
    std::vector<std::pair<std::size_t, std::size_t>> currentPath;
    Pathfinder solver;

    int timeCounter = 0;
    int iterationCounter = 0;
};

#endif // MAPCONTROLLER_H
