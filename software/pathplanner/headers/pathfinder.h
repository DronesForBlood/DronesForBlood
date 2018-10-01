#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <utility>
#include <vector>
#include <iostream>
#include <math.h>
#include <memory>
#include <unistd.h>

#include "nodecollection.h"

class Pathfinder
{
public:
    Pathfinder();
    ~Pathfinder();
    void setMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> aMap) {map = aMap;}
    void setNodeCollections(std::vector<NodeCollection> collections) {nodeCollections = collections;}
    void startSolver();
    void setInitialPosition(std::shared_ptr<std::pair<std::size_t,std::size_t>> position);
    void setCurrentHeading(std::shared_ptr<std::pair<std::size_t,std::size_t>> heading);
    void updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty);
    void updatePenaltyOfNodeGroup(std::vector<std::shared_ptr<Node> > &nodes, double penalty);

private:
    void pauseSolver();
    void resumeSolver();

private:
    std::shared_ptr<std::thread> mapStatusThread;
    std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map;
    std::vector<NodeCollection> nodeCollections;
    std::shared_ptr<std::mutex> collectionControlMutex;

    bool threadRunning = false;
    bool threadClosed = true;

    std::shared_ptr<std::pair<std::size_t,std::size_t>> currentHeading;
};

#endif // PATHFINDER_H
