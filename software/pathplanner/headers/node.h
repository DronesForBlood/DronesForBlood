#ifndef NODE_H
#define NODE_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>
<<<<<<< HEAD
#include <opencv2/core/core.hpp>

#include "headers/global/defines.h"
#include "headers/global/geofunctions.h"
=======

#include "headers/defines.h"
>>>>>>> develop

class Node;

struct NeighborNode {
    NeighborNode() {}
    NeighborNode(std::weak_ptr<Node> node, int distance) {
        this->node = node;
        this->distance = distance;
<<<<<<< HEAD
=======
        //std::cout << distance << std::endl;
>>>>>>> develop
    }
    std::weak_ptr<Node> node;
    int distance;
};

<<<<<<< HEAD
struct DynamicPenalty {
    DynamicPenalty() {}
    DynamicPenalty(std::string ID, int penalty, int epochFrom, int epochTo) {
        this->ID = ID;
        this->penalty = penalty;
        this->epochFrom = epochFrom;
        this->epochTo = epochTo;
    }
    std::string ID;
    int penalty;
    int epochFrom;
    int epochTo;
};

=======
>>>>>>> develop
class Node
{
public:
    Node();
    ~Node();

    Node(std::pair<std::size_t, std::size_t> index, std::pair<double, double> coordinate);
<<<<<<< HEAD
    void addToColor(int r, int g, int b);
    cv::Scalar getColor() {return color;}
    bool checkIfNodeIsInDangerZone(double distanceToNode);
    void removeDynamicPenalty(std::string ID);
    void addDynamicPenalty(std::string ID, int penalty, int epochFrom, int epochTo);
=======
>>>>>>> develop
    void resetNode();
    void setPointerToSelf(std::weak_ptr<Node> pointer) {pointerToSelf = pointer;}
    void setNeighbors(std::vector<std::shared_ptr<Node>> nodes);
    void setNodeReadyMutex(std::shared_ptr<std::mutex> mutex) {nodeReadyMutex = mutex;}
    void setCheckNodesAgain(bool *val) {checkNodesAgain = val;}

    void checkAndUpdateNeighbors();

    void lockAccessNode() {accessNodeMutex.lock();}
    bool tryLockAccessNode() {return accessNodeMutex.try_lock();}
    void unlockAccessNode() {accessNodeMutex.unlock();}

    void unlockNodeReady();

    void setUpdated(bool val) {wasUpdated = updated; updated = val;}
    void setNextUpdated(bool val);
    bool getUpdated() {return updated;}

    void setStable(bool val) {stable = val;}
    void setNextStable(bool val);
    bool getStable() {return stable;}

<<<<<<< HEAD
    int getCost() {return cost;}
    int getPenalty() {return myPenalty;}
    int getTotalPenalty() {return totalPenalty;}
    void addToCost(int val) {cost += val;}
    void addToTotalPenalty(int val) {totalPenalty += val;}
    void setPenalty(int val);
    void setTotalPentaly(int val) {totalPenalty = val;}
    void setCostAndUpdate(int val);
    void addToNextCost(int val, bool mayUpdate);
    void addToNextTotalPenalty(int val, bool mayUpdate);

    int getPenaltyForDynamicZones(int cost);

    std::pair<std::size_t, std::size_t> getNodeIndex() {return selfNodeIndex;}
=======
    double getCost() {return cost;}
    double getPenalty() {return penalty;}
    void addToCost(double val) {cost += val;}
    void setPenalty(double val);
    void setCostAndUpdate(double val);
    void addToNextCost(double val, bool mayUpdate);

>>>>>>> develop
    std::pair<double, double> getWorldCoordinate() {return worldCoordinate;}
    std::pair<std::size_t, std::size_t> getSourceIndex() {return sourceNodeIndex;}
    std::shared_ptr<Node> getPointerToSource() {return pointerToSource;}

    void setNodeAsInit();
    void updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost);
    void setPointerToSource(std::shared_ptr<Node> pointer) {pointerToSource = pointer;}

private:
<<<<<<< HEAD
    bool willBeInDynamicZone(DynamicPenalty &dynamic, int distanceToNode);
=======
    int calcMeterDistanceBetweensCoords(std::pair<double,double> startCoord, std::pair<double,double> endCoord);
>>>>>>> develop

private:
    std::weak_ptr<Node> pointerToSelf;
    std::shared_ptr<Node> pointerToSource = nullptr;
    std::pair<std::size_t, std::size_t> selfNodeIndex;
    std::pair<std::size_t, std::size_t> sourceNodeIndex;

<<<<<<< HEAD
    std::vector<DynamicPenalty> dynamicPenalties;

    std::pair<double, double> worldCoordinate;

    int cost = -1.;
    int myPenalty = 0.;
    int totalPenalty = 0.;
=======
    std::pair<double, double> worldCoordinate;

    double cost = -1.;
    double penalty = 0.;
>>>>>>> develop

    bool stable = true;
    bool updated = false;
    bool wasUpdated = false;

    std::vector<NeighborNode> neighbors;

    std::mutex accessNodeMutex;
    std::shared_ptr<std::mutex> nodeReadyMutex;
    bool *checkNodesAgain;

<<<<<<< HEAD
    int epochETA = 0;

    cv::Scalar color;
=======

>>>>>>> develop
};

#endif // NODE_H
