#ifndef NODE_H
#define NODE_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>


class Node;

struct NeighborNode {
    NeighborNode() {}
    NeighborNode(std::weak_ptr<Node> node, int distance) {
        this->node = node;
        this->distance = distance;
        //std::cout << distance << std::endl;
    }
    std::weak_ptr<Node> node;
    int distance;
};

class Node
{
public:
    Node();
    ~Node();

    Node(std::pair<std::size_t, std::size_t> index, std::pair<double, double> coordinate);
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

    void setUpdated(bool val) {updated = val;}
    void setNextUpdated(bool val);
    bool getUpdated() {return updated;}

    void setStable(bool val) {stable = val;}
    void setNextStable(bool val);
    bool getStable() {return stable;}

    double getCost() {return cost;}
    double getPenalty() {return penalty;}
    void addToCost(double val) {cost += val;}
    void setPenalty(double val);
    void setCostAndUpdate(double val);
    void addToNextCost(double val);

    std::pair<double, double> getWorldCoordinate() {return worldCoordinate;}
    std::pair<std::size_t, std::size_t> getSelfIndex() {return selfNodeIndex;}
    std::pair<std::size_t, std::size_t> getSourceIndex() {return sourceNodeIndex;}
    std::shared_ptr<Node> getPointerToSource() {return pointerToSource;}

    void setNodeAsInit();
    void updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost);
    void setSource(std::pair<std::size_t, std::size_t> source) {sourceNodeIndex = source;}
    void setPointerToSource(std::shared_ptr<Node> pointer) {pointerToSource = pointer;}


private:
    int calcMeterDistanceBetweensCoords(std::pair<double,double> startCoord, std::pair<double,double> endCoord);

private:
    const double pi = 3.14159265359;
    const double radiusEarthMeters = 6371000.0;
    const double minimumDistanceDifference = 25;


    std::weak_ptr<Node> pointerToSelf;
    std::shared_ptr<Node> pointerToSource = nullptr;
    std::pair<std::size_t, std::size_t> selfNodeIndex;
    std::pair<std::size_t, std::size_t> sourceNodeIndex;

    std::pair<double, double> worldCoordinate;

    double cost = -1.;
    double penalty = 0.;

    bool stable = true;
    bool updated = false;

    std::vector<NeighborNode> neighbors;

    std::mutex accessNodeMutex;
    std::shared_ptr<std::mutex> nodeReadyMutex;
    bool *checkNodesAgain;


};

#endif // NODE_H
