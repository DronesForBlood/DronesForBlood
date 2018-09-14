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
    NeighborNode(std::shared_ptr<Node> node, double distance) {
        this->node = node;
        this->distance = distance;
        this->distance = rand() % 100 + 1;
    }
    std::shared_ptr<Node> node;
    double distance;
};

class Node
{
public:
    Node();
    ~Node();

    Node(std::pair<double, double> position);
    void setPointerToSelf(std::shared_ptr<Node> pointer) {pointerToSelf = pointer;}
    void setNeighbors(std::vector<std::shared_ptr<Node>> nodes);
    void setNodeReadyMutex(std::shared_ptr<std::mutex> mutex) {nodeReadyMutex = mutex;}
    void setCheckNodesAgain(bool *val) {checkNodesAgain = val;}

    void checkAndUpdateNeighbors();

    void lockAccessNode() {accessNodeMutex.lock();}
    void unlockAccessNode() {accessNodeMutex.unlock();}

    void unlockNodeReady();

    void setUpdated(bool val) {updated = val;}
    bool getUpdated() {return updated;}

    void setStable(bool val) {stable = val;}
    bool getStable() {return stable;}

    double getCost() {return cost;}
    double getPenalty() {return penalty;}
    void addToCost(double val) {cost += val;}
    void setPenalty(double val);
    void addToNextCost(double val);

    std::pair<double, double> getPosition() {return position;}
    std::pair<std::size_t, std::size_t> getSourceIndex() {return sourceNodeIndex;}
    std::shared_ptr<Node> getPointerToSource() {return pointerToSource;}

    void setNodeAsInit();
    void updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost);
    void setSource(std::pair<std::size_t, std::size_t> source) {sourceNodeIndex = source;}
    void setPointerToSource(std::shared_ptr<Node> pointer) {pointerToSource = pointer;}

private:
    std::shared_ptr<Node> pointerToSelf;
    std::shared_ptr<Node> pointerToSource;
    std::pair<std::size_t, std::size_t> sourceNodeIndex;
    std::pair<double, double> position;

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
