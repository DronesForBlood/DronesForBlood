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
    void setNeighbors(std::vector<std::shared_ptr<Node>> nodes);
    void setNodeReadyMutex(std::shared_ptr<std::mutex> mutex);
    void setCheckNodesAgain(bool *val);


    void checkAndUpdateNeighbors();

    void lockAccessNode() {accessNodeMutex.lock();}
    void unlockAccessNode() {accessNodeMutex.unlock();}

    void unlockNodeReady();

    void setUpdated(bool val) {updated = val;}
    bool getUpdated() {return updated;}

    void setStable(bool val) {stable = val;}
    bool getStable() {return stable;}

    double getCost() {return cost;}
    std::pair<double, double> getPosition() {return position;}
    std::pair<double, double> getSourceIndex() {return sourceNode;}

    void setNodeAsInit();
    void updateSourceAndCost(std::pair<std::size_t, std::size_t> source, double newCost);

private:
    std::pair<std::size_t, std::size_t> sourceNode;
    std::pair<double, double> position;
    double cost = -1.;

    bool stable = true;
    bool updated = false;

    std::vector<NeighborNode> neighbors;

    std::mutex accessNodeMutex;
    std::shared_ptr<std::mutex> nodeReadyMutex;
    bool *checkNodesAgain;

};

#endif // NODE_H
