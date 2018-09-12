
#include "headers/node.h"


Node::Node()
{

}

Node::~Node()
{

}

Node::Node(std::pair<double, double> position)
{
    this->position = position;
}

void Node::setNeighbors(std::vector<std::shared_ptr<Node> > nodes)
{
    for(std::shared_ptr<Node> neighbor : nodes) {
        double distance = sqrt(pow(position.first - neighbor->getPosition().first, 2) + pow(position.second - neighbor->getPosition().second, 2));
        NeighborNode newNeighbor(neighbor, distance);
        neighbors.push_back(newNeighbor);
    }
}

void Node::setNodeReadyMutex(std::shared_ptr<std::mutex> mutex)
{
    nodeReadyMutex = mutex;
}

void Node::setCheckNodesAgain(bool *val)
{
    checkNodesAgain = val;
}

void Node::checkAndUpdateNeighbors()
{
    //if(stable)
    //    return;
    stable = true;

    for(NeighborNode &neighbor : neighbors) {
        double costToMove = neighbor.distance + cost;
        neighbor.node->lockAccessNode();
        if(costToMove < neighbor.node->getCost() || !neighbor.node->getUpdated()) {
            neighbor.node->updateSourceAndCost(position, costToMove);
            neighbor.node->setUpdated(true);
            neighbor.node->setStable(false);
            neighbor.node->unlockNodeReady();
        }
        neighbor.node->unlockAccessNode();
    }

}

void Node::unlockNodeReady()
{
    nodeReadyMutex->lock();
    *checkNodesAgain = true;
    nodeReadyMutex->unlock();
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    sourceNode = position;
    cost = 0;
    //nodeReadyMutex->unlock();

    nodeReadyMutex->lock();
    *checkNodesAgain = true;
    nodeReadyMutex->unlock();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> source, double newCost)
{
    sourceNode = source;
    cost = newCost;
}
