
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

void Node::checkAndUpdateNeighbors()
{
    if(!updated)
        return;

    for(NeighborNode &neighbor : neighbors) {
        double costToMove = neighbor.distance + neighbor.node->getPenalty() + cost;
        neighbor.node->lockAccessNode();
        if(costToMove < neighbor.node->getCost() || !neighbor.node->getUpdated()) {
            neighbor.node->updateSourceAndCost(position, costToMove);
            neighbor.node->setUpdated(true);
            neighbor.node->setStable(false);
            neighbor.node->setPointerToSource(pointerToSelf);
            neighbor.node->unlockNodeReady();
        }
        neighbor.node->unlockAccessNode();
    }
    stable = true;
}

void Node::unlockNodeReady()
{
    if(!*checkNodesAgain) {
        nodeReadyMutex->lock();
        *checkNodesAgain = true;
        nodeReadyMutex->unlock();
    }
}

void Node::setPenalty(double val)
{
    double difference = val - penalty;
    penalty = val;
    cost += difference;
    //addToNextCost(difference);
}

void Node::addToNextCost(double val)
{
    for(NeighborNode neighbor : neighbors) {
        if(pointerToSelf == neighbor.node->getPointerToSource()) {
            neighbor.node->addToCost(val);
            neighbor.node->addToNextCost(val);
            neighbor.node->setUpdated(true);
        }
    }
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    sourceNodeIndex = position;
    cost = 0;

    pointerToSelf = nullptr;

    nodeReadyMutex->lock();
    *checkNodesAgain = true;
    nodeReadyMutex->unlock();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    cost = newCost;
}
