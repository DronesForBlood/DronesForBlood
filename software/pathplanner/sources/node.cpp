
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
    for(std::weak_ptr<Node> neighbor : nodes) {
        double distance = sqrt(pow(position.first - neighbor.lock()->getPosition().first, 2) + pow(position.second - neighbor.lock()->getPosition().second, 2));
        NeighborNode newNeighbor(neighbor, distance);
        neighbors.push_back(newNeighbor);
    }
}

void Node::checkAndUpdateNeighbors()
{
    if(!updated)
        return;

    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        neighborNode->lockAccessNode();
        double costToMove = neighbor.distance + neighborNode->getPenalty() + cost;
        if(costToMove < neighborNode->getCost() || !neighborNode->getUpdated()) {
            neighborNode->updateSourceAndCost(position, costToMove);
            neighborNode->setUpdated(true);
            neighborNode->setStable(false);
            neighborNode->setPointerToSource(pointerToSelf.lock());
            neighborNode->unlockNodeReady();
        }
        neighborNode->unlockAccessNode();
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

void Node::setNextUpdated(bool val)
{
    updated = val;
    for(NeighborNode &neighbor : neighbors) {
        if(pointerToSelf.lock() == neighbor.node.lock()->getPointerToSource()) {
            neighbor.node.lock()->setNextUpdated(val);
        }
    }
}

void Node::setPenalty(double val)
{
    double difference = val - penalty;
    penalty = val;
    cost += difference;
    updated = true;
    addToNextCost(difference); // NOT SURE ABOUT THIS LINE. MAY IMPROVE PERFORMANCE, MAY CRASH. IDK
}

void Node::setCost(double val)
{
    double difference = val - cost;
    cost = val;
    addToNextCost(difference);
}

void Node::addToNextCost(double val)
{
    for(NeighborNode &neighbor : neighbors) {
        if(pointerToSelf.lock() == neighbor.node.lock()->getPointerToSource()) {
            neighbor.node.lock()->addToCost(val);
            neighbor.node.lock()->addToNextCost(val);
            //neighbor.node->setUpdated(true);
        }
    }
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    sourceNodeIndex = position;
    cost = 0;

    //pointerToSelf = nullptr;
    //pointerToSource = pointerToSelf;

    unlockNodeReady();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    //addToCost(newCost - cost);  // NOT SURE ABOUT THIS LINE. MAY IMPROVE PERFORMANCE, MAY CRASH. IDK
    cost = newCost;

}
