
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
        if((costToMove < neighborNode->getCost() || !neighborNode->getUpdated()) /*&& !(pointerToSelf.lock() == neighborNode->getPointerToSource())*/) {
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
            neighbor.node.lock()->setPointerToSource(nullptr);
        }
    }
}

void Node::setNextStable(bool val)
{
    stable = val;
    for(NeighborNode &neighbor : neighbors) {
        if(pointerToSelf.lock() == neighbor.node.lock()->getPointerToSource()) {
            neighbor.node.lock()->setNextStable(val);
        }
        else {
            neighbor.node.lock()->setStable(val);
            neighbor.node.lock()->unlockNodeReady();
        }
    }
}

void Node::setPenalty(double val)
{
    double difference = val - penalty;
    cost += difference;
    penalty = val;

    if(!updated)
        return;

    addToNextCost(difference);
    setNextStable(false);
}

void Node::setCost(double val)
{
    double difference = val - cost;
    cost = val;
    addToNextCost(difference);
}

void Node::addToNextCost(double &val)
{
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->addToCost(val);
            neighborNode->addToNextCost(val);
        }
    }
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    sourceNodeIndex = position;
    cost = 0;

    unlockNodeReady();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    cost = newCost;

}
