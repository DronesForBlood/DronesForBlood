
#include "headers/node.h"

Node::Node()
{

}

Node::~Node()
{

}

Node::Node(std::pair<std::size_t, std::size_t> index, std::pair<double, double> coordinate)
{
    selfNodeIndex = index;
    worldCoordinate = coordinate;
}

void Node::resetNode()
{
    cost = -1.;
    penalty = 0.;

    stable = true;
    updated = false;
    wasUpdated = false;

    pointerToSource = nullptr;
}

void Node::setNeighbors(std::vector<std::shared_ptr<Node> > nodes)
{
    for(std::weak_ptr<Node> neighbor : nodes) {
        int distance = int(GeoFunctions::calcMeterDistanceBetweensCoords(worldCoordinate, neighbor.lock()->getWorldCoordinate()));
        NeighborNode newNeighbor(neighbor, distance);
        neighbors.push_back(newNeighbor);
    }
}

void Node::checkAndUpdateNeighbors()
{
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        neighborNode->lockAccessNode();
        double costToMove = neighbor.distance + neighborNode->getPenalty() + cost;
        if((costToMove < neighborNode->getCost() - MINIMUM_DISTANCE_CHANGE || !neighborNode->getUpdated()) /*&& !(pointerToSelf.lock() == neighborNode->getPointerToSource())*/) {
            neighborNode->updateSourceAndCost(selfNodeIndex, costToMove);
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
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->setNextUpdated(val);
            neighborNode->setPointerToSource(nullptr);
        }
    }
}

void Node::setNextStable(bool val)
{
    stable = val;
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->setNextStable(val);
        }
        else {
            neighborNode->setStable(false);
            neighborNode->unlockNodeReady();
        }
    }
}

void Node::setPenalty(double val)
{
    double difference = val - penalty;
    cost += difference;
    penalty = val;

    if(difference < 0.1 && difference > -0.1)
        return;


    stable = false;

    if(!updated)
        return;

    addToNextCost(difference, false);

    setNextStable(stable);

}

void Node::setCostAndUpdate(double val)
{
    double difference = val - cost;
    addToNextCost(difference, true);
    cost = val;
}

void Node::addToNextCost(double val, bool mayUpdate)
{
    if(mayUpdate && wasUpdated)
        updated = true;
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->lockAccessNode();
            neighborNode->addToNextCost(val, mayUpdate);
            neighborNode->addToCost(val);
            neighborNode->unlockAccessNode();
        }
    }
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    wasUpdated = true;
    pointerToSource = nullptr;
    sourceNodeIndex = selfNodeIndex;

    setCostAndUpdate(0);

    unlockNodeReady();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    cost = newCost;
}
