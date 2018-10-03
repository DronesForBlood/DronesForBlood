
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
    myPenalty = 0.;
    totalPenalty = 0.;

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
        int costToMove = neighbor.distance + cost;
        int penaltyToMove = neighborNode->getPenalty() + totalPenalty + neighborNode->getPenaltyForDynamicZones(cost);
        int neighborTotalCost = neighborNode->getCost() + neighborNode->getTotalPenalty() - MINIMUM_DISTANCE_CHANGE;
        if((costToMove + penaltyToMove < neighborTotalCost || !neighborNode->getUpdated()) /*&& !(pointerToSelf.lock() == neighborNode->getPointerToSource())*/) {
            neighborNode->updateSourceAndCost(selfNodeIndex, costToMove);
            neighborNode->setTotalPentaly(penaltyToMove);
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

void Node::setPenalty(int val)
{
    int difference = val - myPenalty;
    totalPenalty += difference;
    myPenalty = val;

    if(difference < 0.1 && difference > -0.1)
        return;


    stable = false;

    if(!updated)
        return;

    addToNextTotalPenalty(difference, false);
}

void Node::setCostAndUpdate(int val)
{
    double difference = val - cost;
    addToNextCost(difference, true);
    cost = val;
}

void Node::addToNextCost(int val, bool mayUpdate)
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

void Node::addToNextTotalPenalty(int val, bool mayUpdate)
{
    if(mayUpdate && wasUpdated)
        updated = true;
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->lockAccessNode();
            neighborNode->addToNextTotalPenalty(val, mayUpdate);
            neighborNode->addToTotalPenalty(val);
            neighborNode->unlockAccessNode();
        }
        else {
            neighborNode->setStable(false);
            neighborNode->unlockNodeReady();
        }
    }
}

int Node::getPenaltyForDynamicZones(int cost)
{
    int arrivalTime = int(std::time(nullptr)) + cost / DRONE_MAX_SPEED;

    int penalty = 0;
    for(auto it = dynamicPenalties.begin(); it != dynamicPenalties.end(); it++) {
        if(arrivalTime >= it->epochFrom) {
            if(arrivalTime <= it->epochTo)
                penalty += it->penalty;
            else
                dynamicPenalties.erase(it);
        }
    }
    return penalty;
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    wasUpdated = true;
    pointerToSource = nullptr;
    sourceNodeIndex = selfNodeIndex;

    setCostAndUpdate(0);
    //addToNextTotalPenalty(myPenalty - totalPenalty, true);

    totalPenalty = myPenalty;
    unlockNodeReady();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    cost = newCost;
}
