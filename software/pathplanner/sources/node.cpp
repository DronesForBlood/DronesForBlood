
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

    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
}

void Node::addToColor(int r, int g, int b)
{
    color[0] += b;
    color[1] += g;
    color[2] += r;
}

void Node::addDynamicPenalty(int penalty, int epochFrom, int epochTo)
{
    DynamicPenalty dynamicPenalty(penalty, epochFrom, epochTo);
    dynamicPenalties.push_back(dynamicPenalty);

    int difference = 0;
    if(willBeInDynamicZone(dynamicPenalty))
        difference = dynamicPenalty.penalty;

    //updated = false;

    if(difference > 0) {
        penalty += difference;
        totalPenalty += difference;

        stable = false;

        addToNextTotalPenalty(difference, false);
    }

}

void Node::resetNode()
{
    cost = -1.;
    //myPenalty = 0.;
    totalPenalty = myPenalty;

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
    int currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int lateArrivalTime = currentTime + (cost - 1000) / DRONE_MAX_SPEED;
    int earlyArrivalTime = currentTime + (cost + 1000) / DRONE_MAX_SPEED;

    int penalty = 0;
    for(auto it = dynamicPenalties.begin(); it != dynamicPenalties.end(); it++)
        if(willBeInDynamicZone(*it))
            penalty = it->penalty;

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

bool Node::willBeInDynamicZone(DynamicPenalty &dynamic)
{
    int currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int lateArrivalTime = currentTime + (cost - 1000) / DRONE_MAX_SPEED;
    int earlyArrivalTime = currentTime + (cost + 1000) / DRONE_MAX_SPEED;

    if(currentTime < dynamic.epochTo)
        if(earlyArrivalTime >= dynamic.epochFrom)
            if(lateArrivalTime <= dynamic.epochTo)
                return true;
    return false;
}
