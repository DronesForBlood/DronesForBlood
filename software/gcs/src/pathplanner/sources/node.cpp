
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

bool Node::checkIfNodeIsInDangerZone(double distanceToNode)
{
    for(auto &it : dynamicPenalties)
        if(willBeInDynamicZone(it, distanceToNode))
            return true;
    return false;
}

void Node::removeDynamicPenalty(std::string ID)
{
    for(int i = 0; i < dynamicPenalties.size(); i++)
        if(dynamicPenalties[i].ID == ID) {
            dynamicPenalties.erase(dynamicPenalties.begin(), dynamicPenalties.begin() + i);
            break;
        }
}

void Node::addDynamicPenalty(std::string ID, int penalty, int epochFrom, int epochTo)
{
    DynamicPenalty *dynamicPenalty;

    bool exists = false;
    /*
    for(auto &it : dynamicPenalties)
        if(it.ID == ID) {
            dynamicPenalty = &it;
            dynamicPenalty->ID = ID;
            dynamicPenalty->penalty = penalty;
            dynamicPenalty->epochFrom = epochFrom;
            dynamicPenalty->epochTo = epochTo;
            exists = true;
            break;
        }
        */



    if(!exists) {
        DynamicPenalty newDynamicPenalty(ID, penalty, epochFrom, epochTo);
        dynamicPenalties.push_back(newDynamicPenalty);
        dynamicPenalty = &newDynamicPenalty;
    }

    int difference = 0;
    if(willBeInDynamicZone(*dynamicPenalty, cost)) {
        difference = dynamicPenalty->penalty;
    }

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
        double distance = GeoFunctions::calcMeterDistanceBetweensCoords(worldCoordinate, neighbor.lock()->getWorldCoordinate());
        NeighborNode newNeighbor(neighbor, distance);
        neighbors.push_back(newNeighbor);
    }
}

void Node::checkAndUpdateNeighbors()
{
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        neighborNode->lockAccessNode();
        double costToMove = neighbor.distance + cost;
        int penaltyToMove = neighborNode->getPenalty() + totalPenalty + neighborNode->getPenaltyForDynamicZones(cost);
        double neighborTotalCost = neighborNode->getCost() + neighborNode->getTotalPenalty(); // MINIMUM_DISTANCE_CHANGE;
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

int Node::getPenaltyForDynamicZones(double cost)
{
    int penalty = 0;
    for(auto it = dynamicPenalties.begin(); it != dynamicPenalties.end(); it++)
        if(willBeInDynamicZone(*it, cost))
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

bool Node::willBeInDynamicZone(DynamicPenalty &dynamic, double distanceToNode)
{
    int currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    int lateArrivalTime = currentTime + (distanceToNode - 1000) / DRONE_MAX_SPEED;
    int earlyArrivalTime = currentTime + (distanceToNode + 1000) / DRONE_MAX_SPEED;

    /*
    std::cout << "earlyArrivalTime:  " << earlyArrivalTime << std::endl;
    std::cout << "dynamic.epochFrom: " << dynamic.epochFrom << std::endl;
    std::cout << "lateArrivalTime:   " << lateArrivalTime << std::endl;
    std::cout << "dynamic.epochTo:   " << dynamic.epochTo << std::endl;
    */

    if(currentTime < dynamic.epochTo)
        if(earlyArrivalTime >= dynamic.epochFrom)
            if(lateArrivalTime <= dynamic.epochTo)
                return true;
    return false;
}
