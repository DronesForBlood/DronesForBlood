
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
    //std::cout << "start" << std::endl;
    for(std::weak_ptr<Node> neighbor : nodes) {
        //double distance = sqrt(pow(worldCoordinate.first - neighbor.lock()->getWorldCoordinate().first, 2) + pow(worldCoordinate.second - neighbor.lock()->getWorldCoordinate().second, 2));
        int distance = calcMeterDistanceBetweensCoords(worldCoordinate, neighbor.lock()->getWorldCoordinate());
        NeighborNode newNeighbor(neighbor, distance);
        neighbors.push_back(newNeighbor);
        //std::cout << neighbor.lock()->getWorldCoordinate().first << " " << neighbor.lock()->getWorldCoordinate().second << std::endl;
    }
}

void Node::checkAndUpdateNeighbors()
{
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        neighborNode->lockAccessNode();
        double costToMove = neighbor.distance + neighborNode->getPenalty() + cost;
        if((costToMove < neighborNode->getCost() - minimumDistanceDifference || !neighborNode->getUpdated()) /*&& !(pointerToSelf.lock() == neighborNode->getPointerToSource())*/) {
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
            neighborNode->setStable(val);
            neighborNode->unlockNodeReady();
        }
    }
}

void Node::setPenalty(double val)
{
    //std::cout << "C\n";
    double difference = val - penalty;
    cost += difference;
    penalty = val;
    //std::cout << "P: " << cost << std::endl;
    stable = false;

    if(!updated)
        return;

    //std::cout << "c\n"; // Crashed after this
    addToNextCost(difference, false);
    //std::cout << "D\n";
    setNextStable(stable);
    //std::cout << "E\n";
}

void Node::setCostAndUpdate(double val)
{
    //std::cout << "A\n";
    double difference = val - cost;
    addToNextCost(difference, true);
    cost = val;
    //std::cout << "B\n";
}

void Node::addToNextCost(double val, bool mayUpdate)
{
    if(mayUpdate && wasUpdated)
        updated = true;
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->addToNextCost(val, mayUpdate);
            neighborNode->addToCost(val);
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

int Node::calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    double lat1 = startCoord.first;
    double lon1 = startCoord.second;
    double lat2 = endCoord.first;
    double lon2 = endCoord.second;

    lat1 = lat1 * pi / 180.;
    lat2 = lat2 * pi / 180.;
    lon1 = lon1 * pi / 180.;
    lon2 = lon2 * pi / 180.;

    double distance_radians = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2));
    int distanceMeters = int(distance_radians * radiusEarthMeters);
    return distanceMeters;
}
