
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

void Node::resetNode()
{
    cost = -1.;
    penalty = 0.;

    stable = true;
    updated = false;

    pointerToSource = nullptr;
}

void Node::setNeighbors(std::vector<std::shared_ptr<Node> > nodes)
{
    for(std::weak_ptr<Node> neighbor : nodes) {
        double distance = sqrt(pow(position.first - neighbor.lock()->getPosition().first, 2) + pow(position.second - neighbor.lock()->getPosition().second, 2));
        NeighborNode newNeighbor(neighbor, distance + 0.8);
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
    stable = false;

    if(!updated)
        return;

    addToNextCost(difference);

    setNextStable(false);
}

void Node::setCostAndUpdate(double val)
{
    double difference = val - cost;
    addToNextCost(difference);
    cost = val;
}

void Node::addToNextCost(double &val)
{
    updated = true;
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(pointerToSelf.lock() == neighborNode->getPointerToSource()) {
            neighborNode->addToNextCost(val);
            neighborNode->addToCost(val);
        }
    }
}

void Node::setNodeAsInit()
{
    stable = false;
    updated = true;
    pointerToSource = nullptr;
    sourceNodeIndex = position;

    /*
    //std::cout << "HERE 1" << std::endl;
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    for(NeighborNode &neighbor : neighbors) {
        std::shared_ptr<Node> neighborNode = neighbor.node.lock();
        if(neighborNode->getPointerToSource() != pointerToSelf.lock()) {
            neighborNode->setSource(position);
            neighborNode->setPointerToSource(pointerToSelf.lock());
            neighborNode->setNextUpdated(false);

            neighborNode->setCostAndUpdate(neighbor.distance + neighborNode->getPenalty());
            neighborNode->setStable(false);
        }
        neighborNode->setUpdated(true);

    }
    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    std::cout << "Counter: " << counter  << std::endl;
    std::cout << "Time to complete: " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count() << std::endl;
    //std::cout << "HERE 2" << std::endl;
    //*/

    setCostAndUpdate(0);

    unlockNodeReady();
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> sourceNodeIndex, double newCost)
{
    this->sourceNodeIndex = sourceNodeIndex;
    cost = newCost;

}
