
#include "headers/node.h"


Node::Node()
{

}

Node::Node(std::pair<double, double> position)
{
    this->position = position;
}

double Node::getCost()
{
    return cost;
}

std::pair<double, double> Node::getPosition()
{
    return position;
}

std::pair<double, double> Node::getSourcePosition()
{
    return sourceNode;
}

void Node::setNodeAsInit()
{
    sourceNode = position;
    cost = 0;
}

void Node::updateSourceAndCost(std::pair<std::size_t, std::size_t> source, double newCost)
{
    sourceNode = source;
    cost = newCost;
}
