
#include <utility>

class Node
{
public:
    Node();
    Node(std::pair<double, double> position);

    double getCost();
    std::pair<double, double> getPosition();
    std::pair<double, double> getSourcePosition();

    void setNodeAsInit();
    void updateSourceAndCost(std::pair<std::size_t, std::size_t> source, double newCost);

private:
    std::pair<std::size_t, std::size_t> sourceNode;
    double cost = -1.;
    std::pair<double, double> position;
};
