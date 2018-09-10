
#include "headers/pathfinder.h"

Pathfinder::Pathfinder()
{
    generateMap();
    printMap();
    solve(0, 0);
    printCostMap();
    printPathToDestination(19,9);
    printCostMap();
}

void Pathfinder::solve(std::size_t initRow, std::size_t initCol)
{

    map[initRow][initCol].setNodeAsInit();

    for(int i = 0; i < 10; i++) {
        for(std::size_t i = 0; i < map.size(); i++) {
            for(std::size_t j = 0; j < map[i].size(); j++) {
                Node *currentNode = &map[i][j];
                if(currentNode->getCost() < 0)
                    continue;
                std::vector<Node *> neighborNodes;
                calculateNeighbors(i, j, neighborNodes);
                for(Node *node : neighborNodes) {
                    double costToMove = sqrt(pow(currentNode->getPosition().first - node->getPosition().first, 2) + pow(currentNode->getPosition().second - node->getPosition().second, 2));
                    costToMove += currentNode->getCost();
                    if(costToMove < node->getCost() || node->getCost() < 0) {
                        node->updateSourceAndCost(currentNode->getPosition(), costToMove);
                    }
                }
            }
        }
    }

}

void Pathfinder::calculateNeighbors(std::size_t row, std::size_t col, std::vector<Node *> &neighborNodes)
{
    // Top line
    if(row > 0 && col > 0)
        neighborNodes.push_back(&map[row - 1][col - 1]);

    if(row > 0)
        neighborNodes.push_back(&map[row - 1][col]);

    if(row > 0 && col < map[row].size() - 1)
        neighborNodes.push_back(&map[row - 1][col + 1]);


    // Middle line
    if(col > 0)
        neighborNodes.push_back(&map[row][col - 1]);

    if(col < map[row].size() - 1)
        neighborNodes.push_back(&map[row][col + 1]);


    // Bottom line
    if(row < map.size() - 1 && col > 0)
        neighborNodes.push_back(&map[row + 1][col - 1]);

    if(row < map.size() - 1)
        neighborNodes.push_back(&map[row + 1][col]);

    if(row < map.size() - 1 && col < map[row].size()  - 1)
        neighborNodes.push_back(&map[row + 1][col + 1]);
}

void Pathfinder::generateMap()
{
    map.clear();

    for(int i = 0; i < 20; i++) {
        std::vector<Node> row;
        for(int j = 0; j < 20; j++) {
            std::pair<double, double> pos(i,j);
            Node newNode(pos);
            row.push_back(newNode);
        }
        map.push_back(row);
    }
}

void Pathfinder::printMap()
{
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << map[i][j].getPosition().first << map[i][j].getPosition().second << " ";
        }
        std::cout << "\n";
    }
}

void Pathfinder::printCostMap()
{
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << int(map[i][j].getCost()) << " ";
        }
        std::cout << "\n";
    }
}

void Pathfinder::printPathToDestination(std::size_t row, std::size_t col)
{
    std::pair<std::size_t,std::size_t> sourcePos = map[row][col].getSourcePosition();
    std::pair<double,double> pos = map[row][col].getPosition();
    std::cout << pos.first << " " << pos.second << std::endl;



    if(sourcePos.first == row && sourcePos.second == col)
        return;

    printPathToDestination(sourcePos.first, sourcePos.second);

    Node temp(std::pair<double,double>(0,0));
    temp.updateSourceAndCost(std::pair<std::size_t,std::size_t>(0,0), -1);
    map[sourcePos.first][sourcePos.second] = temp;
}
