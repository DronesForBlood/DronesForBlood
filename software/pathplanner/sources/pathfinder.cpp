
#include "headers/pathfinder.h"

Pathfinder::Pathfinder()
{
    //srand(time(0));
    generateMap();
    //printMap();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    solve(5, 5);

    bool temp = false;
    while(!temp) {
        temp = true;

        /*
        for(std::size_t i = 0; i < map.size(); i++) {
            for(std::size_t j = 0; j < map[i].size(); j++) {
                  if(!map[i][j]->getUpdated())
                      temp = false;
            }
        }
        */
        for(NodeCollection collection : nodeCollections)
            if(!collection.isDone())
                temp = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        //std::cout << "CHECK: " << temp << std::endl;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

    path.clear();
    makePathToDestination(0, mapSize-1);

    double pathCost = map[0][mapSize-1]->getCost();

    //printCostMap();
    std::cout << "Done. Length of path: " << path.size() << " Cost: " << pathCost << std::endl;
    //printPathMap(0,9);
}

void Pathfinder::solve(std::size_t initRow, std::size_t initCol)
{
    map[initRow][initCol]->setNodeAsInit();
    for(NodeCollection &collection : nodeCollections)
        collection.start();

    std::cout << "START!\n";

}

void Pathfinder::calculateNeighbors(std::size_t row, std::size_t col, std::vector<std::shared_ptr<Node>> &neighborNodes)
{
    // Top line
    if(row > 0 && col > 0)
        neighborNodes.push_back(map[row - 1][col - 1]);

    if(row > 0)
        neighborNodes.push_back(map[row - 1][col]);

    if(row > 0 && col < map[row].size() - 1)
        neighborNodes.push_back(map[row - 1][col + 1]);


    // Middle line
    if(col > 0)
        neighborNodes.push_back(map[row][col - 1]);

    if(col < map[row].size() - 1)
        neighborNodes.push_back(map[row][col + 1]);


    // Bottom line
    if(row < map.size() - 1 && col > 0)
        neighborNodes.push_back(map[row + 1][col - 1]);

    if(row < map.size() - 1)
        neighborNodes.push_back(map[row + 1][col]);

    if(row < map.size() - 1 && col < map[row].size()  - 1)
        neighborNodes.push_back(map[row + 1][col + 1]);
}

void Pathfinder::generateMap()
{
    map.clear();

    // Generate a simple map
    for(int i = 0; i < mapSize; i++) {
        std::vector<std::shared_ptr<Node>> row;
        for(int j = 0; j < mapSize; j++) {
            std::pair<double, double> pos(i,j);
            std::shared_ptr<Node> newNode(new Node(pos));
            row.push_back(newNode);
        }
        map.push_back(row);
    }

    // Give all nodes a pointer to their neighbors
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::vector<std::shared_ptr<Node>> neighborNodes;
            calculateNeighbors(i, j, neighborNodes);
            std::shared_ptr<Node> currentNode = map[i][j];
            currentNode->setNeighbors(neighborNodes);
        }
    }

    divideIntoCollections();
}

void Pathfinder::divideIntoCollections()
{
    std::size_t gridRows = 40;
    std::size_t gridCols = 40;

    std::size_t rowCount = map.size() / gridRows;
    std::size_t colCount = map.size() / gridCols;

    for(std::size_t i = 0; i < gridRows; i++) {
        for(std::size_t j = 0; j < gridCols; j++) {
        NodeCollection temp;

        std::size_t thisRowCount = rowCount;
        if((i+1) % gridRows == 0)
            thisRowCount += map.size() % gridRows;

        std::size_t thisColCount = colCount;
        if((j+1) % gridCols == 0)
            thisColCount += map.size() % gridCols;

        getSlice(i*rowCount, j*colCount, thisRowCount, thisColCount, temp);
        nodeCollections.push_back(temp);
        }
    }
}

void Pathfinder::getSlice(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, NodeCollection &collection)
{
    std::vector<std::vector<std::shared_ptr<Node> > > temp;
    for(std::size_t i = 0; i < rowCount; i++)
        for(std::size_t j = 0; j < colCount; j++)
            collection.addNode(map[i+row][j+col]);
}

void Pathfinder::printMap()
{
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << map[i][j]->getPosition().first << map[i][j]->getPosition().second << " ";
        }
        std::cout << "\n";
    }
}

void Pathfinder::printCostMap()
{
    std::cout << "Cost" << std::endl;
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << int(map[i][j]->getCost()) << " ";
        }
        std::cout << "\n";
    }
}

void Pathfinder::printPathMap(std::size_t row, std::size_t col)
{
    std::vector<std::vector<char>> pathMap;
    for(std::size_t i = 0; i < map.size(); i++) {
        std::vector<char> temp;
        for(std::size_t j = 0; j < map[i].size(); j++) {
            temp.push_back('+');
        }
        pathMap.push_back(temp);
    }

    path.clear();
    makePathToDestination(row, col);

    for(std::size_t i = 0; i < path.size(); i++)
        pathMap[path[i].first][path[i].second] = 'X';

    for(std::size_t i = 0; i < pathMap.size(); i++) {
        for(std::size_t j = 0; j < pathMap[i].size(); j++)
            std::cout << pathMap[i][j];
        std::cout << std::endl;
    }
}

void Pathfinder::makePathToDestination(std::size_t row, std::size_t col)
{
    std::pair<std::size_t,std::size_t> sourcePos = map[row][col]->getSourceIndex();
    std::pair<double,double> pos = map[row][col]->getPosition();
    //std::cout << "PATH: " << pos.first << " " << pos.second << std::endl;
    path.push_back(pos);

    if(sourcePos.first == row && sourcePos.second == col)
        return;

    makePathToDestination(sourcePos.first, sourcePos.second);
}
