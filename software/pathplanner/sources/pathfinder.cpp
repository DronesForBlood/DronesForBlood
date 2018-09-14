
#include "headers/pathfinder.h"

Pathfinder::Pathfinder()
{
    collectionControlMutex = std::shared_ptr<std::mutex>(new std::mutex);
}

void Pathfinder::setCurrentPosition(std::pair<std::size_t, std::size_t> position)
{
    currentPosition = position;
}


/* Starts the solver.
 * The inital node is set at location startRow, startCol in the map matrix. The cost of the initial node is 0.
 * All NodeCollection threads are then started and wait to be activated.
 * The mapStatusChecker thread is then started.
 */
void Pathfinder::startSolver()
{
    // Start the thread that monitors the status of the map.
    mapStatusThread = std::shared_ptr<std::thread>(new std::thread(&Pathfinder::mapStatusChecker,this));
    mapStatusThread->detach();

    // Set inital node and start solving the map.
    map[currentPosition.first][currentPosition.second]->setNodeAsInit();
    for(NodeCollection &collection : nodeCollections)
        collection.start();
}


/* Generates a simple test map of size mapSize x mapSize (see header).
 * One node is generated for each field on the map.
 * All the nodes are then given a pointer to their neighboring nodes.
 * The nodes are then split into square collections.
 */
void Pathfinder::generateTestMap()
{
    //srand(time(0)); // Set a seed for the random generation of distances in the map. Used in nodes.h
    map.clear();

    // Generate a simple map of nodes.
    for(int i = 0; i < mapSize; i++) {
        std::vector<std::shared_ptr<Node>> row;
        for(int j = 0; j < mapSize; j++) {
            std::pair<double, double> pos(i,j);
            std::shared_ptr<Node> newNode(new Node(pos));
            newNode->setPointerToSelf(newNode);
            row.push_back(newNode);
        }
        map.push_back(row);
    }

    // Give all nodes a pointer to their neighbors.
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::vector<std::shared_ptr<Node>> neighborNodes;
            calculateNeighbors(i, j, neighborNodes);
            std::shared_ptr<Node> currentNode = map[i][j];
            currentNode->setNeighbors(neighborNodes);
        }
    }

    // Divide the nodes into square collections.
    divideIntoCollections();
}

void Pathfinder::updatePenaltyOfNode(std::size_t row, std::size_t col, double penalty)
{
    collectionControlMutex->lock();

    for(NodeCollection &collection : nodeCollections)
        collection.pause();

    for(NodeCollection &collection : nodeCollections)
        while(!collection.getIsPaused())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));


    for(std::size_t i = 0; i < map.size(); i++)
        for(std::size_t j = 0; j < map[i].size(); j++) {
            map[i][j]->setUpdated(false);
            //map[i][j]->setStable(false);
        }

    map[row][col]->setPenalty(penalty);
    map[row][col]->setUpdated(false);
    map[currentPosition.first][currentPosition.second]->setStable(false);
    map[currentPosition.first][currentPosition.second]->setUpdated(true);
    map[currentPosition.first][currentPosition.second]->setSource(currentPosition);
    map[currentPosition.first][currentPosition.second]->unlockNodeReady();

    for(NodeCollection &collection : nodeCollections)
        collection.resume();

    collectionControlMutex->unlock();

    if(!mapIsStable)
        timeMeasureBegin = std::chrono::steady_clock::now();
    mapIsStable = false;

    std::cout << "Penalty of node updated!" << std::endl;
}


/* Get the path to the node at location row, col.
 */
void Pathfinder::makePathToDestination(std::size_t row, std::size_t col, std::vector<std::pair<std::size_t, std::size_t> > &path)
{
    std::pair<std::size_t,std::size_t> sourcePos = map[row][col]->getSourceIndex();

    //std::cout << "PATH1: " << sourcePos.first << " " << sourcePos.second << std::endl;
    std::pair<double,double> pos = map[row][col]->getPosition();
    //std::cout << "PATH: " << pos.first << " " << pos.second << std::endl;
    path.push_back(pos);

    if(sourcePos.first == row && sourcePos.second == col)
        return;

    makePathToDestination(sourcePos.first, sourcePos.second, path);
}


/* Prints the current status of the map with info about node row, col.
 */
void Pathfinder::printMapStatus(std::size_t row, std::size_t col)
{
    std::vector<std::pair<std::size_t, std::size_t>> path;
    makePathToDestination(row, col, path);

    if(path.back().first != currentPosition.first || path.back().second != currentPosition.second) {
        std::cout << "No path exists from node [" << currentPosition.first << ", " << currentPosition.second << "] to [" << row << ", " << col << "] in map of size " << map.size() << "x" << map[0].size() << std::endl;
        return;
    }

    double pathCost = map[row][col]->getCost();

    if(mapIsStable) {
        std::cout << "Map stable: True" << std::endl;
        std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeMeasureEnd - timeMeasureBegin).count() <<std::endl;
    }
    else {
        std::cout << "Map stable: False" << std::endl;
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeMeasureBegin).count() <<std::endl;
    }

    std::cout << "Path from node [" << currentPosition.first << ", " << currentPosition.second << "] to [" << row << ", " << col << "] in map of size " << map.size() << "x" << map[0].size() << std::endl;
    std::cout << "Length: " << path.size() << std::endl;
    std::cout << "Cost:   " << pathCost << std::endl;

}


/* Prints the map using the costs of the nodes.
 */
void Pathfinder::printCostMap()
{
    std::cout << "Cost map:" << std::endl;
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << int(map[i][j]->getCost()) << " ";
        }
        std::cout << "\n";
    }
}


/* Prints the path to the node at location row, col.
 */
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

    std::vector<std::pair<std::size_t, std::size_t>> path;
    makePathToDestination(row, col, path);

    if(path.back().first != currentPosition.first || path.back().second != currentPosition.second) {
        std::cout << "No path exists from node [" << currentPosition.first << ", " << currentPosition.second << "] to [" << row << ", " << col << "] in map of size " << map.size() << "x" << map[0].size() << std::endl;
        return;
    }

    for(std::size_t i = 0; i < path.size(); i++)
        pathMap[path[i].first][path[i].second] = 'X';

    for(std::size_t i = 0; i < pathMap.size(); i++) {
        for(std::size_t j = 0; j < pathMap[i].size(); j++)
            std::cout << pathMap[i][j];
        std::cout << std::endl;
    }
}

/* Checks the current status of the map.
 * When the map is stable, the thread running this function sleeps.
 * When unstable, the stability of the map is checked every 100 ms. When stable, it returns to the wait state.
 */
void Pathfinder::mapStatusChecker()
{
    while(true) {
        waitForMapUnstable();

        timeMeasureBegin = std::chrono::steady_clock::now(); // Start to measure computation time of solver.
        waitForMapStable();
        timeMeasureEnd = std::chrono::steady_clock::now();

        //printCostMap();

        //printMapStatus(0, mapSize - 1);
    }
}


/* Waits for the map to be unstable.
 * The stability of the map is checked every x ms.
 * Note that this stops the entire computation of the path every x ms.
 */
void Pathfinder::waitForMapUnstable()
{
    std::chrono::steady_clock::time_point currentTime1 = std::chrono::steady_clock::now();
    int i = 0;
    while(checkMapStable()) {
        if(++i > 50) {
            std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
            std::cout << "Map has been stable for: " << std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - currentTime1).count() << std::endl;
            i = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mapIsStable = false;
}


/* Waits for the map to be stable.
 * The stability of the map is checked every x ms.
 * Note that this stops the entire computation of the path every x ms.
 */
void Pathfinder::waitForMapStable()
{
    int i = 0;
    while(!checkMapStable()) {
        if(++i > 50) {
            std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
            std::cout << "Stabilzing map. Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeMeasureBegin).count() << std::endl;
            i = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    mapIsStable = true;
}


/* Checks if the map is stable.
 * Stops all computation of the map while checking.
 */
bool Pathfinder::checkMapStable()
{
    collectionControlMutex->lock();

    for(NodeCollection &collection : nodeCollections)
        collection.pause();

    for(NodeCollection &collection : nodeCollections) {
        while(!collection.getIsPaused()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool stable = true;
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            if(!map[i][j]->getStable()) {
                stable = false;
                break;
            }
            if(!stable)
                break;
        }
    }

    for(NodeCollection &collection : nodeCollections)
        collection.resume();

    collectionControlMutex->unlock();

    return stable;
}


/* Calculates and returns the neighbors of a given node.
 * Currently the neighbors are found in a square. Should be changed to hexagon. Look up paper in wiki for instructions.
 */
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


/* Divides the nodes into collections.
 * gridRows decides how many collections there should be pr. row. Same with gridCols.
 * If gridRows = 2 and gridCols = 5, 2*5 = 10 collections are made.
 * If each collection cannot have an equal amount of nodes, the 'overflow' nodes are added to the last collection(s).
 * Each collection runs in its own thread.
 */
void Pathfinder::divideIntoCollections()
{
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


/* Takes a slice of the map and adds it to the collection.
 * row,col: Top left position of the slice.
 * rowCount,colCount: Size of the slice.
 */
void Pathfinder::getSlice(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, NodeCollection &collection)
{
    std::vector<std::vector<std::shared_ptr<Node> > > temp;
    for(std::size_t i = 0; i < rowCount; i++)
        for(std::size_t j = 0; j < colCount; j++)
            collection.addNode(map[i+row][j+col]);
}


/* Prints the map using the positions of the nodes.
 */
void Pathfinder::printMap()
{
    for(std::size_t i = 0; i < map.size(); i++) {
        for(std::size_t j = 0; j < map[i].size(); j++) {
            std::cout << map[i][j]->getPosition().first << map[i][j]->getPosition().second << " ";
        }
        std::cout << "\n";
    }
}
