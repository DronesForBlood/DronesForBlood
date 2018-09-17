
#include "headers/mapgenerator.h"

MapGenerator::MapGenerator()
{

}

/* Generates a simple test map of size mapSize x mapSize (see header).
 * One node is generated for each field on the map.
 * All the nodes are then given a pointer to their neighboring nodes.
 * The nodes are then split into square collections.
 */
void MapGenerator::generateTestMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections)
{
    //srand(time(0)); // Set a seed for the random generation of distances in the map. Used in nodes.h
    map->clear();
    nodeCollections.clear();

    // Generate a simple map of nodes.
    for(int i = 0; i < mapSizeRow; i++) {
        std::vector<std::shared_ptr<Node>> row;
        for(int j = 0; j < mapSizeCol; j++) {
            std::pair<double, double> pos(i,j);
            std::shared_ptr<Node> newNode = std::make_shared<Node>(pos);
            newNode->setPointerToSelf(newNode);
            row.push_back(newNode);
        }
        map->push_back(row);
    }

    // Give all nodes a pointer to their neighbors.
    for(std::size_t i = 0; i < map->size(); i++) {
        for(std::size_t j = 0; j < map->at(i).size(); j++) {
            std::vector<std::shared_ptr<Node>> neighborNodes;
            calculateNeighbors(i, j, neighborNodes, map);
            std::shared_ptr<Node> currentNode = map->at(i).at(j);
            currentNode->setNeighbors(neighborNodes);
        }
    }

    // Divide the nodes into square collections.
    divideIntoCollections(map, nodeCollections);
}

/* Calculates and returns the neighbors of a given node.
 * Currently the neighbors are found in a square. Should be changed to hexagon. Look up paper in wiki for instructions.
 */
void MapGenerator::calculateNeighbors(std::size_t row, std::size_t col, std::vector<std::shared_ptr<Node> > &neighborNodes, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map)
{
    // Top line
    if(row > 0 && col > 0)
        neighborNodes.push_back(map->at(row - 1).at(col - 1));

    if(row > 0)
        neighborNodes.push_back(map->at(row - 1).at(col));

    if(row > 0 && col < map->at(row).size() - 1)
        neighborNodes.push_back(map->at(row - 1).at(col + 1));


    // Middle line
    if(col > 0)
        neighborNodes.push_back(map->at(row).at(col - 1));

    if(col < map->at(row).size() - 1)
        neighborNodes.push_back(map->at(row).at(col + 1));


    // Bottom line
    if(row < map->size() - 1 && col > 0)
        neighborNodes.push_back(map->at(row + 1).at(col - 1));

    if(row < map->size() - 1)
        neighborNodes.push_back(map->at(row + 1).at(col));

    if(row < map->size() - 1 && col < map->at(row).size()  - 1)
        neighborNodes.push_back(map->at(row + 1).at(col + 1));
}

/* Divides the nodes into collections.
 * gridRows decides how many collections there should be pr. row. Same with gridCols.
 * If gridRows = 2 and gridCols = 5, 2*5 = 10 collections are made.
 * If each collection cannot have an equal amount of nodes, the 'overflow' nodes are added to the last collection(s).
 * Each collection runs in its own thread.
 */
void MapGenerator::divideIntoCollections(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, std::vector<NodeCollection> &nodeCollections)
{
    std::size_t rowCount = map->size() / gridRows;
    std::size_t colCount = map->at(0).size() / gridCols;

    for(std::size_t i = 0; i < gridRows; i++) {
        for(std::size_t j = 0; j < gridCols; j++) {
        NodeCollection temp;

        std::size_t thisRowCount = rowCount;
        if((i+1) % gridRows == 0)
            thisRowCount += map->size() % gridRows;

        std::size_t thisColCount = colCount;
        if((j+1) % gridCols == 0)
            thisColCount += map->at(0).size() % gridCols;

        getSlice(i*rowCount, j*colCount, thisRowCount, thisColCount, map, temp);
        nodeCollections.push_back(temp);
        }
    }
}

/* Takes a slice of the map and adds it to the collection.
 * row,col: Top left position of the slice.
 * rowCount,colCount: Size of the slice.
 */
void MapGenerator::getSlice(std::size_t row, std::size_t col, std::size_t rowCount, std::size_t colCount, std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node>>>> map, NodeCollection &collection)
{
    std::vector<std::vector<std::shared_ptr<Node> > > temp;
    for(std::size_t i = 0; i < rowCount; i++)
        for(std::size_t j = 0; j < colCount; j++)
            collection.addNode(map->at(i+row).at(j+col));
}
