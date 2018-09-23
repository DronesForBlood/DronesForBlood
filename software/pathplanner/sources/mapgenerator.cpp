
#include "headers/mapgenerator.h"

MapGenerator::MapGenerator()
{

}

void MapGenerator::generateMap(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node> > > > map, std::vector<NodeCollection> &nodeCollections, std::pair<double, double> startCoord, std::pair<double, double> endCoord, double distanceBetweenNodes, double width, double padLength)
{
    map->clear();
    nodeCollections.clear();

    double length = calcMeterDistanceBetweensCoords(startCoord, endCoord);

    std::size_t mapSizeX = std::size_t(width/distanceBetweenNodes);
    std::size_t mapSizeY = std::size_t((length + 2*padLength)/distanceBetweenNodes);

    std::pair<double,double> normalToLine = calcNormalVector(startCoord, endCoord);
    std::pair<double,double> shifted = calcShiftedCoord(startCoord, normalToLine.second, normalToLine.first);
    std::pair<double,double> normalToUse = calcNormalVector(shifted, startCoord);

    startCoord = calcShiftedCoord(startCoord, normalToUse.second * -padLength, normalToUse.first * -padLength);

    for(std::size_t i = 0; i < mapSizeX + 1; i++) {
        double posOnWidth = width/mapSizeX * i - width/2.;
        std::pair<double,double> shiftedCoord = calcShiftedCoord(startCoord, normalToLine.second * posOnWidth, normalToLine.first * posOnWidth);
        std::vector<std::shared_ptr<Node>> row;

        for(std::size_t j = 0; j < mapSizeY + 1; j++) {
            std::pair<double,double> coord = calcShiftedCoord(shiftedCoord, normalToUse.second * distanceBetweenNodes*j, normalToUse.first * distanceBetweenNodes*j);
            //std::cout << coord.first << ", " << coord.second << std::endl;

            std::pair<std::size_t, std::size_t> index(i,j);
            std::shared_ptr<Node> newNode = std::make_shared<Node>(index, coord);
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

    std::cout << "Map size: " << map->size() << " x " << map->at(0).size() << std::endl;
}

void MapGenerator::divideIntoCollections(std::shared_ptr<std::vector<std::vector<std::shared_ptr<Node> > > > map, std::vector<NodeCollection> &nodeCollections)
{
    for(std::size_t i = 0; i < map->size(); i += nodesPrCollectionAxis) {
        for(std::size_t j = 0; j < map->at(i).size(); j += nodesPrCollectionAxis) {
            NodeCollection temp;

            std::size_t rows = nodesPrCollectionAxis;
            std::size_t cols = nodesPrCollectionAxis;

            if(i + rows > map->size())
                rows = map->size() - i;

            if(j + cols > map->at(i).size())
                cols = map->at(i).size() - j;

            getSlice(i, j, rows, cols, map, temp);
            nodeCollections.push_back(temp);
        }
    }
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

double MapGenerator::calcMeterDistanceBetweensCoords(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    double lat1 = startCoord.first;
    double lon1 = startCoord.second;
    double lat2 = endCoord.first;
    double lon2 = endCoord.second;

    lat1 = lat1 * PI / 180.;
    lat2 = lat2 * PI / 180.;
    lon1 = lon1 * PI / 180.;
    lon2 = lon2 * PI / 180.;

    double distance_radians = acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2));
    double distanceMeters = distance_radians * RADIUS_EARTH_METERS;
    return distanceMeters;
}

std::pair<double, double> MapGenerator::calcShiftedCoord(std::pair<double, double> coord, double dxMeters, double dyMeters)
{
    double newLat = coord.first  + (dyMeters / RADIUS_EARTH_METERS) * (180. / PI);
    double newLon = coord.second + (dxMeters / RADIUS_EARTH_METERS) * (180. / PI) / cos(coord.first * PI/180.);

    std::pair<double,double> newCoord(newLat, newLon);
    return newCoord;
}

std::pair<double, double> MapGenerator::calcNormalVector(std::pair<double, double> startCoord, std::pair<double, double> endCoord)
{
    double dx = endCoord.first - startCoord.first;
    double dy = endCoord.second - startCoord.second;
    std::pair<double,double> normalToLine(-dy, dx);

    double magnitude = sqrt(pow(normalToLine.first, 2) + pow(normalToLine.second, 2));
    normalToLine.first = normalToLine.first / magnitude;
    normalToLine.second = normalToLine.second / magnitude;

    return normalToLine;
}
