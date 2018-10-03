
#include "headers/simulation.h"
#include "headers/pathshortener.h"
#include "headers/pathexporter.h"


Simulation::Simulation()
{
    int i = 1;
    while(true) {
        srand(i++);
        runSingle();
    }

    while(true)
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
}

Simulation::~Simulation()
{

}

void Simulation::realSim()
{
    MapController test;

    std::pair<double, double> startCoord(55.056010, 10.606016); // Svendborg
    std::pair<double, double> endCoord(55.385354, 10.368279); // Odense

    //std::pair<double, double> startCoord(55.471953, 10.41367);
    //std::pair<double, double> endCoord(55.472070, 10.417255);

    test.generateMap(startCoord, endCoord, 50, 10000, 5000);

    std::size_t mapSizeRow = test.getMapSize().first;
    std::size_t mapSizeCol = test.getMapSize().second;

    test.setGoalPosition(endCoord);
    test.startSolver(startCoord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<double, double> > path;

    std::pair<double, double> currentPosition = startCoord;

    int i = 0;

    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    while(true) {

        bool succes = test.getPathToDestination(path);
        if(!succes || path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //std::cout << "EMPTY!" << std::endl;
            continue;
        }
        i++;

        if(path.size() < 2)
            break;

        currentPosition.first = path[path.size() - 1].first;
        currentPosition.second = path[path.size() - 1].second;

        test.setCurrentHeading(currentPosition);

        if(rand() % 5 == 0) {
            double radius = rand() % 1900 + 100;
            std::size_t randRow = rand() % mapSizeRow;
            std::size_t randCol = rand() % mapSizeCol;

            test.updatePenaltyOfArea(test.getWorldCoordAtIndex(randRow, randCol), radius, 10000);
        }
    }

    PathShortener shortener;
}

void Simulation::runSingle()
{
    MapController test;

    std::pair<double, double> startCoord(55.056010, 10.606016); // Svendborg
    std::pair<double, double> endCoord(55.385354, 10.368279); // Odense

    //std::pair<double, double> startCoord(55.471953, 10.41367);
    //std::pair<double, double> endCoord(55.472070, 10.417255);

    test.generateMap(startCoord, endCoord, 50, 10000, 5000);

    std::size_t mapSizeRow = test.getMapSize().first;
    std::size_t mapSizeCol = test.getMapSize().second;

    test.setGoalPosition(endCoord);
    test.startSolver(startCoord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<double, double> > path;
    std::vector<std::pair<double, double> > actualPath;

    std::pair<double, double> currentPosition = startCoord;
    actualPath.push_back(currentPosition);

    int i = 0;

    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    while(true) {

        bool succes = test.getPathToDestination(path);
        if(!succes || path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //std::cout << "EMPTY!" << std::endl;
            continue;
        }
        i++;

        if(path.size() < 2)
            break;

        currentPosition.first = path[path.size() - 1].first;
        currentPosition.second = path[path.size() - 1].second;

        actualPath.push_back(currentPosition);

        test.setCurrentHeading(currentPosition);

        if(rand() % 5 == 0) {
            double radius = rand() % 1900 + 100;
            std::size_t randRow = rand() % mapSizeRow;
            std::size_t randCol = rand() % mapSizeCol;

            test.updatePenaltyOfArea(test.getWorldCoordAtIndex(randRow, randCol), radius, 10000);
        }
    }

    actualPath.push_back(path[0]);
    actualPath.push_back(endCoord);

    PathShortener shortener;
    PathExporter exporter;

    for(auto it : actualPath)
        std::cout << std::setprecision(10) << it.first << " " << it.second << std::endl;

    std::vector<std::pair<double, double> > shortPath;
    shortener.shortenPath(actualPath, shortPath, 100);
    exporter.exportToKml(shortPath, "TEST_PATH", "idk", "idk");

    std::cout << "Iterations: " << i << std::endl;


    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    std::cout << "Time to complete: " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count() << std::endl;
}
