
#include "headers/simulation.h"
#include "headers/pathshortener.h"
#include "headers/pathexporter.h"
#include "headers/global/geofunctions.h"


Simulation::Simulation()
{
    controller = new MapController;
    while(true)
        realSim();
    /*
    int i = 1;
    while(true) {
        srand(i++);
        runSingle();
    }

    while(true)
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    */
}

Simulation::~Simulation()
{

}

void Simulation::realSim()
{
    bool *stopUTMThread = new bool(false);
    bool *stoppedUTMThread = new bool(false);
    std::shared_ptr<std::thread> utmThread = std::shared_ptr<std::thread>(new std::thread(&Simulation::utmSim,this, stopUTMThread, stoppedUTMThread));
    utmThread->detach();

    std::shared_ptr<std::thread> mapThread = std::shared_ptr<std::thread>(new std::thread(&Simulation::mapSim,this));
    mapThread->detach();

    droneSim();

    *stopUTMThread = true;
    while(!*stoppedUTMThread)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    delete stopUTMThread;
    delete stoppedUTMThread;
}

void Simulation::droneSim()
{
    std::pair<double, double> startCoord(55.056010, 10.606016); // Svendborg
    std::pair<double, double> endCoord(55.385354, 10.368279); // Odense
    setupMap(startCoord, endCoord, 50, 10000, 5000);

    std::pair<double, double> currentPosition = startCoord;
    //setCurrentPosition(currentPosition);
    std::vector<std::pair<double, double> > path;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while(true) {
        if(GeoFunctions::calcMeterDistanceBetweensCoords(currentPosition, endCoord) < 100)
            break;

        bool succes = getPath(path);
        if(!succes || path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //std::cout << "EMPTY!" << std::endl;
            continue;
        }

        if(path.size() < 2)
            break;

        std::pair<double, double> newPosition;
        newPosition.first = path[path.size() - 2].first;
        newPosition.second = path[path.size() - 2].second;

        setNewHeading(newPosition);
        //controller->setCurrentHeading(newPosition);

        double distanceToMove = GeoFunctions::calcMeterDistanceBetweensCoords(currentPosition, newPosition);

        double timeToMove = distanceToMove/DRONE_MAX_SPEED;

        std::cout << "timeToMove: " << timeToMove << std::endl;

        std::pair<double,double> stepPrSecond;
        stepPrSecond.first = (currentPosition.first - newPosition.first) / timeToMove;
        stepPrSecond.second = (currentPosition.second - newPosition.second) / timeToMove;

        for(int i = 0; i < timeToMove; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            currentPosition.first -= stepPrSecond.first;
            currentPosition.second -= stepPrSecond.second;
            setCurrentPosition(currentPosition);
        }

        currentPosition = newPosition;
        setCurrentPosition(newPosition);
    }
    std::cout << "DONE" << std::endl;
}

void Simulation::mapSim()
{
    while(!controller->getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<std::pair<double, double> > path;

    while(true) {
        if(newHeadingSet) {
            controller->setCurrentHeading(currentHeading);
            newHeadingSet = false;
        }

        if(newPositionSet) {
            controller->setCurrentPosition(currentPosition);
            newPositionSet = false;
        }

        controller->getPathToDestination(path);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Simulation::utmSim(bool *stop, bool *stopped)
{
    while(!controller->getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::size_t mapSizeRow = controller->getMapSize().first;
    std::size_t mapSizeCol = controller->getMapSize().second;

    for(int i = 0; i < 3; i++) {
        rand();
        double radius = rand() % 1900 + 100;
        std::size_t randRow = rand() % mapSizeRow;
        std::size_t randCol = rand() % mapSizeCol;
        controller->updatePenaltyOfArea(controller->getWorldCoordAtIndex(randRow, randCol), radius, 10000);
    }

    while(!*stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double radius = rand() % 1900 + 100;
        std::size_t randRow = rand() % mapSizeRow;
        std::size_t randCol = rand() % mapSizeCol;

        controller->updatePenaltyOfArea(controller->getWorldCoordAtIndex(randRow, randCol), radius, 10000);
    }
    *stopped = true;
}

void Simulation::setupMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord, int nodeDistance, int width, int padLength)
{
    controller->generateMap(startCoord, endCoord, nodeDistance, width, padLength);

    controller->setGoalPosition(endCoord);
    controller->startSolver(startCoord);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void Simulation::setNewHeading(std::pair<double, double> newHeading)
{
    currentHeading = newHeading;
    newHeadingSet = true;
}

void Simulation::setCurrentPosition(std::pair<double, double> newPosition)
{
    currentPosition = newPosition;
    newPositionSet = true;
}

bool Simulation::getPath(std::vector<std::pair<double, double> > &path)
{
    return controller->getPathToDestination(path);
}

void Simulation::runSingle()
{
    std::cout << "NO" << std::endl;
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



/*
std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
iterationCounter++;
std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
std::cout << "Time2: " << timeCounter/iterationCounter << "  " << iterationCounter << std::endl;
*/
