
#include "headers/simulation.h"
<<<<<<< HEAD
#include "headers/pathshortener.h"
#include "headers/pathexporter.h"
#include "headers/global/geofunctions.h"
=======
>>>>>>> develop


Simulation::Simulation()
{
<<<<<<< HEAD
    for(int i = 0; i < 5; i++) {
        std::string name = "Drone";
        name.append(std::to_string(i));
        Drone drone(name, "Name", 1, 1, 1);
        drones.push_back(drone);
    }

    while(true) {
        controller = new MapController;
        realSim();
        delete  controller;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        controller = new MapController;
        flightPathCompromised = false;
        newHeadingSet = false;
        newPositionSet = false;

        for(auto &drone : drones)
            drone.init = false;
    }
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

    bool *stopMapThread = new bool(false);
    bool *stoppedMapThread = new bool(false);
    std::shared_ptr<std::thread> mapThread = std::shared_ptr<std::thread>(new std::thread(&Simulation::mapSim,this, stopMapThread, stoppedMapThread));
    mapThread->detach();

    droneSim();

    *stopUTMThread = true;
    *stopMapThread = true;

    while(!*stoppedUTMThread || !*stoppedMapThread)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    delete stopUTMThread;
    delete stoppedUTMThread;
    delete stopMapThread;
    delete stoppedMapThread;
}

void Simulation::droneSim()
{
    //std::pair<double, double> startCoord(55.056010, 10.606016); // Svendborg
    //std::pair<double, double> endCoord(55.385354, 10.368279); // Odense

    std::pair<double, double> startCoord(55.056010, 10.606016);
    std::pair<double, double> endCoord(55.185354, 10.468279);

    setupMap(startCoord, endCoord, 20, 5000, 500);

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

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //getPath(path);

        if(path.size() < 2)
            break;

        std::pair<double, double> newPosition;
        newPosition.first = path[path.size() - 2].first;
        newPosition.second = path[path.size() - 2].second;

        setNewHeading(newPosition);
        //controller->setCurrentHeading(newPosition);

        double distanceToMove = GeoFunctions::calcMeterDistanceBetweensCoords(currentPosition, newPosition);

        double timeToMove = distanceToMove/DRONE_MAX_SPEED;

        //std::cout << "timeToMove: " << timeToMove << std::endl;

        std::pair<double,double> stepPrSecond;
        stepPrSecond.first = (currentPosition.first - newPosition.first) / timeToMove;
        stepPrSecond.second = (currentPosition.second - newPosition.second) / timeToMove;

        std::time_t timeOverSlept = 0;
        for(int i = 0; i < timeToMove; i++) {
            std::time_t start = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 + timeOverSlept));
            timeOverSlept = 1000 - (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - start);
            currentPosition.first -= stepPrSecond.first;
            currentPosition.second -= stepPrSecond.second;
            if(flightPathCompromised)
                break;
            setCurrentPosition(currentPosition);
        }
        if(flightPathCompromised) {
            flightPathCompromised = false;
            setNewHeading(currentPosition);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        currentPosition = newPosition;
        setCurrentPosition(newPosition);
    }
    std::cout << "DONE" << std::endl;
}

void Simulation::mapSim(bool *stop, bool *stopped)
{
    while(!controller->getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::vector<std::pair<double, double> > path;

    while(!*stop) {
        if(newHeadingSet) {
            controller->setCurrentHeading(currentHeading);
            newHeadingSet = false;
        }

        if(newPositionSet) {
            controller->setCurrentPosition(currentPosition);
            newPositionSet = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    *stopped = true;
}

void Simulation::utmSim(bool *stop, bool *stopped)
{
    while(!controller->getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for(int i = 0; i < 0; i++) {
        if(rand() % 2)
            updatePenaltyAreaCircle();
        else
            updatePenaltyAreaPolygon();
    }

    while(!*stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        bool wasCompromised = flightPathCompromised;

        for(int i = 0; i < drones.size(); i++)
            flightPathCompromised = updateDrone(i) || flightPathCompromised;

        int randomNumber = 4; //rand() % 4;

        switch (randomNumber) {
        case 0:
            flightPathCompromised = updatePenaltyAreaCircle() || flightPathCompromised;
            break;
        case 1:
            flightPathCompromised = updatePenaltyAreaCircleDynamic() || flightPathCompromised;
            break;
        case 2:
            flightPathCompromised = updatePenaltyAreaPolygon() || flightPathCompromised;
            break;
        case 3:
            flightPathCompromised = updatePenaltyAreaPolygonDynamic() || flightPathCompromised;
            break;
        }

        if(wasCompromised)
            flightPathCompromised = true;
        if(flightPathCompromised)
            std::cout << "Compromised!" << std::endl;
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

bool Simulation::updateDrone(int index)
{
    std::size_t mapSizeRow = controller->getMapSize().first;
    std::size_t mapSizeCol = controller->getMapSize().second;

    std::size_t randRow = 0;//rand() % mapSizeRow;
    std::size_t randCol = 50 + rand() % (mapSizeCol - 100);

    int randRow2 = mapSizeRow - 1;//randRow + rand() % 5000 - 2500;
    int randCol2 = randCol + rand() % 5000 - 2500;

    if(randRow2 >= mapSizeRow)
        randRow2 = mapSizeRow - 1;

    if(randCol2 >= mapSizeCol)
        randCol2 = mapSizeCol - 1;

    if(randRow2 < 0)
        randRow2 = 0;

    if(randCol2 < 0)
        randCol2 = 0;

    Drone *drone = &drones[index];

    if(!drone->init) {
        drone->init = true;

        drone->currentPos = controller->getWorldCoordAtIndex(randRow, randCol);
        drone->nextPos = controller->getWorldCoordAtIndex(randRow2, randCol);
    }

    double distanceToMove = GeoFunctions::calcMeterDistanceBetweensCoords(drone->currentPos, drone->nextPos);

    double timeToMove = distanceToMove/100;

    std::pair<double,double> stepSize;
    stepSize.first = (drone->currentPos.first - drone->nextPos.first) / (timeToMove) * 2;
    stepSize.second = (drone->currentPos.second - drone->nextPos.second) / (timeToMove) * 2;

    drone->currentPos.first -= stepSize.first;
    drone->currentPos.second -= stepSize.second;

    if(GeoFunctions::calcMeterDistanceBetweensCoords(drone->currentPos, drone->nextPos) < 100)
        drone->nextPos = controller->getWorldCoordAtIndex(randRow, randCol);

    return controller->updateDrone(drone->droneID, drone->name, drone->operationStatus, drone->trackingEntry, drone->GPSTimestamp, drone->currentPos, drone->nextPos);
}

bool Simulation::updatePenaltyAreaCircle()
{
    std::size_t mapSizeRow = controller->getMapSize().first;
    std::size_t mapSizeCol = controller->getMapSize().second;

    double radius = rand() % 3900 + 100;
    std::size_t randRow = rand() % mapSizeRow;
    std::size_t randCol = rand() % mapSizeCol;

    return controller->updatePenaltyOfAreaCircle(controller->getWorldCoordAtIndex(randRow, randCol), radius, 10000);
}

bool Simulation::updatePenaltyAreaCircleDynamic()
{
    std::size_t mapSizeRow = controller->getMapSize().first;
    std::size_t mapSizeCol = controller->getMapSize().second;

    double radius = rand() % 3900 + 100;
    std::size_t randRow = mapSizeRow/2; //rand() % mapSizeRow;
    std::size_t randCol = rand() % mapSizeCol;

    std::time_t from = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    from += 5;
    std::time_t to = from + 15;

    return controller->updatePenaltyOfAreaCircle(controller->getWorldCoordAtIndex(randRow, randCol), radius, 10000, from, to);
}

bool Simulation::updatePenaltyAreaPolygon()
{
    size_t mapSizeRow = controller->getMapSize().first;
    size_t mapSizeCol = controller->getMapSize().second;

    size_t x = rand() % mapSizeRow;
    size_t y = rand() % mapSizeCol;

    std::vector<std::pair<double,double>> polygonCoordinates;

    polygonCoordinates.push_back(controller->getWorldCoordAtIndex(x, y));

    for(int i = 0; i < 6; i++) {
        x += rand() % 100 - 50;
        y += rand() % 100 - 50;
        if(x > 200)
            x = 200;
        if(y > 800)
            y = 800;

        if(x < 0)
            x = 0;
        if(y < 0)
            y = 0;
        polygonCoordinates.push_back(controller->getWorldCoordAtIndex(x, y));
    }

    polygonCoordinates.push_back(polygonCoordinates.front());

    return controller->updatePenaltyOfAreaPolygon(polygonCoordinates, 10000);
}

bool Simulation::updatePenaltyAreaPolygonDynamic()
{
    size_t mapSizeRow = controller->getMapSize().first;
    size_t mapSizeCol = controller->getMapSize().second;

    size_t x = rand() % mapSizeRow;
    size_t y = rand() % mapSizeCol;

    std::vector<std::pair<double,double>> polygonCoordinates;

    polygonCoordinates.push_back(controller->getWorldCoordAtIndex(x, y));

    for(int i = 0; i < 6; i++) {
        x += rand() % 100 - 50;
        y += rand() % 100 - 50;
        if(x > 200)
            x = 200;
        if(y > 800)
            y = 800;

        if(x < 0)
            x = 0;
        if(y < 0)
            y = 0;
        polygonCoordinates.push_back(controller->getWorldCoordAtIndex(x, y));
    }

    polygonCoordinates.push_back(polygonCoordinates.front());

    std::time_t from = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    from += 5;
    std::time_t to = from + 15;

    return controller->updatePenaltyOfAreaPolygon(polygonCoordinates, 10000, from, to);
=======
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

>>>>>>> develop
}

void Simulation::runSingle()
{
<<<<<<< HEAD
    std::cout << "NO" << std::endl;
    MapController test;

    std::pair<double, double> startCoord(55.056010, 10.606016); // Svendborg
    std::pair<double, double> endCoord(55.385354, 10.368279); // Odense

    //std::pair<double, double> startCoord(55.471953, 10.41367);
    //std::pair<double, double> endCoord(55.472070, 10.417255);

    test.generateMap(startCoord, endCoord, 50, 10000, 5000);
=======
    MapController test;

    std::pair<double, double> startCoord(55.056010, 10.606016);
    std::pair<double, double> endCoord(55.385354, 10.368279);

    test.generateMap(startCoord, endCoord);
>>>>>>> develop

    std::size_t mapSizeRow = test.getMapSize().first;
    std::size_t mapSizeCol = test.getMapSize().second;

    test.setGoalPosition(endCoord);
    test.startSolver(startCoord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<double, double> > path;
<<<<<<< HEAD
    std::vector<std::pair<double, double> > actualPath;

    std::pair<double, double> currentPosition = startCoord;
    actualPath.push_back(currentPosition);

    int i = 0;

    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
    while(true) {

=======

    std::pair<double, double> currentPosition = startCoord;

    int i = 0;
    int timeCounter = 0;

    while(true) {

        std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();

>>>>>>> develop
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

<<<<<<< HEAD
        actualPath.push_back(currentPosition);

        test.setCurrentHeading(currentPosition);

        if(rand() % 5 == 0) {
            double radius = rand() % 1900 + 100;
            std::size_t randRow = rand() % mapSizeRow;
            std::size_t randCol = rand() % mapSizeCol;

            test.updatePenaltyOfAreaCircle(test.getWorldCoordAtIndex(randRow, randCol), radius, 10000);
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
=======
        test.setCurrentHeading(currentPosition);

        std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
        timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
        std::cout << "Time: " << timeCounter/i << "  " << i << std::endl;

        if(rand() % 1 == 0) {
            int randRow = rand() % mapSizeRow;
            int randCol = rand() % mapSizeCol;
            int minusRow = rand() % 20 + 5;
            int minusCol = rand() % 20 + 5;

            std::vector<std::pair<std::size_t, std::size_t> > positions;
            for(int i = randRow - minusRow; i < randRow; i++)
                for(int j = randCol - minusCol; j < randCol; j++)
                    if(i > 0 && j > 0)
                        positions.push_back(std::pair<std::size_t, std::size_t>(i,j));

            test.updatePenaltyOfNodeGroup(positions, 1000);
        }



        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "Iterations: " << i << std::endl;
    //std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    //std::cout << "Time to complete: " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count() << std::endl;
}
>>>>>>> develop
