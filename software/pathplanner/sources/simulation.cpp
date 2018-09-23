
#include "headers/simulation.h"


Simulation::Simulation()
{
    /*
    MapController test;
    test.generateMap();

    while(true)
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        */

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

void Simulation::runSingle()
{
    MapController test;

    std::pair<double, double> startCoord(55.056010, 10.606016);
    std::pair<double, double> endCoord(55.385354, 10.368279);

    test.generateMap(startCoord, endCoord);

    std::size_t mapSizeRow = test.getMapSize().first;
    std::size_t mapSizeCol = test.getMapSize().second;

    //std::pair<std::size_t,std::size_t> currentPosition(rand() % mapSizeRow, rand() % mapSizeCol);
    //std::pair<std::size_t,std::size_t> goalPosition(rand() % mapSizeRow, rand() % mapSizeCol);
    test.setGoalPosition(endCoord);
    test.startSolver(startCoord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<double, double> > path;

    //std::cout << "BEGIN!" << std::endl;
    //std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();

    std::pair<double, double> currentPosition = startCoord;

    int i = 0;
    int timeCounter = 0;

    while(true) {

        std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();
        //std::cout << "Get current path" << std::endl;
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


        if(rand() % 5000 == 0) {
            //currentPosition.first = rand() % 800;
            //currentPosition.second = rand() % 1600;
            //std::cout << "Choose random point" << std::endl;
        }

        test.setCurrentHeading(currentPosition);

        std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
        timeCounter += std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count();
        //std::cout << "Time: " << timeCounter/i << "  " << i << std::endl;

        if(rand() % 1 == 0) {
            int randRow = rand() % mapSizeRow;
            int randCol = rand() % mapSizeCol;
            /*
            for(int i = 0; i < 100; i++)
                test.updatePenaltyOfNode(rand() % 300, rand() % 300, 500);
            */
            //*
            int minusRow = rand() % 20 + 5;
            int minusCol = rand() % 20 + 5;
            std::vector<std::pair<std::size_t, std::size_t> > positions;
            for(int i = randRow - minusRow; i < randRow; i++)
                for(int j = randCol - minusCol; j < randCol; j++)
                    if(i > 0 && j > 0)
                        positions.push_back(std::pair<std::size_t, std::size_t>(i,j));
            test.updatePenaltyOfNodeGroup(positions, 1000);
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            //*/
        }



        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "Iterations: " << i << std::endl;
    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    //std::cout << "Time to complete: " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count() << std::endl;
}
