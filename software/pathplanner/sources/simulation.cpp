
#include "headers/simulation.h"


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

void Simulation::runSingle()
{
    MapController test;
    test.generateTestMap();
    std::pair<std::size_t,std::size_t> currentPosition(rand() % 800, rand() % 1600);
    std::pair<std::size_t,std::size_t> goalPosition(rand() % 800, rand() % 1600);
    //std::pair<std::size_t,std::size_t> currentPosition(rand() % 400, rand() % 400);
    //std::pair<std::size_t,std::size_t> goalPosition(rand() % 400, rand() % 400);
    test.setGoalPosition(goalPosition);
    test.startSolver(currentPosition);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<std::size_t, std::size_t> > path;

    //std::cout << "BEGIN!" << std::endl;
    std::chrono::steady_clock::time_point beginTime = std::chrono::steady_clock::now();

    int i = 0;
    while(true) {
        i++;
        //std::cout << "Get current path" << std::endl;
        bool succes = test.getPathToDestination(path);
        if(!succes || path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cout << "EMPTY!" << std::endl;
            continue;
        }

        if(path.size() < 1)
            break;

        if(i % 1 == 0) {
            if(path.size() > 1) {
                currentPosition.first = path[path.size() - 1].first;
                currentPosition.second = path[path.size() - 1].second;
            }
            else
                break;
        }


        // std::cout << "Iteration: " << i << std::endl;

        if(rand() % 5000 == 0) {
            //currentPosition.first = rand() % 800;
            //currentPosition.second = rand() % 1600;
            //std::cout << "Choose random point" << std::endl;
        }
        test.setCurrentHeading(currentPosition);

        if(rand() % 5 == 0) {
            if(path.size() > 10)
                test.updatePenaltyOfNode(path[path.size() - 10].first, path[path.size() - 10].second, 500);
            //std::cout << "Update penalty" << std::endl;
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        //std::cout << "Iteration done" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::chrono::steady_clock::time_point finishTime = std::chrono::steady_clock::now();
    std::cout << "Time to complete: " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - beginTime).count() << std::endl;
}
