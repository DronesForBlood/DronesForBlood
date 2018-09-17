
#include "headers/simulation.h"


Simulation::Simulation()
{
    while(true) {
        runSingle();
        std::cout << "DONE" << std::endl;
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
    //std::pair<std::size_t,std::size_t> currentPosition(0, 0);
    //std::pair<std::size_t,std::size_t> goalPosition(10, 10);
    std::pair<std::size_t,std::size_t> currentPosition(rand() % 800, rand() % 1600);
    std::pair<std::size_t,std::size_t> goalPosition(rand() % 800, rand() % 1600);
    //test.setCurrentHeading(currentPosition);
    test.setGoalPosition(goalPosition);
    test.startSolver(currentPosition);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::vector<std::pair<std::size_t, std::size_t> > path;

    std::cout << "BEGIN!" << std::endl;
    int i = 0;
    while(true) {
        bool succes = test.getPathToDestination(path);
        if(!succes || path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        if(path.size() > 2) {
            currentPosition.first = path[path.size() - 1].first;
            currentPosition.second = path[path.size() - 1].second;
        }
        else
            return;


        i++;

        if(rand() % 1000 == 0) {
            currentPosition.first = rand() % 800;
            currentPosition.second = rand() % 1600;
        }
        test.setCurrentHeading(currentPosition);

        if(rand() % 50 == 0) {
            test.updatePenaltyOfNode(path[path.size() - 10].first, path[path.size() - 10].second, 500);
            //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    }
}
