
#include "headers/simulation.h"


Simulation::Simulation()
{
    MapController test;
    test.generateTestMap();
    std::pair<std::size_t,std::size_t> currentPosition(0,0);
    std::pair<std::size_t,std::size_t> goalPosition(799,999);
    test.setCurrentPosition(currentPosition);
    test.setGoalPosition(goalPosition);
    test.startSolver();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "BEGIN!" << std::endl;
    while(true) {
        std::vector<std::pair<std::size_t, std::size_t> > path;
        test.getPathToDestination(path);
        if(path.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        currentPosition.first = path[path.size() - 2].first;
        currentPosition.second = path[path.size() - 2].second;
        std::cout << "New pos: " << currentPosition.first << " " << currentPosition.second << std::endl;
        test.setCurrentHeading(currentPosition);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        /*
        while(!test.getMapStable()) {
            test.printMapStatus();
            //test.printCostMap();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        test.printMapStatus();
        //test.printCostMap();
        //*/

        /*

        std::cout << "Update penalty!" << std::endl;

        std::vector<std::pair<std::size_t, std::size_t>> path;
        test.makePathToDestination(dest, dest, path);

        std::size_t pathIndex = path.size() / 2;
        currentPosition.first = path[pathIndex].first;
        currentPosition.second = path[pathIndex].second;
        test.setCurrentPosition(currentPosition);
        test.updatePenaltyOfNode(path[pathIndex - 1].first, path[pathIndex - 1].second, 5000);
        //test.printCostMap();


        while(!test.getMapStable()) {
            test.printMapStatus(dest, dest);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "DONE" << std::endl;
        test.printMapStatus(dest, dest);
        //test.printCostMap();
        //test.printPathMap(dest, dest);
        //*/
    }
    while(true)
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
}

Simulation::~Simulation()
{

}
