
#include "headers/simulation.h"


Simulation::Simulation()
{
    Pathfinder test;
    test.generateTestMap();
    std::pair<std::size_t,std::size_t> currentPosition(500,500);
    test.setCurrentPosition(currentPosition);
    test.startSolver();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while(true) {
        std::size_t dest = 1999;
        while(!test.getMapStable()) {
            test.printMapStatus(dest, dest);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        test.printMapStatus(dest, dest);
        //test.printCostMap();


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

        break;
    }
}

Simulation::~Simulation()
{

}
