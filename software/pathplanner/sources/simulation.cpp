
#include "headers/simulation.h"


Simulation::Simulation()
{
    Pathfinder test;
    test.generateTestMap();
    std::pair<std::size_t,std::size_t> currentPosition(0,0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    test.setCurrentPosition(currentPosition);
    test.startSolver();

    while(true) {
        while(!test.getMapStable()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //test.printMapStatus(0, 19);
            //test.printPathMap(0, 19);
        }
        test.printMapStatus(0, 19);

        std::cout << "Update penalty!" << std::endl;
        currentPosition.first = 15;
        currentPosition.second = 15;
        test.setCurrentPosition(currentPosition);
        test.updatePenaltyOfNode(3, 3, 0);
        //test.printMapStatus(0, 100);


        while(!test.getMapStable()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            //test.printMapStatus(0, 19);
            //test.printPathMap(0, 19);
        }
        test.printPathMap(0, 19);
        test.printMapStatus(0, 19);
        break;
    }
}

Simulation::~Simulation()
{

}
