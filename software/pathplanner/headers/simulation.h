#ifndef SIMULATION_H
#define SIMULATION_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>

#include "headers/mapcontroller.h"

class Simulation
{
public:
    Simulation();
    ~Simulation();

private:
    void realSim();
    void mapSim();
    void droneSim();
    void runSingle();

private:
   Pathfinder pathfinder;

};

#endif // SIMULATION_H
