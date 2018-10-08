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
    void droneSim();
    void mapSim(bool *stop, bool *stopped);
    void utmSim(bool *stop, bool *stopped);

    void setupMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord, int nodeDistance, int width, int padLength);
    void setNewHeading(std::pair<double, double> newHeading);
    void setCurrentPosition(std::pair<double, double> newPosition);
    bool getPath(std::vector<std::pair<double, double> > &path);

    bool updatePenaltyAreaCircle();
    bool updatePenaltyAreaPolygon();
    void runSingle();

private:
   MapController *controller;

   bool flightPathCompromised = false;

   bool newHeadingSet = false;
   std::pair<double, double> currentHeading;

   bool newPositionSet = false;
   std::pair<double, double> currentPosition;

};

#endif // SIMULATION_H
