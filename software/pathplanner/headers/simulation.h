<<<<<<< HEAD
﻿#ifndef SIMULATION_H
=======
#ifndef SIMULATION_H
>>>>>>> develop
#define SIMULATION_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>

#include "headers/mapcontroller.h"

<<<<<<< HEAD
struct Drone {
    Drone() {}
    Drone(std::string aDroneID, std::string aName, int operationStatus, int trackingEntry, int aGPSTimestamp) {
        this->droneID = aDroneID;
        this->name = aName;
        this->operationStatus = operationStatus;
        this->trackingEntry = trackingEntry;
        this->GPSTimestamp = aGPSTimestamp;
    }
    std::string droneID;
    std::string name;
    int operationStatus;
    int trackingEntry;
    int GPSTimestamp;
    std::pair<double,double> currentPos;
    std::pair<double,double> nextPos;
    bool init = false;

};

=======
>>>>>>> develop
class Simulation
{
public:
    Simulation();
    ~Simulation();

private:
<<<<<<< HEAD
    void realSim();
    void droneSim();
    void mapSim(bool *stop, bool *stopped);
    void utmSim(bool *stop, bool *stopped);

    void setupMap(std::pair<double, double> startCoord, std::pair<double, double> endCoord, int nodeDistance, int width, int padLength);
    void setNewHeading(std::pair<double, double> newHeading);
    void setCurrentPosition(std::pair<double, double> newPosition);
    bool getPath(std::vector<std::pair<double, double> > &path);

    bool updateDrone(int index);
    bool updatePenaltyAreaCircle();
    bool updatePenaltyAreaCircleDynamic();
    bool updatePenaltyAreaPolygon();
    bool updatePenaltyAreaPolygonDynamic();
    void runSingle();

private:
   MapController *controller;
   std::vector<Drone> drones;

   bool flightPathCompromised = false;

   bool newHeadingSet = false;
   std::pair<double, double> currentHeading;

   bool newPositionSet = false;
   std::pair<double, double> currentPosition;
=======
    void runSingle();

private:
   Pathfinder pathfinder;
>>>>>>> develop

};

#endif // SIMULATION_H
