
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{
    subStart();
}

void rosMsg::addNoFlightCircle(const PATHPLANNER::add_no_flight_circle &msg)
{
    std::pair<double,double> coord;
    coord.first = msg.lat;
    coord.second = msg.lon;

    if(solvingStarted)
        controller.updatePenaltyOfAreaCircle(coord, msg.radius, msg.penalty, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius, msg.penalty, msg.epochValidFrom, msg.epochValidTo);
}

void rosMsg::addNoFlightArea(const PATHPLANNER::add_no_flight_area &msg)
{
    std::vector<std::pair<double,double>> coords;

    for(size_t i = 0; i < msg.polygonCoordinates.size(); i += 2) {
        std::pair<double,double> coord(msg.polygonCoordinates[i], msg.polygonCoordinates[i+1]);
        coords.push_back(coord);
    }

    if(solvingStarted)
        controller.updatePenaltyOfAreaPolygon(coords, msg.penalty, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaPolygon(coords, msg.penalty, msg.epochValidFrom, msg.epochValidTo);
};

void rosMsg::setupMap(const PATHPLANNER::start_end_coord& msg)
{
    solvingStarted = true;

    startCoord.first = msg.coords[0];
    startCoord.second = msg.coords[1];

    endCoord.first = msg.coords[2];
    endCoord.second = msg.coords[3];

    nodeDist = msg.mapParams[0];
    mapWidth =  msg.mapParams[1];
    padLength = msg.mapParams[2];

    std::cout << "startCoord: " << startCoord.first << " " << startCoord.second << std::endl;
    std::cout << "endCoord: " << endCoord.first << " " << endCoord.second << std::endl;
    std::cout << "nodeDist: " << nodeDist << std::endl;
    std::cout << "mapWidth: " << mapWidth << std::endl;
    std::cout << "padLength: " << padLength << std::endl;


    controller.generateMap(startCoord, endCoord, nodeDist, mapWidth, padLength);
    controller.setGoalPosition(endCoord);
    controller.startSolver(startCoord);

    while(!controller.getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void rosMsg::requestPath(const PATHPLANNER::request_flight_mission &msg)
{
    pub  = n.advertise<PATHPLANNER::flight_mission>("flightMission", 1);

    bool succes = controller.getPathToDestination(path);

    std::vector<double> missions;

    if(succes) {
        for(auto it = path.rbegin(); it != path.rend(); it++) {
            missions.push_back(it->first);
            missions.push_back(it->second);
        }
    }


    PATHPLANNER::flight_mission newMsg;
    newMsg.mission_list = missions;

    pub.publish(newMsg);
};

void rosMsg::subStart()
{
    std::cout << "subStart" << std::endl;

    subMap = n.subscribe("mapParam", 1000, &rosMsg::setupMap,  this );

    subRequestPath = n.subscribe("requestPath", 1, &rosMsg::requestPath,  this );

    subNoFlightCircles = n.subscribe("noFlightCircles", 1000, &rosMsg::addNoFlightCircle, this);

    subNoFlightAreas = n.subscribe("noFlightAreas", 1000, &rosMsg::addNoFlightArea, this);
};



void rosMsg::Publish()
{
    pub  = n.advertise<std_msgs::Int64>("path to goal", 1);


    //pub.publish(map.getPathToDestination(path));


};

rosMsg::~rosMsg()
{

}
