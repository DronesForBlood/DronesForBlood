
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{
    subStart();
}

rosMsg::~rosMsg()
{

}

void rosMsg::addNoFlightCircle(const PATHPLANNER::no_flight_circle &msg)
{
    std::pair<double,double> coord;
    coord.first = msg.lat;
    coord.second = msg.lon;

    if(solvingStarted)
        controller.updatePenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);
}

void rosMsg::addNoFlightArea(const PATHPLANNER::no_flight_area &msg)
{
    std::vector<std::pair<double,double>> coords;

    for(size_t i = 0; i < msg.polygonCoordinates.size(); i += 2) {
        std::pair<double,double> coord(msg.polygonCoordinates[i], msg.polygonCoordinates[i+1]);
        coords.push_back(coord);
    }

    if(solvingStarted)
        controller.updatePenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);
}

void rosMsg::setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    currentCoord.first = msg.lat;
    currentCoord.second = msg.lon;

    if(solvingStarted)
        controller.setCurrentPosition(currentCoord);

    if(!solvingStarted && goalCoordSet)
        generateNewMap();

    currentCoordSet = true;
}

void rosMsg::setGoalPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    goalCoord.first = msg.lat;
    goalCoord.second = msg.lon;

    if(currentCoordSet)
        generateNewMap();

    goalCoordSet = true;
}

void rosMsg::calculatePath(const std_msgs::Bool &msg)
{
    mavlink_lora::mavlink_lora_mission_list newMsg;

    bool succes = controller.getPathToDestination(path);

    if(!succes) {
        for(int i = 0; i < 3; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            succes = controller.getPathToDestination(path);
            if(succes)
                break;
        }
    }


    if(succes) {
        unsigned short i = 0;
        for(auto it = path.rbegin(); it != path.rend(); it++) {
            mavlink_lora::mavlink_lora_mission_item_int item;
            item.target_system = 0;
            item.target_component = 0;
            item.seq = i;
            item.frame = 6;     // global pos, relative alt_int
            item.command = 16;
            item.param1 = 0;    // hold time
            item.param2 = 5;    // acceptance radius in mmsg = mavlink_lora.msg.mavlink_lora_mission_list()
            item.param3 = 0;    // pass though waypoint, no trajectory control

            std::cout << "Sending x: " << it->first << std::endl;
            std::cout << "Sending y: " << it->second << std::endl;
            std::cout << "Sending X: " << int32_t(it->first * 1e7) << std::endl;
            std::cout << "Sending Y: " << int32_t(it->second * 1e7) << std::endl;

            item.x = int32_t(it->first * 1e7);
            item.y = int32_t(it->second * 1e7);
            item.z = altitude;
            item.autocontinue = 1;

            newMsg.waypoints.push_back(item);
            i++;
        }
    }

    pubPath.publish(newMsg);
}

void rosMsg::generateNewMap()
{
    solvingStarted = true;

    nodeDist = 10;
    mapWidth =  500;
    padLength = 250;

    std::cout << "startCoord: " << currentCoord.first << " " << currentCoord.second << std::endl;
    std::cout << "endCoord: " << goalCoord.first << " " << goalCoord.second << std::endl;
    std::cout << "nodeDist: " << nodeDist << std::endl;
    std::cout << "mapWidth: " << mapWidth << std::endl;
    std::cout << "padLength: " << padLength << std::endl;

    controller.generateMap(currentCoord, goalCoord, nodeDist, mapWidth, padLength);
    controller.setGoalPosition(goalCoord);
    controller.startSolver(currentCoord);

    while(!controller.getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
};

void rosMsg::subStart()
{
    std::cout << "subStart" << std::endl;

    subCurrentPosition = n.subscribe("mavlink_pos", 1, &rosMsg::setCurrentPosition, this );
    subGoalPosition = n.subscribe("dronelink/destination", 1, &rosMsg::setGoalPosition, this );
    subCalculatePath = n.subscribe("gcs_master/calculate_path", 1, &rosMsg::calculatePath, this );

    subNoFlightCircles = n.subscribe("noFlightCircles", 1000, &rosMsg::addNoFlightCircle, this);

    subNoFlightAreas = n.subscribe("noFlightAreas", 1000, &rosMsg::addNoFlightArea, this);

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
};
