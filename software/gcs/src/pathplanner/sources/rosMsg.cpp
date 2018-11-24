
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{
    subStart();
}

rosMsg::~rosMsg()
{

}

void rosMsg::addNoFlightCircle(const utm::utm_no_flight_circle &msg)
{
    std::cout << "addNoFlightCircle at " << msg.lat << ", " << msg.lon << std::endl;

    /*
    NoFlightCircle zone;
    zone.id = int(msg.id);
    zone.name = msg.name;
    zone.coord.first = msg.lat;
    zone.coord.second = msg.lon;
    zone.epochValidFrom = int(msg.epochValidFrom);
    zone.epochValidTo = int(msg.epochValidTo);

    circleZones.push_back(zone);



    return;
    */

    std::pair<double,double> coord;
    coord.first = msg.lat;
    coord.second = msg.lon;

    if(solvingStarted)
        controller.updatePenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);

}

void rosMsg::addNoFlightArea(const utm::utm_no_flight_area &msg)
{
    /*
    std::cout << "addNoFlightArea" << std::endl;

    NoFlightArea zone;
    zone.id = int(msg.id);
    zone.name = msg.name;
    zone.epochValidFrom = int(msg.epochValidFrom);
    zone.epochValidTo = int(msg.epochValidTo);

    for(size_t i = 0; i < msg.polygonCoordinates.size(); i += 2) {
        std::pair<double,double> coord(msg.polygonCoordinates[i], msg.polygonCoordinates[i+1]);
        zone.coordinates.push_back(coord);
    }

    areaZones.push_back(zone);

    return;
    */

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
    std::cout << "setCurrentPosition" << std::endl;

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
    std::cout << "setGoalPosition" << std::endl;

    goalCoord.first = msg.lat;
    goalCoord.second = msg.lon;

    if(currentCoordSet)
        generateNewMap();

    goalCoordSet = true;
}

void rosMsg::calculatePath(const std_msgs::Bool &msg)
{
    std::cout << "calculatePath" << std::endl;

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
    std::cout << "generateNewMap" << std::endl;

    nodeDist = 2;
    mapWidth =  100;
    padLength = 50;

    std::cout << "startCoord: " << currentCoord.first << " " << currentCoord.second << std::endl;
    std::cout << "endCoord: " << goalCoord.first << " " << goalCoord.second << std::endl;
    std::cout << "nodeDist: " << nodeDist << std::endl;
    std::cout << "mapWidth: " << mapWidth << std::endl;
    std::cout << "padLength: " << padLength << std::endl;

    controller.generateMap(currentCoord, goalCoord, nodeDist, mapWidth, padLength);
    controller.setGoalPosition(goalCoord);

    //for(auto &it : circleZones)
    //    controller.addPreMapPenaltyOfAreaCircle(it.coord, it.radius, 10000, it.epochValidFrom, it.epochValidTo);

    //for(auto &it : areaZones)
    //    controller.addPreMapPenaltyOfAreaPolygon(it.coordinates, 10000, it.epochValidFrom, it.epochValidTo);

    solvingStarted = true;

    std::cout << "generateNewMap start" << std::endl;
    controller.startSolver(currentCoord);
    std::cout << "generateNewMap start done" << std::endl;

    while(!controller.getMapReady())
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "generateNewMap ready" << std::endl;
};

void rosMsg::subStart()
{
    std::cout << "Subscribe and publish begin" << std::endl;

    subCurrentPosition = n.subscribe("mavlink_pos", 1, &rosMsg::setCurrentPosition, this );
    subGoalPosition = n.subscribe("dronelink/destination", 1, &rosMsg::setGoalPosition, this );
    subCalculatePath = n.subscribe("gcs_master/calculate_path", 1, &rosMsg::calculatePath, this );

    subNoFlightCircles = n.subscribe("utm/fetch_no_flight_circles", 1000, &rosMsg::addNoFlightCircle, this);
    subNoFlightAreas = n.subscribe("utm/fetch_no_flight_areas", 1000, &rosMsg::addNoFlightArea, this);

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
    pubFetchNoFlightZones = n.advertise<std_msgs::Bool>("utm/request_no_flight_zones", 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std_msgs::Bool msg;
    pubFetchNoFlightZones.publish(msg);

    std::cout << "Subscribe and publish completed" << std::endl;
};
