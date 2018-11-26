
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{
    subStart();
}

rosMsg::~rosMsg()
{

}

void rosMsg::isReady(const std_msgs::Bool &msg)
{
    bool ready = numberOfZonesReceived >= numberOfExpectedZones;
    initialZonesLoaded = ready;

    std::cout << "Ready: " << ready << std::endl;
    //std::cout << "numberOfZonesReceived: " << numberOfZonesReceived << std::endl;
    //std::cout << "numberOfExpectedZones: " << numberOfExpectedZones << std::endl;

    std_msgs::Bool newMsg;
    newMsg.data = ready;
    pubIsReady.publish(newMsg);
}

bool rosMsg::getIsReady()
{
    return initialZonesLoaded;
}

void rosMsg::setNumberOfExpectedZones(const std_msgs::Int64 &msg)
{
    if(!initialZonesLoaded)
        numberOfExpectedZones = int(msg.data);
}

void rosMsg::addNoFlightCircle(const utm::utm_no_flight_circle &msg)
{
    //std::cout << "Got no flight circle" << std::endl;
    if(!initialZonesLoaded) {
        incrementZonesMutex.lock();
        numberOfZonesReceived++;
        incrementZonesMutex.unlock();
    }

    std::pair<double,double> coord;
    coord.first = msg.lat;
    coord.second = msg.lon;

    bool dynamicZone = msg.epochValidFrom >= 0;
    if(dynamicZone) {
        int id = int(msg.id);
        bool idExists = checkIfIDExists(id);
        if(idExists)
            return;
        else
            dynamicIDs.push_back(id);
    }

    if(solvingStarted)
        controller.updatePenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);

}

void rosMsg::addNoFlightArea(const utm::utm_no_flight_area &msg)
{
    //std::cout << "Got no flight area" << std::endl;

    if(!initialZonesLoaded) {
        incrementZonesMutex.lock();
        numberOfZonesReceived++;
        incrementZonesMutex.unlock();
    }

    bool dynamicZone = msg.epochValidFrom >= 0;
    if(dynamicZone) {
        int id = int(msg.id);
        bool idExists = checkIfIDExists(id);
        if(idExists)
            return;
        else
            dynamicIDs.push_back(id);
    }

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
    if(initialZonesLoaded) {
        std::cout << "setCurrentPosition" << std::endl;

        currentCoord.first = msg.lat;
        currentCoord.second = msg.lon;

        if(solvingStarted)
            controller.setCurrentPosition(currentCoord);

        if(!solvingStarted && goalCoordSet)
            generateNewMap();

        currentCoordSet = true;
    }

}

void rosMsg::setGoalPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    if(initialZonesLoaded) {
        std::cout << "setGoalPosition" << std::endl;

        goalCoord.first = msg.lat;
        goalCoord.second = msg.lon;

        if(currentCoordSet)
            generateNewMap();

        goalCoordSet = true;
    }

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

            /*
            std::cout << "Sending x: " << it->first << std::endl;
            std::cout << "Sending y: " << it->second << std::endl;
            std::cout << "Sending X: " << int32_t(it->first * 1e7) << std::endl;
            std::cout << "Sending Y: " << int32_t(it->second * 1e7) << std::endl;
            */

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
    while(numberOfZonesReceived < numberOfExpectedZones)
        return;

    std::cout << "generateNewMap" << std::endl;

    nodeDist = 1;
    mapWidth =  200;
    padLength = 100;

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
};

void rosMsg::subStart()
{
    std::cout << "Subscribe and publish begin" << std::endl;

    numberOfExpectedZones = INT_MAX;
    numberOfZonesReceived = 0;

    subCurrentPosition = n.subscribe("mavlink_pos", 1, &rosMsg::setCurrentPosition, this );
    subGoalPosition = n.subscribe("dronelink/destination", 1, &rosMsg::setGoalPosition, this );
    subCalculatePath = n.subscribe("gcs_master/calculate_path", 1, &rosMsg::calculatePath, this );

    subNumberOfZones = n.subscribe("utm/number_of_zones", 1, &rosMsg::setNumberOfExpectedZones, this);
    subNoFlightCircles = n.subscribe("utm/fetch_no_flight_circles", 1000, &rosMsg::addNoFlightCircle, this);
    subNoFlightAreas = n.subscribe("utm/fetch_no_flight_areas", 1000, &rosMsg::addNoFlightArea, this);

    subIsReady = n.subscribe("pathplanner/get_is_ready", 1, &rosMsg::isReady, this );

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
    pubFetchNoFlightZones = n.advertise<std_msgs::Bool>("utm/request_no_flight_zones", 1);
    pubIsReady = n.advertise<std_msgs::Bool>("pathplanner/is_ready", 1);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std_msgs::Bool msg;
    msg.data = true;
    pubFetchNoFlightZones.publish(msg);

    std::cout << "Subscribe and publish completed" << std::endl;
}

void rosMsg::checkForNewNoFlightZones()
{
    if(initialZonesLoaded) {
        std::cout << "Checking for new dynamic zones" << std::endl;
        std_msgs::Bool msg;
        msg.data = false;
        pubFetchNoFlightZones.publish(msg);
    }
}

bool rosMsg::checkIfIDExists(int id)
{
    for(auto &it : dynamicIDs)
        if(id == it)
            return true;
    return false;
};
