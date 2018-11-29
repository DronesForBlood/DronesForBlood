
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{
    subStart();
}

rosMsg::~rosMsg()
{

}

void rosMsg::isReady(const mavlink_lora::mavlink_lora_pos &msg)
{
    initialZonesLoaded = numberOfZonesReceived >= numberOfExpectedZones;

    bool ready = initialZonesLoaded && currentCoordSet;

    if(ready) {
        std::cout << "The pathplanner is ready" << std::endl;
        std::pair<double, double> coord(msg.lat, msg.lon);
        setGoalPosition(coord);
    }
    else
        std::cout << "The pathplanner is not ready" << std::endl;

    std_msgs::Bool newMsg;
    newMsg.data = ready;
    pubIsReady.publish(newMsg);
}

bool rosMsg::getIsReady()
{
    return initialZonesLoaded;
}

void rosMsg::getZonesFromUTM()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std_msgs::Bool msg;
    msg.data = true;
    pubFetchNoFlightZones.publish(msg);
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
        zoneMutex.lock();
        numberOfZonesReceived++;
        zoneMutex.unlock();
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

    bool intersectsWithFlightPath = false;

    if(solvingStarted)
        intersectsWithFlightPath = controller.updatePenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius, 10000, msg.epochValidFrom, msg.epochValidTo);

    if(intersectsWithFlightPath)
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void rosMsg::addNoFlightArea(const utm::utm_no_flight_area &msg)
{
    //std::cout << "Got no flight area" << std::endl;

    if(!initialZonesLoaded) {
        zoneMutex.lock();
        numberOfZonesReceived++;
        zoneMutex.unlock();
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

    bool intersectsWithFlightPath = false;

    if(solvingStarted)
        intersectsWithFlightPath = controller.updatePenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);
    else
        controller.addPreMapPenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);

    if(intersectsWithFlightPath)
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void rosMsg::checkDrones(const utm::utm_tracking_data &msg)
{

}

void rosMsg::setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    currentCoordSet = true;

    currentCoord.first = msg.lat;
    currentCoord.second = msg.lon;

    if(solvingStarted)
        controller.setCurrentPosition(currentCoord);

    if(!mapHasBeenGenerated && goalCoordSet) {
        generateNewMap();
        controller.setCurrentPosition(currentCoord);
    }

}

void rosMsg::setGoalPosition(std::pair<double, double> coord)
{
    goalCoordSet = true;

    goalCoord.first = coord.first;
    goalCoord.second = coord.second;

    if(currentCoordSet)
        generateNewMap();
}

void rosMsg::calculatePath(const std_msgs::Bool &msg)
{
    //std::cout << "Calculating a new path" << std::endl;
    if(!goalCoordSet || !currentCoordSet || !mapHasBeenGenerated) {
        std::cout << "Unable to calculate a path. Missing information" << std::endl;
        return;
    }

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
        path.pop_back();

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
            item.x = int32_t(it->first * 1e7);
            item.y = int32_t(it->second * 1e7);
            item.z = altitude;
            item.autocontinue = 1;

            /*
            std::cout << "Sending x: " << it->first << std::endl;
            std::cout << "Sending y: " << it->second << std::endl;
            std::cout << "Sending X: " << int32_t(it->first * 1e7) << std::endl;
            std::cout << "Sending Y: " << int32_t(it->second * 1e7) << std::endl;
            */

            newMsg.waypoints.push_back(item);
            i++;
        }
    }

    pubPath.publish(newMsg);
}

void rosMsg::generateNewMap()
{
    std::cout << "Generating a new map" << std::endl;
    if(!goalCoordSet || !currentCoordSet || !initialZonesLoaded) {
        std::cout << goalCoordSet << " " << currentCoordSet << " " << initialZonesLoaded << std::endl;
        std::cout << "Unable to generate new map. Missing information" << std::endl;
        return;
    }

    mapHasBeenGenerated = true;

    nodeDist = 1;
    mapWidth =  200;
    padLength = 100;

    std::cout << "Start: " << currentCoord.first << " " << currentCoord.second << "    ";
    std::cout << "Goal: " << goalCoord.first << " " << goalCoord.second << "    ";
    std::cout << "Node dist: " << nodeDist << "    ";
    std::cout << "Map width: " << mapWidth << "    ";
    std::cout << "Pad length: " << padLength << "    ";

    controller.generateMap(currentCoord, goalCoord, nodeDist, mapWidth, padLength);
    controller.setGoalPosition(goalCoord);

    solvingStarted = true;

    controller.startSolver(currentCoord);
};

void rosMsg::subStart()
{
    std::cout << "Setting up subscribers and publishers" << std::endl;

    numberOfExpectedZones = INT_MAX;
    numberOfZonesReceived = 0;

    subCurrentPosition = n.subscribe("mavlink_pos", 1, &rosMsg::setCurrentPosition, this );
    //subGoalPosition = n.subscribe("dronelink/destination", 1, &rosMsg::setGoalPosition, this );
    subCalculatePath = n.subscribe("gcs_master/calculate_path", 1, &rosMsg::calculatePath, this );

    subNumberOfZones = n.subscribe("utm/number_of_zones", 1, &rosMsg::setNumberOfExpectedZones, this);
    subNoFlightCircles = n.subscribe("utm/fetch_no_flight_circles", 1000, &rosMsg::addNoFlightCircle, this);
    subNoFlightAreas = n.subscribe("utm/fetch_no_flight_areas", 1000, &rosMsg::addNoFlightArea, this);
    subDrones = n.subscribe("utm/fetch_tracking_data", 1, &rosMsg::checkDrones, this);

    subIsReady = n.subscribe("pathplanner/get_is_ready", 1, &rosMsg::isReady, this );

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
    pubFetchNoFlightZones = n.advertise<std_msgs::Bool>("utm/request_no_flight_zones", 1);
    pubIsReady = n.advertise<std_msgs::Bool>("pathplanner/is_ready", 1);

    getZonesFromUTM();

    std::cout << "Subscribes and publishers succesfully setup" << std::endl;
}

void rosMsg::checkForNewNoFlightZones()
{
    if(initialZonesLoaded) {
        std_msgs::Bool msg;
        msg.data = false;
        pubFetchNoFlightZones.publish(msg);
    }
}

void rosMsg::checkForDrones()
{
    if(solvingStarted) {
        std_msgs::Bool msg;
        pubFetchDrones.publish(msg);
    }
}

bool rosMsg::checkIfIDExists(int id)
{
    //std::cout << "Checking if id " << id << " exists" << std::endl;
    for(auto &it : dynamicIDs)
        if(id == it)
            return true;
    return false;
};