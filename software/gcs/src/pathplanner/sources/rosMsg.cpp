
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
    std::cout << "Check ready" << std::endl;

    initialZonesLoaded = numberOfZonesReceived >= numberOfExpectedZones;
    //std::cout << "numberOfZonesReceived: " << numberOfZonesReceived << std::endl;

    bool currentIsInsideNoFligthZone = false;
    if(currentCoordSet)
        currentIsInsideNoFligthZone = controller.checkIfPointIsInNoFlightZone(currentCoord);

    bool hasAllInformation = initialZonesLoaded && currentCoordSet && !currentIsInsideNoFligthZone;

    if(!initialZonesLoaded)
        std::cout << "Not ready! Waiting for no flight zones" << std::endl;
    else if(!currentCoordSet)
        std::cout << "Not ready! Waiting for current position" << std::endl;
    else if(currentIsInsideNoFligthZone)
        std::cout << "Not ready! Current position is inside a no flight zone" << std::endl;

    // REMOVE THIS LINE
    // hasAllInformation = initialZonesLoaded && currentCoordSet;

    if(hasAllInformation) {
        std::pair<double, double> coord(msg.lat, msg.lon);

        bool sameGoalAsLast = GeoFunctions::calcMeterDistanceBetweensCoords(coord, goalCoord) < 10;

        if(!sameGoalAsLast) {
            std::cout << "New goal set" << std::endl;
            bool goalIsInsideNoFligthZone = controller.checkIfPointIsInNoFlightZone(coord);

            if(!goalIsInsideNoFligthZone) {
                goalCoordSet = true;
                goalCoord.first = coord.first;
                goalCoord.second = coord.second;
                generateNewMap();
                std::cout << "The pathplanner is ready" << std::endl;
                pathplannerReady = true;
            }
            else {
                solvingStarted = false;
                mapHasBeenGenerated = false;
                pathplannerReady = false;
                std::cout << "Not ready! Goal position is inside a no flight zone" << std::endl;
            }

        }
        else {
            std::cout << "Same goal set" << std::endl;
            //solvingStarted = false;
            //mapHasBeenGenerated = false;
        }
    }
    else {
        solvingStarted = false;
        mapHasBeenGenerated = false;
        pathplannerReady = false;
    }

    std_msgs::Bool newMsg;
    newMsg.data = pathplannerReady;
    pubIsReady.publish(newMsg);
}

bool rosMsg::getIsReady()
{
    return pathplannerReady;
}

void rosMsg::getZonesFromUTM()
{
    std::cout << "getZonesFromUTM" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std_msgs::Bool msg;
    msg.data = true;
    pubFetchNoFlightZones.publish(msg);
}

void rosMsg::setNumberOfExpectedZones(const std_msgs::Int64 &msg)
{
    if(!initialZonesLoaded) {
        numberOfExpectedZones = int(msg.data);
        std::cout << "numberOfExpectedZones: " << numberOfExpectedZones << std::endl;
    }
}

void rosMsg::addNoFlightCircle(const utm::utm_no_flight_circle &msg)
{
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
        DynamicNoFlightZone newZone;
        newZone.ID = int(msg.id);
        newZone.epochFrom = int(msg.epochValidFrom);
        newZone.epochTo = int(msg.epochValidTo);

        bool idExists = checkIfZoneExists(newZone);
        if(idExists) {
            return;
        }
        else
            dynamicZones.push_back(newZone);
    }

    bool intersectsWithFlightPath = false;

    if(solvingStarted)
        intersectsWithFlightPath = controller.updatePenaltyOfAreaCircle(coord, msg.radius + expandZonesByMeters, 10000, msg.epochValidFrom, msg.epochValidTo);

    controller.addPreMapPenaltyOfAreaCircle(coord, msg.radius + expandZonesByMeters, 10000, msg.epochValidFrom, msg.epochValidTo);

    std::cout << "intersectsWithFlightPath " << intersectsWithFlightPath << std::endl;
    //std::cout << "solvingStarted " << solvingStarted << std::endl;

    if(intersectsWithFlightPath) {
        controller.setCurrentHeading(currentCoord);
        std_msgs::Bool msg;
        pubEmergency.publish(msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        calculatePath(msg);
    }

    double distanceToCircle = GeoFunctions::calcMeterDistanceBetweensCoords(coord, goalCoord);
    if(distanceToCircle <= msg.radius)
        publishBlockedGoal(int(msg.epochValidTo));
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
        DynamicNoFlightZone newZone;
        newZone.ID = int(msg.id);
        newZone.epochFrom = int(msg.epochValidFrom);
        newZone.epochTo = int(msg.epochValidTo);

        bool idExists = checkIfZoneExists(newZone);
        if(idExists)
            return;
        else
            dynamicZones.push_back(newZone);
    }

    std::vector<std::pair<double,double>> coords;

    for(size_t i = 0; i < msg.polygonCoordinates.size(); i += 2) {
        std::pair<double,double> coord(msg.polygonCoordinates[i], msg.polygonCoordinates[i+1]);
        coords.push_back(coord);
        std::cout << coord.first << ", " << coord.second << std::endl;
    }

    coords = GeoFunctions::offsetPolygon(coords, expandZonesByMeters);

    bool intersectsWithFlightPath = false;

    if(solvingStarted)
        intersectsWithFlightPath = controller.updatePenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);

    controller.addPreMapPenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);

    std::cout << "intersectsWithFlightPath " << intersectsWithFlightPath << std::endl;

    if(intersectsWithFlightPath) {
        controller.setCurrentHeading(currentCoord);
        std_msgs::Bool msg;
        pubEmergency.publish(msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        calculatePath(msg);
    }

    if(GeoFunctions::pointIsInsidePolygon(coords, goalCoord))
        publishBlockedGoal(int(msg.epochValidTo));
}

void rosMsg::checkDrones(const utm::utm_tracking_data &msg)
{

}

void rosMsg::rallyPointsForBlockedGoal(const utm::utm_rally_point_list &msg)
{
    std::cout << "CHECK RALLY POINTS PLZ. Size: " << msg.rally_point_list.size() << std::endl;

    std::pair<double,double> shortestDistancePoint(msg.rally_point_list[0].lat_dd, msg.rally_point_list[0].lng_dd);
    double minDistance = GeoFunctions::calcMeterDistanceBetweensCoords(shortestDistancePoint, currentCoord);

    for(auto &it : msg.rally_point_list) {
        std::pair<double,double> point(it.lat_dd, it.lng_dd);
        double distanceToPoint = GeoFunctions::calcMeterDistanceBetweensCoords(point, currentCoord);
        if(distanceToPoint < minDistance) {
            shortestDistancePoint = point;
            minDistance = distanceToPoint;
        }
    }

    double distanceToHome = GeoFunctions::calcMeterDistanceBetweensCoords(initCoord, currentCoord);

    pathplanner::blocked_goal newMsg;

    newMsg.epochBlockedUntil = epochBlockedUntil;

    newMsg.distanceToHomePoint_m = distanceToHome;
    newMsg.homePointLat = initCoord.first;
    newMsg.homePointLon = initCoord.second;

    newMsg.distanceToRallyPoint_m = minDistance;
    newMsg.rallyPointLat = shortestDistancePoint.first;
    newMsg.rallyPointLon = shortestDistancePoint.second;

    pubBlockedGoal.publish(newMsg);
}

void rosMsg::setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    currentCoordSet = true;

    currentCoord.first = msg.lat;
    currentCoord.second = msg.lon;

    if(solvingStarted)
        controller.setCurrentPosition(currentCoord);

    if(!mapHasBeenGenerated && goalCoordSet) {
        std::cout << "THIS SHOULD NEVER HAPPEN... right?" << std::endl;
        //generateNewMap();
        //controller.setCurrentPosition(currentCoord);
    }
}

void rosMsg::calculatePath(const std_msgs::Bool &msg)
{
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

        controller.setCurrentHeading(path.back());

        double ETA = 0;
        std::pair<double,double> previousCoord = currentCoord;

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

            double distanceToPoint = GeoFunctions::calcMeterDistanceBetweensCoords(previousCoord, std::pair<double,double>(it->first, it->second));
            ETA += distanceToPoint / DRONE_MAX_SPEED;

            item.param1 = ETA;

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
    else
        std::cout << "UNABLE TO FIND A PATH. THIS MIGHT BE BAD" << std::endl;

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

    initCoord = currentCoord;

    nodeDist = 1;
    mapWidth =  200;
    padLength = 100;

    std::cout << "Start: " << currentCoord.first << " " << currentCoord.second << "    ";
    std::cout << "Goal: " << goalCoord.first << " " << goalCoord.second << "    ";
    std::cout << "Node dist: " << nodeDist << "    ";
    std::cout << "Map width: " << mapWidth << "    ";
    std::cout << "Pad length: " << padLength << "    " << std::endl;

    std::cout << "Generate new map begin" << std::endl;
    controller.generateMap(currentCoord, goalCoord, nodeDist, mapWidth, padLength);
    std::cout << "Generate new map done" << std::endl;
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
    subDrones = n.subscribe("utm/fetch_tracking_data", 1000, &rosMsg::checkDrones, this);
    subRallyPoints = n.subscribe("utm/fetch_rally_points", 1, &rosMsg::rallyPointsForBlockedGoal, this );

    subIsReady = n.subscribe("pathplanner/get_is_ready", 1, &rosMsg::isReady, this );

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
    pubFetchNoFlightZones = n.advertise<std_msgs::Bool>("utm/request_no_flight_zones", 1);
    pubIsReady = n.advertise<std_msgs::Bool>("pathplanner/is_ready", 1);
    pubEmergency = n.advertise<std_msgs::Bool>("pathplanner/emergency", 1);
    pubBlockedGoal = n.advertise<pathplanner::blocked_goal>("pathplanner/blocked_goal", 1);
    pubFetchRallyPoints = n.advertise<std_msgs::Bool>("utm/request_rally_points", 1);

    getZonesFromUTM();

    std::cout << "Subscribers and publishers succesfully setup" << std::endl;
}

void rosMsg::checkForNewNoFlightZones()
{
    //std::cout << "checkForNewNoFlightZones" << std::endl;
    if(initialZonesLoaded) {
        std_msgs::Bool msg;
        msg.data = false;
        pubFetchNoFlightZones.publish(msg);
    }
}

bool rosMsg::checkIfZoneExists(DynamicNoFlightZone &zone)
{
    //std::cout << "Checking if id " << id << " exists" << std::endl;
    for(auto it = dynamicZones.begin(); it < dynamicZones.end(); it++)
        if(zone.ID == it->ID) {
            int currentTime = int(std::time(nullptr));
            if(currentTime <= it->epochTo)
                return true;

            dynamicZones.erase(it);
            return false;
        }
    return false;
}

void rosMsg::publishBlockedGoal(int epochOver)
{
    std::cout << "BLOCKED GOAL!" << std::endl;
    epochBlockedUntil = epochOver;
    std_msgs::Bool msg;
    pubFetchRallyPoints.publish(msg);


};
