
#include "headers/rosMsg.h"

rosMsg::rosMsg()
{

}

rosMsg::rosMsg(std::string *main, std::string *current)
{
    setStatusMessages(main, current);
    subStart();
}

rosMsg::~rosMsg()
{

}

void rosMsg::setCheckZonesBeforeTakeoff(bool val)
{
    checkZonesBeforeTakeoff = val;
}

void rosMsg::setStatusMessages(std::string *main, std::string *current)
{
    mainStatus = main;
    currentTask = current;
    *mainStatus = "Setting up";
}

void rosMsg::isReady(const mavlink_lora::mavlink_lora_pos &msg)
{
    if(waitingForRallyPoints)
        return;

    *mainStatus = "Not ready";
    *currentTask = "Checking if ready";
    //std::cout << "Check ready" << std::endl;

    initialZonesLoaded = numberOfZonesReceived >= numberOfExpectedZones;
    //std::cout << "numberOfZonesReceived: " << numberOfZonesReceived << std::endl;

    bool currentIsInsideNoFligthZone = false;
    if(currentCoordSet)
        currentIsInsideNoFligthZone = controller.checkIfPointIsInNoFlightZone(currentCoord);

    bool hasAllInformation = initialZonesLoaded && currentCoordSet && !currentIsInsideNoFligthZone;

    if(!initialZonesLoaded) {
        *currentTask = "Waiting for zones (" + std::to_string(numberOfExpectedZones) + ", " + std::to_string(numberOfZonesReceived) + ")";
        std::cout << "PATHPLANNER: Not ready! Waiting for no flight zones" << std::endl;
    }
    else if(!currentCoordSet) {
        *currentTask = "Waiting for the drones position";
        std::cout << "PATHPLANNER: Not ready! Waiting for current position" << std::endl;
    }
    else if(currentIsInsideNoFligthZone) {
        *currentTask = "Current position is in no fly zone";
        std::cout << "PATHPLANNER: Not ready! Current position is inside a no fly zone" << std::endl;
    }

    if(!checkZonesBeforeTakeoff)
        hasAllInformation = initialZonesLoaded && currentCoordSet;

    if(hasAllInformation) {
        std::pair<double, double> coord(msg.lat, msg.lon);

        bool sameGoalAsLast = GeoFunctions::calcMeterDistanceBetweensCoords(coord, goalCoord) < 2;

        if(!sameGoalAsLast) {
            //std::cout << "New goal set" << std::endl;
            bool goalIsInsideNoFligthZone = controller.checkIfPointIsInNoFlightZone(coord);

            if(!checkZonesBeforeTakeoff)
                goalIsInsideNoFligthZone = false;

            if(!goalIsInsideNoFligthZone) {
                goalCoordSet = true;
                goalCoord.first = coord.first;
                goalCoord.second = coord.second;
                generateNewMap();
                *mainStatus = "Ready";
                *currentTask = "Idle";
                std::cout << "PATHPLANNER: Ready!" << std::endl;
                pathplannerReady = true;
            }
            else {
                solvingStarted = false;
                mapHasBeenGenerated = false;
                pathplannerReady = false;
                *currentTask = "Goal position is in no fly zone";
                std::cout << "PATHPLANNER: Not ready! Goal position is inside a no flight zone" << std::endl;
            }

        }
        else {
            //std::cout << "Same goal set" << std::endl;
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
    *currentTask = "Waiting for zones (" + std::to_string(numberOfExpectedZones) + ", " + std::to_string(numberOfZonesReceived) + ")";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std_msgs::Bool msg;
    msg.data = true;
    pubFetchNoFlightZones.publish(msg);
}

void rosMsg::setNumberOfExpectedZones(const std_msgs::Int64 &msg)
{
    *currentTask = "Waiting for zones (" + std::to_string(numberOfExpectedZones) + ", " + std::to_string(numberOfZonesReceived) + ")";
    if(!initialZonesLoaded) {
        numberOfExpectedZones = int(msg.data);
        //std::cout << "numberOfExpectedZones: " << numberOfExpectedZones << std::endl;
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

    //std::cout << "intersectsWithFlightPath " << intersectsWithFlightPath << std::endl;
    //std::cout << "solvingStarted " << solvingStarted << std::endl;

    double distanceToCircle = GeoFunctions::calcMeterDistanceBetweensCoords(coord, goalCoord);
    if(distanceToCircle <= msg.radius)
        publishBlockedGoal(int(msg.epochValidTo));
    else if(intersectsWithFlightPath) {
        *currentTask = "Zone intersects path. Calculating new";
        std_msgs::Bool msg;
        pubEmergency.publish(msg);
        ros::spinOnce();
        controller.setCurrentHeading(currentCoord);

        //mavlink_lora::mavlink_lora_mission_list newMsg;
        //pubPath.publish(newMsg);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        //calculatePath(msg);
        //*currentTask = "Zone intersects path. New send";
    }
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
        //std::cout << coord.first << ", " << coord.second << std::endl;
    }

    coords = GeoFunctions::offsetPolygon(coords, expandZonesByMeters);

    bool intersectsWithFlightPath = false;

    if(solvingStarted)
        intersectsWithFlightPath = controller.updatePenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);

    controller.addPreMapPenaltyOfAreaPolygon(coords, 10000, msg.epochValidFrom, msg.epochValidTo);

    //std::cout << "intersectsWithFlightPath " << intersectsWithFlightPath << std::endl;

    if(GeoFunctions::pointIsInsidePolygon(coords, goalCoord))
        publishBlockedGoal(int(msg.epochValidTo));
    else if(intersectsWithFlightPath) {
        *currentTask = "Zone intersects path. Calculating new";
        std_msgs::Bool msg;
        pubEmergency.publish(msg);
        ros::spinOnce();
        controller.setCurrentHeading(currentCoord);

        //mavlink_lora::mavlink_lora_mission_list newMsg;
        //pubPath.publish(newMsg);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        //calculatePath(msg);
        //*currentTask = "Zone intersects path. New path send";
    }
}

void rosMsg::rallyPointsForBlockedGoal(const utm::utm_rally_point_list &msg)
{
    //std::cout << "CHECK RALLY POINTS PLZ. Size: " << msg.rally_point_list.size() << std::endl;

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

    bool rallyPointIsFree = true;
    for(auto &it : otherDrones.drone_list) {
        std::pair<double,double> otherDroneNextWp(it.next_WP.latitude, it.next_WP.longitude);
        double nextWpDistanceToRallyPoint = GeoFunctions::calcMeterDistanceBetweensCoords(shortestDistancePoint, otherDroneNextWp);
        if(nextWpDistanceToRallyPoint < 5) {
            rallyPointIsFree = false;
            break;
        }
    }

    waitingForRallyPoints = false;

    if(rallyPointIsFree) {
        goalCoord = shortestDistancePoint;
        generateNewMap();
        *mainStatus = "Ready";
        *currentTask = "Idle";
        generateNewMap();
    }
    else {
        bool homeIsClear = controller.checkIfPointIsInNoFlightZone(initCoord);
        if(homeIsClear) {
            goalCoord = initCoord;
            generateNewMap();
            *mainStatus = "Ready";
            *currentTask = "Idle";
            generateNewMap();
        }
        else {
            std_msgs::Bool msg;
            pubLandNow.publish(msg);
        }
    }

    /*
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
    */
}

void rosMsg::droneData(const drone_decon::UTMDroneList &msg)
{
    otherDrones = msg;
}

void rosMsg::setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg)
{
    currentCoordSet = true;

    currentActualAltitude = msg.alt;
    currentCoord.first = msg.lat;
    currentCoord.second = msg.lon;

    if(solvingStarted)
        controller.setCurrentPosition(currentCoord);

    if(!mapHasBeenGenerated && goalCoordSet) {
        //std::cout << "THIS SHOULD NEVER HAPPEN... right?" << std::endl;
        //generateNewMap();
        //controller.setCurrentPosition(currentCoord);
    }
}

void rosMsg::getPath(const std_msgs::Bool &msg)
{
    if(justGotRedirect) {
        justGotRedirect = false;
        pubPath.publish(missionMsg);
    }
    else
        calculatePath();
}

void rosMsg::calculatePath()
{
    if(!goalCoordSet || !currentCoordSet || !mapHasBeenGenerated) {
        *currentTask = "Cannot find path. Missing info";
        std::cout << "PATHPLANNER: Unable to calculate a path. Missing information" << std::endl;
        return;
    }

    missionMsg.waypoints.clear();

    bool succes = controller.getPathToDestination(path);

    double distanceFromExpectedPosition = GeoFunctions::calcMeterDistanceBetweensCoords(currentCoord, currentHeading);
    if(distanceFromExpectedPosition > 10) {
        controller.setCurrentHeading(currentCoord);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    if(!succes) {
        for(int i = 0; i < 3; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            succes = controller.getPathToDestination(path);
            if(succes)
                break;
        }
    }

    if(succes) {
        unsigned short i = 0;
        path.pop_back();

        currentHeading = path.back();
        controller.setCurrentHeading(currentHeading);

        double ETA = std::time(nullptr);
        std::pair<double,double> previousCoord = currentCoord;

        for(auto it = path.rbegin(); it != path.rend(); it++) {
        //for(int j = int(path.size()) - 1; j >= 0; j--) {
            mavlink_lora::mavlink_lora_mission_item_int item;
            item.target_system = 0;
            item.target_component = 0;
            item.seq = i;
            item.frame = 6;     // global pos, relative alt_int
            item.command = 16;
            item.param2 = 5;    // acceptance radius in mmsg = mavlink_lora.msg.mavlink_lora_mission_list()
            item.param3 = 0;    // pass though waypoint, no trajectory control
            item.x = int32_t(it->first * 1e7);
            item.y = int32_t(it->second * 1e7);
            item.z = altitude;
            item.autocontinue = 1;

            double distanceToPoint = GeoFunctions::calcMeterDistanceBetweensCoords(previousCoord, std::pair<double,double>(it->first, it->second));
            ETA += distanceToPoint / DRONE_MAX_SPEED;
            item.param4 = ETA;


            if(i < path.size() - 1) {
                std::pair<double, double> currentCoord(it->first, it->second);
                it++;
                std::pair<double, double> nextCoord((it)->first, (it)->second);
                it--;
                double heading = GeoFunctions::calcAngle(currentCoord, nextCoord);
                item.param1 = heading;
            }
            else
                item.param1 = 0.0;

            /*
            std::cout << "Sending x: " << it->first << std::endl;
            std::cout << "Sending y: " << it->second << std::endl;
            std::cout << "Sending X: " << int32_t(it->first * 1e7) << std::endl;
            std::cout << "Sending Y: " << int32_t(it->second * 1e7) << std::endl;
            */

            missionMsg.waypoints.push_back(item);
            i++;
        }
        pubPath.publish(missionMsg);
    }
    else {
        *currentTask = "UNABLE TO FIND A PATH";
        std::cout << "PATHPLANNER: UNABLE TO FIND A PATH. THIS MIGHT BE BAD" << std::endl;
    }
}

void rosMsg::gotRedirect(const drone_decon::RedirectDrone &msg)
{
    std::cout << "PATHPLANNER: Got redirect" << std::endl;

    if(missionMsg.waypoints.empty())
        return;

    altitude += msg.position.altitude - currentActualAltitude;
    if(altitude > 80) {
        altitude = 80;
        std::cout << "PATHPLANNER: Max altitude set: " << altitude << std::endl;
    }
    else
        std::cout << "PATHPLANNER: New altitude: " << altitude << std::endl;

    if(msg.insertBeforeNextWayPoint) {
        mavlink_lora::mavlink_lora_mission_item_int item = missionMsg.waypoints.front();
        item.x = int32_t(msg.position.latitude * 1e7);
        item.y = int32_t(msg.position.longitude * 1e7);
        item.z = altitude;
        missionMsg.waypoints.insert(missionMsg.waypoints.begin(), item);

        for(size_t i = 1; i < missionMsg.waypoints.size(); i++) {
            missionMsg.waypoints[i].seq = ushort(i);
            missionMsg.waypoints[i].z = altitude;
        }
    }
    else
        for(size_t i = 0; i < missionMsg.waypoints.size(); i++)
            missionMsg.waypoints[i].z = altitude;

    std_msgs::Bool emergencyMsg;
    pubEmergency.publish(emergencyMsg);

    justGotRedirect = true;
}

void rosMsg::generateNewMap()
{
    std::cout << "PATHPLANNER: Generating a new map" << std::endl;
    if(!goalCoordSet || !currentCoordSet || !initialZonesLoaded) {
        *currentTask = "Cannot make map. Missing info";
        std::cout << goalCoordSet << " " << currentCoordSet << " " << initialZonesLoaded << std::endl;
        std::cout << "PATHPLANNER: Unable to generate new map. Missing information" << std::endl;
        return;
    }

    *currentTask = "Generating new map";

    mapHasBeenGenerated = true;

    initCoord = currentCoord;

    nodeDist = 1;
    mapWidth =  200;
    padLength = 100;

    std::cout << "PATHPLANNER:\nStart: " << currentCoord.first << " " << currentCoord.second << "    ";
    std::cout << "Goal: " << goalCoord.first << " " << goalCoord.second << "    ";
    std::cout << "Node dist: " << nodeDist << "    ";
    std::cout << "Map width: " << mapWidth << "    ";
    std::cout << "Pad length: " << padLength << "    " << std::endl;

    //std::cout << "Generate new map begin" << std::endl;
    controller.generateMap(currentCoord, goalCoord, nodeDist, mapWidth, padLength);
    //std::cout << "Generate new map done" << std::endl;
    controller.setGoalPosition(goalCoord);

    solvingStarted = true;

    controller.startSolver(currentCoord);

    *currentTask = "Idle";
};

void rosMsg::subStart()
{
    *currentTask = "Setting up sub and pub";
    std::cout << "PATHPLANNER: Setting up subscribers and publishers" << std::endl;

    numberOfExpectedZones = INT_MAX;
    numberOfZonesReceived = 0;

    subCurrentPosition = n.subscribe("mavlink_pos", 1, &rosMsg::setCurrentPosition, this );
    //subGoalPosition = n.subscribe("dronelink/destination", 1, &rosMsg::setGoalPosition, this );
    subCalculatePath = n.subscribe("gcs_master/calculate_path", 1, &rosMsg::getPath, this );

    subNumberOfZones = n.subscribe("utm/number_of_zones", 1, &rosMsg::setNumberOfExpectedZones, this);
    subNoFlightCircles = n.subscribe("utm/fetch_no_flight_circles", 1000, &rosMsg::addNoFlightCircle, this);
    subNoFlightAreas = n.subscribe("utm/fetch_no_flight_areas", 1000, &rosMsg::addNoFlightArea, this);
    subRallyPoints = n.subscribe("utm/fetch_rally_points", 1, &rosMsg::rallyPointsForBlockedGoal, this );
    droneRedirectSub = n.subscribe("/drone_decon/redirect", 10, &rosMsg::gotRedirect, this);


    subDrones = n.subscribe("/utm/dronesList", 100, &rosMsg::droneData, this);
    subIsReady = n.subscribe("pathplanner/get_is_ready", 1, &rosMsg::isReady, this );

    pubPath  = n.advertise<mavlink_lora::mavlink_lora_mission_list>("pathplanner/mission_list", 1);
    pubFetchNoFlightZones = n.advertise<std_msgs::Bool>("utm/request_no_flight_zones", 1);
    pubIsReady = n.advertise<std_msgs::Bool>("pathplanner/is_ready", 1);
    pubEmergency = n.advertise<std_msgs::Bool>("pathplanner/emergency", 1);
    pubBlockedGoal = n.advertise<pathplanner::blocked_goal>("pathplanner/blocked_goal", 1);
    pubFetchRallyPoints = n.advertise<std_msgs::Bool>("utm/request_rally_points", 1);
    pubLandNow = n.advertise<std_msgs::Bool>("pathplanner/land_now", 1);


    getZonesFromUTM();

    std::cout << "PATHPLANNER: Subscribers and publishers succesfully setup" << std::endl;
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
    *currentTask = "PATHPLANNER: Goal is blocked";

    //std::cout << "BLOCKED GOAL!" << std::endl;
    epochBlockedUntil = epochOver;
    std_msgs::Bool msg;
    pubFetchRallyPoints.publish(msg);
    waitingForRallyPoints = true;

    *mainStatus = "Goal blocked";
    *currentTask = "Loading rally points";
};
