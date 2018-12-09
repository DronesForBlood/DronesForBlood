

#include <iostream>
#include <sstream>
#include <utility>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <drone_decon/UTMDroneList.h>
#include <drone_decon/UTMDrone.h>
#include <drone_decon/RegisterDrone.h>
#include <drone_decon/RedirectDrone.h>
#include <drone_decon/takeOffAndLandCheck.h>

#include "geofunctions.hpp"

void gotRedirect(const drone_decon::RedirectDrone &msg)
{
    std::cout << "GOT REDIRECT!" << std::endl;
    std::cout << msg.drone_id << std::endl;
    std::cout << msg.insertBeforeNextWayPoint << std::endl;
    std::cout << msg.position.latitude << " " << msg.position.longitude << " " << msg.position.altitude << std::endl;
}

double calculateHeading(drone_decon::GPS &current, drone_decon::GPS &next)
{
    std::pair<double,double> currentCoord(current.latitude, current.longitude);
    std::pair<double,double> nextCoord(next.latitude, next.longitude);

    return GeoFunctions::calcAngle(currentCoord, nextCoord);
}

int calculateETA(drone_decon::UTMDrone &drone)
{
    std::pair<double,double> currentCoord(drone.cur_pos.latitude, drone.cur_pos.longitude);
    std::pair<double,double> nextCoord(drone.next_WP.latitude, drone.next_WP.longitude);

    double distance = GeoFunctions::calcMeterDistanceBetweensCoords(currentCoord, nextCoord);

    int secondsToArrival = int(distance / drone.cur_vel);

    return secondsToArrival + int(drone.gps_time);
}

void calculateNextDrone(drone_decon::UTMDrone &drone)
{
    int currentTime = int(std::time(nullptr));
    int timeSinceLastUpdate = currentTime - int(drone.gps_time);

    double distanceMoved = drone.cur_vel * timeSinceLastUpdate;

    double movementInX = cos(drone.cur_heading*PI/180) * distanceMoved;
    double movementInY = sin(drone.cur_heading*PI/180) * distanceMoved;

    double latIncrease = movementInX / 111111.;
    double lonIncrease = movementInY / (111111. * cos(drone.cur_pos.latitude*PI/180));

    drone.cur_pos.latitude += latIncrease;
    drone.cur_pos.longitude += lonIncrease;

    //std::cout << "drone.cur_heading: " << drone.cur_heading << std::endl;

    drone.cur_heading = calculateHeading(drone.cur_pos, drone.next_WP);
    drone.gps_time = currentTime;
    drone.ETA_next_WP = calculateETA(drone);

    std::cout.precision(10);
    //std::cout << drone.cur_pos.latitude << ", " << drone.cur_pos.longitude << std::endl;
}

drone_decon::UTMDrone makeDrone(unsigned int id, std::pair<double,double> currentCoord, std::pair<double,double> nextCoord)
{
    drone_decon::GPS gpsCurrent;
    gpsCurrent.altitude = 20;
    gpsCurrent.latitude = currentCoord.first;
    gpsCurrent.longitude = currentCoord.second;

    drone_decon::GPS gpsGoal;
    gpsGoal.altitude = 20;
    gpsGoal.latitude = nextCoord.first;
    gpsGoal.longitude = nextCoord.second;

    drone_decon::UTMDrone drone;

    drone.drone_id = id;
    drone.next_WP = gpsGoal;
    drone.cur_pos = gpsCurrent;
    drone.next_vel = 10;
    drone.cur_vel = 10;

    drone.cur_heading = calculateHeading(gpsCurrent, gpsGoal);
    drone.next_heading = calculateHeading(gpsGoal, gpsCurrent) - 5;

    drone.gps_time = int(std::time(nullptr));
    drone.ETA_next_WP = calculateETA(drone);

    drone.drone_priority = 3;

    return drone;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim");

    ros::NodeHandle n;

    ros::Publisher droneListPub = n.advertise<drone_decon::UTMDroneList>("/utm/dronesList", 100);
    ros::Publisher droneRegisterPub = n.advertise<drone_decon::RegisterDrone>("/drone_decon/register", 100);

    ros::Subscriber droneRedirectSub = n.subscribe("drone_decon/redirect", 10, &gotRedirect);

    ros::Rate loopRate(0.5);

    ros::spinOnce();
    loopRate.sleep();

    drone_decon::RegisterDrone registerMsg;
    registerMsg.drone_id = 3013;

    droneRegisterPub.publish(registerMsg);

    std::pair<double,double> currentCoord1(55.472089, 10.414875);
    std::pair<double,double> nextCoord1(55.472169, 10.417184);


    std::pair<double,double> currentCoord2(55.47228,10.41148);//(55.471871, 10.414911);
    std::pair<double,double> nextCoord2(55.4728, 10.4135);//(55.471925, 10.417237);

    drone_decon::UTMDrone drone1 = makeDrone(3013, currentCoord2, nextCoord1);
    drone1.cur_vel = 30;

    drone_decon::UTMDrone drone2 = makeDrone(3012, nextCoord2, currentCoord1);
    drone2.cur_pos.altitude = 22;
    drone2.next_WP.altitude = 20;
    drone2.drone_priority = 3;
    drone2.cur_vel = 10;

    drone_decon::UTMDroneList droneListMsg;
    droneListMsg.drone_list.push_back(drone1);
    droneListMsg.drone_list.push_back(drone2);


    droneListPub.publish(droneListMsg);
    ros::spinOnce();
    loopRate.sleep();

    while(ros::ok()) {
        //std::cout << "Update and send drones" << std::endl;
        calculateNextDrone(drone1);
        calculateNextDrone(drone2);

        droneListMsg.drone_list.clear();
        droneListMsg.drone_list.push_back(drone1);
        droneListMsg.drone_list.push_back(drone2);
        //droneListPub.publish(droneListMsg);

        //std::cout << "Running.." << std::endl;

        ros::spinOnce();
        loopRate.sleep();

    }

    std::cout << "Stopped sim?" << std::endl;


    ros::spin();
}

