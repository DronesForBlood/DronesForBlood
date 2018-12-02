

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>

#include <utm/utm_tracking_data.h>
#include <utm/utm_no_flight_circle.h>

#include <headers/global/geofunctions.h>

struct GPS {
    double latitude;
    double longitude;
    double altitude;
};

struct UTMDrone {
    GPS next_WP;
    GPS cur_pos;
    double next_vel;
    double cur_vel;
    double next_heading;
    double cur_heading;
    int gps_time;
    int drone_priority;
    int ETA_next_WP;
    int drone_id;
};

static UTMDrone drone;

static bool isReady = false;

static bool emptyPath = false;

static ros::Publisher calculatePathPub;

void pathplannerReady(const std_msgs::Bool &msg)
{
    isReady = msg.data;

    if(isReady) {
        std::cout << "PATHPLANNER IS NOW READY" << std::endl;
        std::cout << "Send calculatePath" << std::endl;
        std_msgs::Bool msg;
        calculatePathPub.publish(msg);
    }
}

double calculateHeading(GPS &current, GPS &next)
{
    std::pair<double,double> currentCoord(current.latitude, current.longitude);
    std::pair<double,double> nextCoord(next.latitude, next.longitude);

    return GeoFunctions::calcAngle(currentCoord, nextCoord);
}

void gotPath(const mavlink_lora::mavlink_lora_mission_list &msg)
{
    std::cout << "GOT PATH!" << std::endl;


    if(!msg.waypoints.empty()) {
        drone.next_WP.latitude = double(msg.waypoints.front().x)/1e7;
        drone.next_WP.longitude = double(msg.waypoints.front().y)/1e7;
        std::cout << "drone.next_WP.latitude: " << drone.next_WP.latitude << std::endl;
        std::cout << "drone.next_WP.longitude: " << drone.next_WP.longitude << std::endl;
        drone.cur_heading = calculateHeading(drone.cur_pos, drone.next_WP);
    }
    else
        emptyPath = true;
}


int calculateETA(UTMDrone &drone)
{
    std::pair<double,double> currentCoord(drone.cur_pos.latitude, drone.cur_pos.longitude);
    std::pair<double,double> nextCoord(drone.next_WP.latitude, drone.next_WP.longitude);

    double distance = GeoFunctions::calcMeterDistanceBetweensCoords(currentCoord, nextCoord);

    int secondsToArrival = int(distance / drone.cur_vel);

    return secondsToArrival + int(drone.gps_time);
}

void calculateNextDrone(UTMDrone &drone)
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

UTMDrone makeDrone(std::pair<double,double> currentCoord, std::pair<double,double> nextCoord)
{
    GPS gpsCurrent;
    gpsCurrent.altitude = 20;
    gpsCurrent.latitude = currentCoord.first;
    gpsCurrent.longitude = currentCoord.second;

    GPS gpsGoal;
    gpsGoal.altitude = 20;
    gpsGoal.latitude = nextCoord.first;
    gpsGoal.longitude = nextCoord.second;

    UTMDrone drone;

    drone.next_WP = gpsGoal;
    drone.cur_pos = gpsCurrent;
    drone.next_vel = 5;
    drone.cur_vel = 5;

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

    ros::Publisher currentPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("mavlink_pos", 1);
    //ros::Publisher goalPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("dronelink/destination", 1);
    calculatePathPub = n.advertise<std_msgs::Bool>("gcs_master/calculate_path", 1);
    ros::Publisher readyPub = n.advertise<mavlink_lora::mavlink_lora_pos>("pathplanner/get_is_ready", 1);
    ros::Publisher dronePub = n.advertise<utm::utm_tracking_data>("utm/add_tracking_data", 1);


    ros::Subscriber pathSub = n.subscribe("pathplanner/mission_list", 1, &gotPath);
    ros::Subscriber readySub = n.subscribe("pathplanner/is_ready", 1, &pathplannerReady);


    std::pair<double,double> currentCoord(55.472015, 10.414711);
    std::pair<double,double> goalCoord(55.472127, 10.417346);

    mavlink_lora::mavlink_lora_pos currentPositionMsg;
    currentPositionMsg.lat = currentCoord.first;
    currentPositionMsg.lon = currentCoord.second;

    mavlink_lora::mavlink_lora_pos initPositionMsg;
    initPositionMsg.lat = currentCoord.first;
    initPositionMsg.lon = currentCoord.second;

    mavlink_lora::mavlink_lora_pos goalPositionMsg;
    goalPositionMsg.lat = goalCoord.first;
    goalPositionMsg.lon = goalCoord.second;

    utm::utm_tracking_data droneMsg;

    ros::Rate loopRateSlow(0.25);
    ros::Rate loopRateFast(1);

    ros::spinOnce();

    droneMsg.uav_op_status = 3;
    droneMsg.pos_cur_lat_dd = 55.472022;
    droneMsg.pos_cur_lng_dd = 10.416062;
    droneMsg.pos_cur_alt_m = 20;
    droneMsg.pos_cur_hdg_deg = 100;
    droneMsg.pos_cur_vel_mps = 10;
    droneMsg.pos_cur_gps_timestamp = 1234;
    droneMsg.wp_next_lat_dd = 55.472160;
    droneMsg.wp_next_lng_dd = 10.416674;
    droneMsg.wp_next_alt_m = 20;
    droneMsg.wp_next_hdg_deg = 100;
    droneMsg.wp_next_vel_mps = 10;
    droneMsg.wp_next_eta_epoch = 1234;
    droneMsg.uav_bat_soc = 100;

    dronePub.publish(droneMsg);

	
    bool first = true;
	while(ros::ok()) {


        //dronePub.publish(droneMsg);

        if(!isReady) {
            std::cout << "Running not ready" << std::endl;
            currentPositionPub.publish(currentPositionMsg);
            readyPub.publish(goalPositionMsg);
            ros::spinOnce();
            loopRateSlow.sleep();
        }
        else if (first){
            std::cout << "Running first" << std::endl;
            first = false;
            drone = makeDrone(currentCoord, goalCoord);
        }
        else {
            std::cout << "Running else" << std::endl;
            std::pair<double,double> currentPos(drone.cur_pos.latitude, drone.cur_pos.longitude);
            std::pair<double,double> nextPos(drone.next_WP.latitude, drone.next_WP.longitude);
            double distanceToNextWp = GeoFunctions::calcMeterDistanceBetweensCoords(currentPos, nextPos);

            if(distanceToNextWp < 10) {
                if(emptyPath && isReady) {
                    std::cout << "GENERATE NEW MAP" << std::endl;

                    mavlink_lora::mavlink_lora_pos temp = goalPositionMsg;
                    goalPositionMsg = initPositionMsg;
                    initPositionMsg = temp;

                    std::pair<double,double> temp2 = goalCoord;
                    goalCoord = currentCoord;
                    currentCoord = temp2;
                    currentPositionPub.publish(currentPositionMsg);

                    ros::spinOnce();
                    loopRateFast.sleep();
                    isReady = false;
                    emptyPath = false;
                    first = true;
                }
                else {
                    std_msgs::Bool msg;
                    calculatePathPub.publish(msg);
                }
            }

            if(isReady) {
                calculateNextDrone(drone);

                currentPositionMsg.lat = drone.cur_pos.latitude;
                currentPositionMsg.lon = drone.cur_pos.longitude;
                currentPositionPub.publish(currentPositionMsg);
                ros::spinOnce();
                loopRateFast.sleep();
            }
        }



	}

    std::cout << "Stopped sim?" << std::endl;


    ros::spin();
}


