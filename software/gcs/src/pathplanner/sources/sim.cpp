

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>

#include <utm/utm_tracking_data.h>
#include <utm/utm_no_flight_circle.h>

static bool isReady = false;

void gotPath(const mavlink_lora::mavlink_lora_mission_list &msg)
{
    std::cout << "GOT PATH!" << std::endl;

    /*
    for(auto &it : msg.waypoints) {
        std::cout << it.x << std::endl;
        std::cout << it.y << std::endl;
        std::cout << it.z << std::endl;
    }
    */
}

void pathplannerReady(const std_msgs::Bool &msg)
{
    isReady = msg.data;

    if(isReady)
        std::cout << "PATHPLANNER IS NOW READY" << std::endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sim");

    ros::NodeHandle n;

    ros::Publisher currentPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("mavlink_pos", 1);
    //ros::Publisher goalPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("dronelink/destination", 1);
    ros::Publisher calculatePathPub = n.advertise<std_msgs::Bool>("gcs_master/calculate_path", 1);
    ros::Publisher readyPub = n.advertise<mavlink_lora::mavlink_lora_pos>("pathplanner/get_is_ready", 1);
    ros::Publisher dronePub = n.advertise<utm::utm_tracking_data>("utm/add_tracking_data", 1);


    ros::Subscriber pathSub = n.subscribe("pathplanner/mission_list", 1, &gotPath);
    ros::Subscriber readySub = n.subscribe("pathplanner/is_ready", 1, &pathplannerReady);

    /*
    self.no_flight_circles_pub = rospy.Publisher(
        "utm/fetch_no_flight_circles",
        utm_no_flight_circle,
        queue_size=1000)
    */

    ros::Publisher addNoFlightZonePub = n.advertise<utm::utm_no_flight_circle>("utm/fetch_no_flight_circles", 1000);
    utm::utm_no_flight_circle noFlightMsg;
    noFlightMsg.id = 3057027508;
    noFlightMsg.lat = 55.47212;//55.472002;
    noFlightMsg.lon = 10.41734;//10.415481;
    noFlightMsg.radius = 25;
    noFlightMsg.name = "Dont know dont care";


    mavlink_lora::mavlink_lora_pos currentPositionMsg;
    currentPositionMsg.lat = 55.472015;
    currentPositionMsg.lon = 10.414711;

    mavlink_lora::mavlink_lora_pos goalPositionMsg;
    goalPositionMsg.lat = 55.472127;
    goalPositionMsg.lon = 10.417346;

    utm::utm_tracking_data droneMsg;

    ros::Rate loopRate(0.5);

    ros::spinOnce();
    loopRate.sleep();

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

    //goalPositionPub.publish(goalPositionMsg);
	
    bool first = true;
	while(ros::ok()) {

        std::cout << "Running.." << std::endl;

        dronePub.publish(droneMsg);
        currentPositionPub.publish(currentPositionMsg);

        if(!isReady) {
            readyPub.publish(goalPositionMsg);
        }
        else {
            std::cout << "Send calculatePath" << std::endl;
            std_msgs::Bool msg;
            calculatePathPub.publish(msg);


            noFlightMsg.epochValidFrom = int(std::time(nullptr)) + 0;
            noFlightMsg.epochValidTo = int(std::time(nullptr)) + 2500;

            if(first)
                addNoFlightZonePub.publish(noFlightMsg);
            first = false;
        }

        ros::spinOnce();
        loopRate.sleep();

	}

    std::cout << "Stopped sim?" << std::endl;


    ros::spin();
}


