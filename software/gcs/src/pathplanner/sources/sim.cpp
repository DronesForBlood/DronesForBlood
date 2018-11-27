

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <PATHPLANNER/no_flight_circle.h>
#include <PATHPLANNER/no_flight_area.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>

#include <utm/utm_tracking_data.h>

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

    mavlink_lora::mavlink_lora_pos currentPositionMsg;
    currentPositionMsg.lat = 55.472015;
    currentPositionMsg.lon = 10.414711;

    mavlink_lora::mavlink_lora_pos goalPositionMsg;
    goalPositionMsg.lat = 55.472127;
    goalPositionMsg.lon = 10.417346;

    utm::utm_tracking_data droneMsg;

    ros::Rate loopRate(0.1);

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
	
	while(ros::ok()) {

        std::cout << "Running.." << std::endl;

        currentPositionPub.publish(currentPositionMsg);

        if(!isReady) {
            readyPub.publish(goalPositionMsg);
        }
        else {
            std::cout << "Send calculatePath" << std::endl;
            std_msgs::Bool msg;
            calculatePathPub.publish(msg);
        }

        ros::spinOnce();
        loopRate.sleep();

	}

    std::cout << "Stopped sim?" << std::endl;


    ros::spin();
}


