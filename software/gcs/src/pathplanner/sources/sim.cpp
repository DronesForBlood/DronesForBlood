

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <PATHPLANNER/no_flight_circle.h>
#include <PATHPLANNER/no_flight_area.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>


void gotPath(const mavlink_lora::mavlink_lora_mission_list &msg)
{
    std::cout << "GOT PATH!" << std::endl;
    /*
    std::vector<double> path = msg.;

    for(int i = 0; i < path.size(); i += 2)
        std::cout << path[i] << "    " << path[i+1] << std::endl;
        */
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sim");

    ros::NodeHandle n;

    ros::Publisher currentPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("mavlink_pos", 1);
    ros::Publisher goalPositionPub = n.advertise<mavlink_lora::mavlink_lora_pos>("dronelink/destination", 1);
    ros::Publisher calculatePathPub = n.advertise<std_msgs::Bool>("gcs_master/calculate_path", 1);

    ros::Subscriber pathSub = n.subscribe("pathplanner/mission_list", 1, &gotPath);

    mavlink_lora::mavlink_lora_pos currentPositionMsg;
    currentPositionMsg.lat = 55.056010;
    currentPositionMsg.lon = 10.606016;

    mavlink_lora::mavlink_lora_pos goalPositionMsg;
    goalPositionMsg.lat = 55.185354;
    goalPositionMsg.lon = 10.468279;

    ros::Rate loopRate(0.5);

    ros::spinOnce();
    loopRate.sleep();

    bool first = true;
	
	while(ros::ok()) {
	
        std::cout << "Running.." << std::endl;

        if(first) {
            currentPositionPub.publish(currentPositionMsg);
            goalPositionPub.publish(goalPositionMsg);
            first = false;
        }

        std_msgs::Bool msg;
        calculatePathPub.publish(msg);

        //PATHPLANNER::request_flight_mission requestMsg;
        //pubRequestPath.publish(requestMsg);

    	ros::spinOnce();
    	loopRate.sleep();
	}


    ros::spin();
}


