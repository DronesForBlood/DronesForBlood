

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <PATHPLANNER/start_end_coord.h>
#include <PATHPLANNER/flight_mission.h>
#include <PATHPLANNER/request_flight_mission.h>
#include <PATHPLANNER/add_no_flight_circle.h>
#include <PATHPLANNER/add_no_flight_area.h>

void gotPath(const PATHPLANNER::flight_mission &msg)
{
    std::vector<double> path = msg.mission_list;

    for(int i = 0; i < path.size(); i += 2)
        std::cout << path[i] << "    " << path[i+1] << std::endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sim");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<PATHPLANNER::start_end_coord>("mapParam", 1000);
    ros::Publisher pubRequestPath = n.advertise<PATHPLANNER::request_flight_mission>("requestPath", 1000);

    ros::Subscriber subPath = n.subscribe("flightMission", 1, &gotPath);

    PATHPLANNER::start_end_coord temp;

    std::vector<double> flyCoords {55.056010, 10.606016, 55.185354, 10.468279};
    temp.coords = flyCoords;

    std::vector<long> params {20, 2000, 1000 };
    temp.mapParams = params;

	ros::Rate loopRate(1);

    ros::spinOnce();
    loopRate.sleep();

    bool first = true;

	
	while(ros::ok()) {
	
        std::cout << "Running.." << std::endl;

        if(first) {
            chatter_pub.publish(temp);
            first = false;
        }

        PATHPLANNER::request_flight_mission requestMsg;
        pubRequestPath.publish(requestMsg);

    	ros::spinOnce();
    	loopRate.sleep();
	}


    ros::spin();
}

