

#include <iostream>
#include <sstream>
#include "headers/simulation.h"
#include "headers/mapcontroller.h"
#include "headers/rosMsg.h"

#include <std_msgs/Bool.h>

static bool utmReady = false;

void utmIsUp(const std_msgs::Bool &msg)
{
    utmReady = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathplanner");

    ros::NodeHandle n;

    ros::Subscriber subIsReady = n.subscribe("utm/utm_is_up", 1, &utmIsUp );

    ros::Publisher pubCheckReady  = n.advertise<std_msgs::Bool>("utm/request_utm_is_up", 1);

    std_msgs::Bool msg;

    ros::Rate loopRate(0.5);
    ros::spinOnce();

    loopRate.sleep();

    std::cout << "Connecting to UTM node" << std::endl;

    while(ros::ok() && !utmReady) {
        pubCheckReady.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    std::cout << "Succesfully connected to UTM node" << std::endl;

    rosMsg rosMsgObject;

    while(ros::ok()) {
        rosMsgObject.checkForNewNoFlightZones();

        ros::spinOnce();
        loopRate.sleep();
    }

    ros::spin();
}
