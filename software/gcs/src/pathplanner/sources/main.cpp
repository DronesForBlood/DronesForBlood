

#include <iostream>
#include <sstream>
#include "headers/simulation.h"
#include "headers/mapcontroller.h"
#include "headers/rosMsg.h"

#include <PATHPLANNER/start_end_coord.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathplanner");

    rosMsg rosMsgObject;

    ros::Rate loopRate(0.5);
    ros::spinOnce();

    loopRate.sleep();

    while(ros::ok()) {

        std::cout << "Running.." << std::endl;

        rosMsgObject.checkForNewNoFlightZones();

        ros::spinOnce();
        loopRate.sleep();
    }

    ros::spin();
}
