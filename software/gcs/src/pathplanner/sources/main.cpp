

#include <iostream>
#include <sstream>
#include "headers/simulation.h"
#include "headers/mapcontroller.h"
#include "headers/rosMsg.h"
#include "headers/global/geofunctions.h"

#include <lora_ground_control/heartbeat_node.h>

#include <std_msgs/Bool.h>

#include <algorithm>

#define HEARTBEAT_PERIOD 0.5
#define PATHPLANNER_ID 9

static bool utmReady = false;
static ros::Publisher pubHeartBeat;
static std::string *mainStatus;
static std::string *currentTask;
static bool *hasError = new bool(false);

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

void utmIsUp(const std_msgs::Bool &msg)
{
    utmReady = true;
}

void sendHeartBeat(){
    lora_ground_control::heartbeat_node heartBeatMsg;
    heartBeatMsg.id = PATHPLANNER_ID;
    heartBeatMsg.name = "Pathplanner";
    heartBeatMsg.expected_interval = HEARTBEAT_PERIOD;
    heartBeatMsg.main_status = *mainStatus;
    heartBeatMsg.current_task = *currentTask;
    heartBeatMsg.has_error = *hasError;
    pubHeartBeat.publish(heartBeatMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathplanner");

    ros::Time heartbeatMsgSent;

    ros::NodeHandle n;

    ros::Subscriber subIsReady = n.subscribe("utm/utm_is_up", 1, &utmIsUp );

    ros::Publisher pubCheckReady  = n.advertise<std_msgs::Bool>("utm/request_utm_is_up", 1);
    pubHeartBeat  = n.advertise<lora_ground_control::heartbeat_node>("/lora_ground_control/heartbeat_nodes_rx", 1);

    std_msgs::Bool msg;

    ros::Rate loopRate(0.5);
    ros::spinOnce();

    loopRate.sleep();

    std::cout << "PATHPLANNER: Connecting to UTM node" << std::endl;

    while(ros::ok() && !utmReady) {
        pubCheckReady.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    std::cout << "PATHPLANNER: Succesfully connected to UTM node" << std::endl;

    mainStatus = new std::string("NOT SET");
    currentTask = new std::string("NOT SET");
    rosMsg rosMsgObject(mainStatus, currentTask);

    if(cmdOptionExists(argv, argv+argc, "-z")) {
        std::cout << "PATHPLANNER: Running without pre-check for no flight zones!" << std::endl;
        rosMsgObject.setCheckZonesBeforeTakeoff(false);
    }

    while(ros::ok()) {
        rosMsgObject.checkForNewNoFlightZones();

        sendHeartBeat();

        ros::spinOnce();
        loopRate.sleep();
    }

    ros::spin();
}
