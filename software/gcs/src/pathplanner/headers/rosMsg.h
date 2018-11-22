
#ifndef ROSMSG_H
#define ROSMSG_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/builtin_int64.h>
#include <std_msgs/Bool.h>
//#include <std_msgs/String.h>
#include <PATHPLANNER/start_end_coord.h>
#include <PATHPLANNER/flight_mission.h>
#include <PATHPLANNER/request.h>
#include <PATHPLANNER/no_flight_circle.h>
#include <PATHPLANNER/no_flight_area.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>

#include "headers/mapcontroller.h"



class rosMsg
{
 public:
    rosMsg();
    ~rosMsg();

    // UTM
    void addNoFlightCircle(const PATHPLANNER::no_flight_circle &msg);
    void addNoFlightArea(const PATHPLANNER::no_flight_area &msg);

    // Dronelink
    void setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg);

    // User interface
    void setGoalPosition(const mavlink_lora::mavlink_lora_pos &msg);

    // Mavlink_lora
    void calculatePath(const std_msgs::Bool &msg);

    void generateNewMap();

    void subStart();

private:

    ros::NodeHandle n;

    ros::Publisher pubPath;

    ros::Subscriber subCurrentPosition;
    ros::Subscriber subGoalPosition;
    ros::Subscriber subCalculatePath;

    ros::Subscriber subNoFlightCircles;
    ros::Subscriber subNoFlightAreas;

    std::vector<std::pair<double, double> > path;
    std::pair<double, double> currentCoord;
    std::pair<double, double> goalCoord;
    int nodeDist;
    int mapWidth;
    int padLength;

    int altitude = 20;

    MapController controller;

    bool solvingStarted = false;
    bool currentCoordSet = false;
    bool goalCoordSet = false;

};


#endif // ROSMSG_H
