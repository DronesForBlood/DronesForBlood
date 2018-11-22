
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
//#include <std_msgs/String.h>
#include <PATHPLANNER/start_end_coord.h>
#include <PATHPLANNER/flight_mission.h>
#include <PATHPLANNER/request.h>
#include <PATHPLANNER/no_flight_circle.h>
#include <PATHPLANNER/no_flight_area.h>

#include "headers/mapcontroller.h"



class rosMsg
{
 public:
    rosMsg();
    ~rosMsg();

   void addNoFlightCircle(const PATHPLANNER::no_flight_circle &msg);
   void addNoFlightArea(const PATHPLANNER::no_flight_area &msg);
   void setupMap(const PATHPLANNER::start_end_coord &msg); //callackb
   void requestPath(const PATHPLANNER::request &msg);
   void subStart();
   void subEnd();


   void Publish();

private:


 private:

   ros::NodeHandle n;

   ros::Publisher pub;

   ros::Subscriber subMap;
   ros::Subscriber subRequestPath;
   ros::Subscriber subNoFlightCircles;
   ros::Subscriber subNoFlightAreas;

   std::vector<std::pair<double, double> > path;
   std::pair<double, double> startCoord;
   std::pair<double, double> endCoord;
   int nodeDist;
   int mapWidth;
   int padLength;

   MapController controller;

   bool solvingStarted = false;

};


#endif // ROSMSG_H
