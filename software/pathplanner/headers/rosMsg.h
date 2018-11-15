
#ifndef SIMULATION_H
#define SIMULATION_H

#include <utility>
#include <vector>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/builtin_int64.h>
#include <std_msgs/String.h>


#include "headers/mapcontroller.h"



class rosMsg
{
 public:
   void Publish();
   void callback(const SubscribedTO& input);
   void startCoor(const std_msgs::Int64& msg); //callack
   void endCoor(const std_msgs::Int64& msg);  //callback
   void subStart();
   void subEnd();


 private:

   ros::NodeHandle n;
   ros::Publisher pub;
   ros::Subscriber subStartCoor;
   ros::Subscriber subEndCoor;
   std_msgs::Int64 msg;
   std::vector<std::pair<double, double> > path;
   std::pair<double, double> startCoord;
   std::pair<double, double> endCoord;
   MapController map;
   int nodeDist;
   int mapWidth;
   int padLenght;










};





#endif // SIMULATION_H
