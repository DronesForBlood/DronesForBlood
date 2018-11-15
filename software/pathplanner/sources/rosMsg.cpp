
#include "headers/rosMsg.h"



 // start/end coor --> double floats
// dsit between  nodes   -->  int
// search area --> two ints
rosMsg::rosMsg()
{

};

void rosMsg::coor(const std_msgs::vector& msg)
{
    ROS_INFO("start coordinate set to", msg-> data);



   startCoord = msg[1];
   endCoord = (msg[0]);

    // ikke sikker paa hvordan jeg ros besked til coordinatet

};

void rosMsg::mapParam(const std_msgs::vector& msg)  //dist between search nodes and search area size
{
    ROS_INFO("end coordinate set to", msg-> data);

    nodeDist = msg[0];
    mapWidth =  msg[1];
    padLength = msg[2];



};


void rosMsg::coordinates()
{
        Coor = n.subscribe("coordinates",std_msgs::vector, 1000, coor );
};


void rosMsg::subStart()
{
        mapParam = n.subscribe("mapParam",std_msgs::vector, 1000, mapParam );
};




void rosMsg::Publish()
{
    pub  = n.advertise<std_msgs::Int64>("path to goal", 1);


    pub.publish(map.getPathToDestination(path));


};









rosMsg::~rosMsg()
{

};
