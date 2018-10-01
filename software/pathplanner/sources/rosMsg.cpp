
#include "headers/rosMsg.h"




rosMsg::rosMsg()
{

};

void rosMsg::startCoor(const std_msgs::Int64& msg)
{
    ROS_INFO("start coordinate set to", msg-> data);


    map.startSolver();

    // ikke sikker paa hvordan jeg ros besked til coordinatet

};

void rosMsg::endCoor(const std_msgs::Int64& msg)
{
    ROS_INFO("end coordinate set to", msg-> data);


    map.setGoalPosition()
};


void rosMsg::subEnd()
{
        subEndCoor = n.subscribe("end coor topic",std_msgs::Int64, 1000, endCoor );
};


void rosMsg::subStart()
{
        subStartCoor = n.subscribe("start coor topic",std_msgs::Int64, 1000, startCoor );
};




void rosMsg::Publish()
{
    pub  = n.advertise<std_msgs::Int64>("path to goal", 1);

    pub.publish(map.getPathToDestination(path));


};









rosMsg::~rosMsg()
{

};
