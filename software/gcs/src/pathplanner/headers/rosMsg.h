
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
#include <pathplanner/start_end_coord.h>
#include <pathplanner/flight_mission.h>
#include <pathplanner/blocked_goal.h>

#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_pos.h>

#include <utm/utm_no_flight_area.h>
#include <utm/utm_no_flight_circle.h>
#include <utm/utm_tracking_data.h>
#include <utm/utm_rally_point.h>
#include <utm/utm_rally_point_list.h>

#include <drone_decon/RedirectDrone.h>
#include <drone_decon/UTMDrone.h>
#include <drone_decon/UTMDroneList.h>

#include "headers/mapcontroller.h"
#include "headers/global/geofunctions.h"

struct DynamicNoFlightZone {
    int ID;
    int epochFrom;
    int epochTo;
};

class rosMsg
{
 public:
    rosMsg();
    rosMsg(std::string *main, std::string *current);
    ~rosMsg();

    void setCheckZonesBeforeTakeoff(bool val);
    void setStatusMessages(std::string *main, std::string *current);
    void isReady(const mavlink_lora::mavlink_lora_pos &msg);
    bool getIsReady();

    // UTM
    void getZonesFromUTM();
    void setNumberOfExpectedZones(const std_msgs::Int64 &msg);
    void addNoFlightCircle(const utm::utm_no_flight_circle &msg);
    void addNoFlightArea(const utm::utm_no_flight_area &msg);
    void rallyPointsForBlockedGoal(const utm::utm_rally_point_list &msg);
    void droneData(const drone_decon::UTMDroneList &msg);

    // Dronelink
    void setCurrentPosition(const mavlink_lora::mavlink_lora_pos &msg);

    // Mavlink_lora
    void getPath(const std_msgs::Bool &msg);
    void calculatePath();

    // Collision avoidance
    void gotRedirect(const drone_decon::RedirectDrone &msg);

    void lowBattery(const std_msgs::Bool &msg);

    void generateNewMap();

    void subStart();

    void checkForNewNoFlightZones();

private:
    bool checkIfZoneExists(DynamicNoFlightZone &zone);
    void publishBlockedGoal(int epochOver);

private:

    ros::NodeHandle n;

    ros::Publisher pubPath;
    ros::Publisher pubFetchNoFlightZones;
    ros::Publisher pubEmergency;
    ros::Publisher pubBlockedGoal;
    ros::Publisher pubFetchRallyPoints;
    ros::Publisher pubLandNow;
    ros::Publisher pubChangeGoal;

    ros::Subscriber subCurrentPosition;
    ros::Subscriber subGoalPosition;
    ros::Subscriber subCalculatePath;

    ros::Subscriber subNumberOfZones;
    ros::Subscriber subNoFlightCircles;
    ros::Subscriber subNoFlightAreas;
    ros::Subscriber subDeconflict;
    ros::Subscriber subRallyPoints;
    ros::Subscriber subDrones;
    ros::Subscriber subLowBattery;

    ros::Publisher pubIsReady;
    ros::Subscriber subIsReady;

    ros::Publisher droneRegisterPub;
    ros::Subscriber droneRedirectSub;

    std::vector<std::pair<double, double> > path;
    std::pair<double, double> initCoord;
    std::pair<double, double> currentCoord;
    std::pair<double, double> currentHeading;
    std::pair<double, double> goalCoord;
    int nodeDist;
    int mapWidth;
    int padLength;

    double altitude = 20;
    double currentActualAltitude = 0;

    MapController controller;

    bool solvingStarted = false;
    bool currentCoordSet = false;
    bool goalCoordSet = false;
    bool mapHasBeenGenerated = false;

    std::vector<DynamicNoFlightZone> dynamicZones;

    std::mutex zoneMutex;
    int numberOfExpectedZones = INT_MAX;
    int numberOfZonesReceived = 0;

    bool initialZonesLoaded = false;
    bool pathplannerReady = false;

    int expandZonesByMeters = 10;

    int epochBlockedUntil = INT_MAX;

    std::string *mainStatus;
    std::string *currentTask;

    bool checkZonesBeforeTakeoff = true;

    mavlink_lora::mavlink_lora_mission_list missionMsg;

    bool justGotRedirect = false;
    bool waitingForRallyPoints = false;

    drone_decon::UTMDroneList otherDrones;
};


#endif // ROSMSG_H
