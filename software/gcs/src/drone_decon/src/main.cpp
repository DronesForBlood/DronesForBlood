// INCLUDE STD
#include <iostream>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <math.h>
#include <locale>

// INCLUDE ROS
#include <ros/ros.h>
#include <ros/package.h>

// INCLUDE MESSAGES AND SERVICES
#include <drone_decon/GPS.h>
#include <drone_decon/UTMDrone.h>
#include <drone_decon/UTMDroneList.h>
#include <drone_decon/RedirectDrone.h>
#include <drone_decon/RegisterDrone.h>
#include <drone_decon/takeOffAndLandCheck.h>
#include <drone_decon/heartbeatDecon.h>

#include <lora_ground_control/heartbeat_node.h>

// INCLUDE OWN FILES
#include <DroneDeconflict.hpp>
#include <defines.hpp> // DEBUG is defined in here

using namespace std;
using namespace drone_decon;



#define DO_PREFLIGHT_CHECK false

#define MIN_HEIGHT_EVASION 10


// ############################ Clients, Subscribers and Publishers #######################
ros::Publisher Redirect_pub;

ros::Subscriber UTMDrone_sub;

ros::ServiceServer takeOff_Landing_server; 



// ############################# Global Variables ######################################
ros::NodeHandle* nh;

//node_monitor::heartbeat heartbeat_msg;

std::map<ID_t,simpleDrone> OtherDrones;
std::vector<ID_t> ourDrones;

bool newUTMdata = false;

#define HEARTBEAT_PERIOD 2
#define DRONE_DECON_ID 10
static std::string *mainStatus;
static std::string *currentTask;
static bool *hasError = new bool(false);
ros::Publisher pubHeartBeat;


// ################################ Misulanius ###########################################
ostream& operator<<(ostream& os, const GPS& pos)  
{  
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    os << "GPS(" << pos.latitude << ", " << pos.longitude << "), Alt(" << pos.altitude << ")";  
    return os;  
} 
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

// ############################ Service ##################################

bool cheack_TakeOff_Landing(takeOffAndLandCheckRequest &req, takeOffAndLandCheckResponse &res){




    return true;
}
// ############################### Message Handlers ###################################
void UTMdrone_Handler(UTMDroneList msg){
    for(size_t i = 0; i < msg.drone_list.size(); i++){
        UTMDrone* drone = &msg.drone_list[i];
        OtherDrones[drone->drone_id].update_values(*drone);
        if(DEBUG) cout << "UTM Drone Update: " << OtherDrones[drone->drone_id].getID() << endl;
    }
    newUTMdata = true;  
}


// ############################### Heartbeat ###################################

void sendHeartBeat(){
    lora_ground_control::heartbeat_node heartBeatMsg;
    heartBeatMsg.id = DRONE_DECON_ID;
    heartBeatMsg.name = "DroneDecon";
    heartBeatMsg.expected_interval = HEARTBEAT_PERIOD;
    heartBeatMsg.main_status = *mainStatus;
    heartBeatMsg.current_task = *currentTask;
    heartBeatMsg.has_error = *hasError;
    pubHeartBeat.publish(heartBeatMsg);
}



// ################################### Main Program ##########################################

int main(int argc, char** argv){
    ros::init(argc,argv,"drone_decon");
    nh = new ros::NodeHandle();
    UTMDrone_sub = nh->subscribe("/utm/dronesList",100,UTMdrone_Handler);
    Redirect_pub = nh->advertise<RedirectDrone>("/drone_decon/redirect",10);
    pubHeartBeat  = nh->advertise<lora_ground_control::heartbeat_node>("/lora_ground_control/heartbeat_nodes_rx", 1);
    std::cout << std::fixed;
    std::cout << std::setprecision(6);

    ourDrones.push_back(3013);

    mainStatus = new std::string("Running");
    currentTask = new std::string("Idle");

    int rate = 1;
    ros::Rate r(rate);
    while(ros::ok()){
        *currentTask = "Idle";
        ros::spinOnce();
        r.sleep();
        // ############### Drone deconfliction ################
        if(newUTMdata){
            *currentTask = "Checking for conflicts";
            newUTMdata = false;
            for(size_t i = 0; i < ourDrones.size();i++){
                ID_t ID = ourDrones[i];
                if(DEBUG) std::cout << "################################ Drone "<< ID << "###############################" << std::endl;
                simpleDrone ourDrone = OtherDrones[ID];
                vector<UTM> ourDronePath = ourDrone.getPath(simpleDroneDeconflict::maxSearchTime);
                if(DEBUG) std::cout << "Drone path size:" << ourDronePath.size() << endl;
                for (auto it = OtherDrones.begin(); it != OtherDrones.end(); it++ )
                { // first = key, second = data
                    if(it->first != ID){ // Make Sure it is not our Drone

                        if(DEBUG){ 
                            std::cout << "########### New detect ##########" << std::endl;
                            cout << "OurDrone   : " << ourDrone.getPositionU() << endl;
                            cout << "otherDrone : " << it->second.getPositionU() << endl;
                        }
                        simpleDroneDeconflict deCon(ourDrone,it->second,ourDronePath);
                        if(deCon.isSameHeight()){
                            if(DEBUG) std::cout << "Is same Height Area" << endl;

                            if(deCon.isWithinSeachArea()){
                                if(DEBUG) std::cout << "Is withing detection area" << endl;
                                //TODO assert that both Drones are in same UTM zone
                                if(deCon.crashDetected()){
                                    
                                    
                                    if(it->second.getPriority() <= ourDrone.getPriority()){ //Deconflict  ourDrone SAME Priority or less
                                        
                                        auto crash = deCon.getOurCrashSites();

                                        if(DEBUG){
					                        ofstream crashfile;
					                        crashfile.open ("crashfile.txt");
                                            cout << "CHRASHSITES" << endl;
                                            for(size_t i = 0; i < crash.size(); i ++){
						                        crashfile << setprecision(10) << UTM2GPS(crash[i]).latitude << ", " << UTM2GPS(crash[i]).longitude << ", " << 
							                    UTM2GPS(crash[i]).altitude << "\n";
                                                cout << UTM2GPS(crash[i]).latitude << ", " << UTM2GPS(crash[i]).longitude << endl;
                                            }
					                        crashfile.close();
                                        }
                                        

                                        GPS crashPos = UTM2GPS(crash[0]);

                                        //TODO check that min altitude is above ground
                                        double minAltitude = 0;
                                        double altCorrection = 0;
                                        if(crashPos.altitude-MIN_HEIGHT_EVASION < minAltitude){
                                            altCorrection = std::abs(crashPos.altitude-MIN_HEIGHT_EVASION-minAltitude);
                                        }
                                            
                                        if(ourDrone.getRawHeading() > it->second.getRawHeading()){
                                            crashPos.altitude+= (MIN_HEIGHT_EVASION+altCorrection);  
                                        }else if(ourDrone.getRawHeading() < it->second.getRawHeading()){
                                            crashPos.altitude-= (MIN_HEIGHT_EVASION-altCorrection);
                                        }else if(ourDrone.getID() > it->second.getID()){
                                            crashPos.altitude+= (MIN_HEIGHT_EVASION+altCorrection);
                                        }else{
                                            crashPos.altitude-= (MIN_HEIGHT_EVASION-altCorrection);
                                        }
                                        RedirectDrone msg;
                                        msg.drone_id = ID;
                                        msg.insertBeforeNextWayPoint = deCon.isOurCrashSitesBeforeWaypoint(0);
                                        msg.position = crashPos;
                                        Redirect_pub.publish(msg);
                                    }else{
                                        //If Our Drone Has Higher Priority
                                    }
                                }
                                if(DEBUG){
                                    ofstream drone1file;
                                    ofstream drone2file;	
                                    drone1file.open ("drone1file.txt");
                                    drone2file.open ("drone2file.txt");
   
                                    cout << "OurPositions" << endl;
                                    drone1file << setprecision(10) << OtherDrones[3013].getPosition().latitude << ", " << 						OtherDrones[3013].getPosition().longitude << ", " << OtherDrones[3013].getPosition().altitude << "\n";
                                    drone2file << setprecision(10) << OtherDrones[3012].getPosition().latitude << ", " << OtherDrones[3012].getPosition().longitude << ", " << OtherDrones[3012].getPosition().altitude << "\n";
                                    for(size_t i = 0; i < deCon.ourPositions.size(); i ++){
					                    drone1file << setprecision(10) << deCon.ourPositions[i].latitude <<  ", "<< deCon.ourPositions[i].longitude << ", " <<  							deCon.ourPositions[i].altitude << "\n";
                                        cout << deCon.ourPositions[i].latitude <<  ", "<< deCon.ourPositions[i].longitude << endl;
                                    }
				                    drone1file.close();
			
                                    cout << "OtherPositions" << endl;
                                    for(size_t i = 0; i < deCon.otherPositions.size(); i ++){
					                    drone2file << setprecision(10) << deCon.otherPositions[i].latitude <<  ", "<< deCon.otherPositions[i].longitude << ", " 							<<  deCon.otherPositions[i].altitude << "\n";
                                        cout << deCon.otherPositions[i].latitude <<  ", "<< deCon.otherPositions[i].longitude << endl;
                                    }
				                    drone2file.close();
                                }
                            }
                        }
                    }
                }
            }
        }
        sendHeartBeat();
    }

    delete nh;
}
