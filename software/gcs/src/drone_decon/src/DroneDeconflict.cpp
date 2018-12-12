#include <DroneDeconflict.hpp>
#include <iostream>
#include <math.h>
#include <cmath>
#include <ctime>
#include <ros/ros.h>

using namespace std;
#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

// This function returns distance in meters between two GPS coordinates
double GPSdistanceMeters(drone_decon::GPS pos1, drone_decon::GPS pos2) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(pos1.latitude);
  lon1r = deg2rad(pos1.longitude);
  lat2r = deg2rad(pos2.latitude);
  lon2r = deg2rad(pos2.longitude);
  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);
  return 1000*(2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)));
}

double UTMdistance(UTM pos1, UTM pos2){
    if(pos1.zone == pos2.zone){
        return std::sqrt(
            std::pow(pos1.east-pos2.east,2)+
            std::pow(pos1.north-pos2.north,2)
        );
    }else{
        //TODO
    }
}

drone_decon::GPS UTM2GPS(UTM &coord){
    drone_decon::GPS ret;
    Geo::UTMtoLL(coord.north,coord.east,coord.zone,ret.latitude,ret.longitude);
    ret.altitude=coord.altitude;
    return ret;

}
UTM GPS2UTM(drone_decon::GPS coord){
  UTM ret;
  Geo::LLtoUTM(coord.latitude,coord.longitude,ret.north,ret.east,ret.zone);
  ret.altitude = coord.altitude;
  return ret;
}

direction operator*(direction lhs, const double& rhs){
    lhs.east*=rhs;
    lhs.north*=rhs;
    return lhs;
}
UTM operator+(UTM lhs, const direction& rhs){
    lhs.east+=rhs.east;
    lhs.north+=rhs.north;
    return lhs;
}

UTM& UTM::operator +=(const direction& b){
    this->east+=b.east;
    this->north+=b.north;
    return *this;
}
point& point::operator =(const UTM& b){
    this->x = b.east;
    this->y = b.north;
    return *this;
}
point UTM2point(UTM pos){
    point one;
    one.x = pos.east;
    one.y = pos.north;
    return one;

}

ostream& operator<<(ostream& os, const UTM& pos)
{
    os << "East(" << long(pos.east) << "), North(" << long(pos.north) << "), Alt(" << pos.altitude << ")";
    return os;
}
ostream& operator<<(ostream& os, const point& pos)
{
    os << "point("<< long(pos.x) << ", " << long(pos.y) << ")";
    return os;
}
ostream& operator<<(ostream& os, const direction& pos)
{
    os << "direction("<< pos.east << ", " << pos.north <<")";
    return os;
}
ostream& operator<<(ostream& os, const line& pos)
{
    os << "line("<< "0="<< pos.a << "X + " << pos.b << "Y + " << pos.c <<")";
    return os;
}
ostream& operator<<(ostream& os, simpleDrone d){
    os << "--------- SimpleDrone " << d.getID() << "---------" <<endl;
    os << "Cur heading  : " << d.getCurHeading() << endl;
    os << "Next heading : " << d.getNextHeading() << endl;
    os << "Cur Position : " << d.getPositionU() << endl;
    os << "Next Position: " << d.getNextPositionU() << endl;
    os << "Cur Velocity : " << d.getCurVelocity() << endl;
    os << "Next Velocity: " << d.getNextVelocity() << endl;
    os << "Est Velocity : " << d.getEstimatedVelocity() << endl;
    os << "drone Time   : " << d.getTime() << endl;
    os << "ETA next WP  : " << d.getEtaNextWP() << endl;
    os << "Priority     : " << int(d.getPriority()) << endl;
    os << "Battery SOC  : " << d.getBatterySOC() << endl;
    return os;
}

//################### SimpleDrone ######################
simpleDrone::simpleDrone():
    vel_list(LIST_SIZE),gps_time_list(LIST_SIZE),cur_pos_list(LIST_SIZE){}
simpleDrone::simpleDrone(drone_decon::UTMDrone info):
    vel_list(LIST_SIZE),gps_time_list(LIST_SIZE),cur_pos_list(LIST_SIZE)
{
    this->update_values(info);
}
void simpleDrone::update_values(drone_decon::UTMDrone info){
    
    
    //Essential values
    this->drone_priority = info.drone_priority;
    this->gps_time = info.gps_time;
    this->time = info.time;
    this->cur_pos = info.cur_pos;
    if(this->drone_id == 0){
        for(size_t i = 0; i < gps_time_list.size();i++) gps_time_list[i]= this->gps_time;
        for(size_t i = 0; i < cur_pos_list.size();i++) cur_pos_list[i]= this->cur_pos;
    }else{
        this->cur_pos_list.push_back(this->cur_pos);
        this->cur_pos_list.pop_front();
        this->gps_time_list.push_back(this->gps_time);
        this->gps_time_list.pop_front();
    }

    //Update Current Heading
    if(info.cur_heading != -1){
        while(info.cur_heading > 360) info.cur_heading -= 360;
        while(info.cur_heading<0 ) info.cur_heading +=360;
        while(info.next_heading > 360) info.next_heading -= 360;
        while(info.next_heading<0 ) info.next_heading +=360;
        this->cur_heading = info.cur_heading;
    }else{
        this->cur_heading = 360;
    }

    //Update Next Heading
    if(info.next_heading != -1){
        this->next_heading = info.next_heading;
    }else{
        this->next_heading = this->cur_heading;
    }

    bool invalid_next_WP = info.ETA_next_WP == -1;
    


    //Update Current Velocity
    if(std::abs(info.cur_vel) > 0.00001 && std::abs(info.cur_vel) < 50){
        this->cur_vel = info.cur_vel;
    }else{
        long t = std::abs(gps_time_list.front()-gps_time_list.back());
        if(t == 0){
            this->cur_vel = 0;
        }else{
            this->cur_vel = GPSdistanceMeters(this->cur_pos_list.front(),this->cur_pos_list.back())/t;
        }
        
        info.ETA_next_WP = -1;
    }
    if(this->cur_vel > 50){
        cout << "drone id: " << this->drone_id << "is moving at: " << this->cur_vel << "m/s" << endl;
        ROS_ERROR("Velocity is too high truncated to 5 m/s");
        this->cur_vel = 5;
        info.ETA_next_WP = -1;
    }

    //Velocity Estimate
    if(this->drone_id == 0){
        vel_acc = 0;
        for(size_t i = 0; i < vel_list.size();i++){
            vel_list[i]= this->cur_vel;
            vel_acc+=this->cur_vel;
        }     
    }else{
        this->vel_list.push_back(this->cur_vel);
        this->vel_acc += this->cur_vel - this->vel_list.front();

        this->vel_list.pop_front();
    }
    this->cur_vel_est = this->vel_acc/this->vel_list.size();
    
    if( this->cur_vel_est > 50 || this->cur_vel_est < 0){
        cout << "curvel: " << this->cur_vel << endl;
        cout << "velest: " << this->cur_vel_est << endl;
        cout << "velacc: " << this->vel_acc << endl;
        cout << "vellist: ";
        for(size_t i = 0; i < vel_list.size();i++){
            cout << vel_list[i] << ", ";
        }
        cout << endl;
        if(this->cur_vel_est > 50){
            ROS_ERROR("Estimated Velocity To HIGH, using current velocity");
        }else{
            ROS_ERROR("Estimated Velocity To LOW, using current velocity");
        }
        this->cur_vel_est = this->cur_vel;

    }

    //Update Next Waypoint
    if(!((info.next_WP.longitude == -1 &&
        info.next_WP.latitude == -1 &&
        info.next_WP.altitude == -1) || 
        invalid_next_WP))
    {
        this->next_wp = info.next_WP;
    }else{
        UTM cur = this->getPositionU();
        cur += this->getCurHeading()*this->getEstimatedVelocity()*20;
        this->next_wp = UTM2GPS(cur);
        info.ETA_next_WP = -1;
    }

    if(info.next_vel != -1){
        this->next_vel = info.next_vel;
    }else{
        this->next_vel = cur_vel_est;
    }

    if(info.ETA_next_WP != -1){
        this->ETA_next_WP= info.ETA_next_WP;
    }else{
        this->ETA_next_WP= GPSdistanceMeters(this->cur_pos_list.front(),this->cur_pos_list.back())*this->cur_vel_est;
    }

    this->battery_soc = info.battery_soc;




    //Mmust be innitialized last so it can be detected if this is first Update
    this->drone_id = info.drone_id;
    if(DEBUG) cout << *this << endl;
}
drone_decon::GPS simpleDrone::getPosition(){return this->cur_pos;}
UTM simpleDrone::getPositionU(){return GPS2UTM(this->cur_pos);}
UTM simpleDrone::getNextPositionU(){return GPS2UTM(this->next_wp);}
direction simpleDrone::getHeading(double heading){
    heading = M_PI * heading/180;
    direction ret;
    ret.north = std::cos(heading);
    ret.east = std::sin(heading);

    return ret;
}
float simpleDrone::getRawHeading(){return this->cur_heading;}
direction simpleDrone::getCurHeading(){return getHeading(this->cur_heading);}
direction simpleDrone::getNextHeading(){return getHeading(this->next_heading);}
double simpleDrone::getCurVelocity(){return this->cur_vel;}
double simpleDrone::getEstimatedVelocity(){return this->cur_vel_est;}
double simpleDrone::getNextVelocity(){return this->next_vel;}
ID_t simpleDrone::getID(){return this->drone_id;}
long simpleDrone::getTime(){
    if(this->gps_time > 150000) return this->gps_time;  
    if(this->time > 150000) return this->time;
    return std::time(nullptr);
}
double simpleDrone::getEtaNextWP(){return this->ETA_next_WP;}
double simpleDrone::getBatterySOC(){return this->battery_soc;}
uint8_t simpleDrone::getPriority(){return this->drone_priority;}
vector<UTM> simpleDrone::getPath(double time,double distance_step){
    if(DEBUG) cout << "################# Path Teselation ##############" << endl;
    if(DEBUG) cout << *this << endl;
    UTM curPos = this->getPositionU();
    UTM nextPos = this->getNextPositionU();

    time+= std::abs(std::time(nullptr)-this->getTime());
    if(DEBUG){
        cout << "Path calculate forward in time by: " << time << "s " << endl;
        cout << "distance step: " << distance_step << endl;
        cout << "current velocity: " << this->cur_vel_est << endl;
    }

    double timeStep = distance_step/this->cur_vel_est;

    vector<UTM> path;
    if(std::abs(this->cur_vel_est) < 0.1 || std::abs(timeStep) <0.05){
        path.push_back(curPos);
        path.push_back(curPos);

        if(std::abs(timeStep) <0.05){
            cout << "distance step: " << distance_step << endl;
            cout << "current velocity: " << this->cur_vel_est << endl;
            throw "Vary Small time Step";
        }
    }else{
        if(DEBUG) cout << "Time step: "  << timeStep << endl;
        if(timeStep > time/60){
            timeStep = time/60;
            distance_step = timeStep*this->cur_vel_est;
            if(DEBUG) cout << "Time step: "  << timeStep << endl;
            
        }
        if(std::abs(timeStep) <0.05){
            cout << "TimeStep 0" << endl;
            throw "Vary Small time Step";
        };
            


        direction step = this->getCurHeading()*distance_step;
        if(DEBUG) cout << "Direction step: " << step << endl;

        
        double tNow = 0;
        while(UTMdistance(curPos,nextPos)>distance_step){
            path.push_back(curPos);
            curPos+=step;
            tNow+=timeStep;
            if(tNow>time) break; 
        }
        if(tNow < time){
            int loops = 0;
            step = this->getNextHeading()*distance_step;
            while(tNow<time){
                path.push_back(curPos);
                curPos+=step;
                tNow+=timeStep;
                if(tNow<0){
                    ROS_ERROR("tNow is less the 0");
                    cout << "timeStep     : " << timeStep << endl;
                    cout << "distance_step: " << distance_step << endl;
                    cout << "vel_est      : " << this->cur_vel_est << endl;
                    break;
                }
                if(loops++ > 1000){
                    ROS_ERROR("Force quitting while loop, search time too big");
                    cout << "time       : " << time << endl;
                    cout << "SysTime    : " << std::time(nullptr) << endl; 
                    cout << "DroneTime  : " << this->getTime() << endl;
                    break;
                }

            }
        }
    }
    return path;

}



//################### simpleDroneDeconflict ############################
simpleDroneDeconflict::simpleDroneDeconflict(simpleDrone &ourDrone, simpleDrone &otherDrone, std::vector<UTM> &ourDronePath):
    otherDrone(otherDrone),ourDrone(ourDrone),ourDronePath(ourDronePath){
    this->ourSearchTime = maxSearchTime+ time(nullptr)-ourDrone.getTime();
    this->otherSearchTime = maxSearchTime+ time(nullptr)-otherDrone.getTime();
}
bool simpleDroneDeconflict::isSameHeight(){
    UTM otherPos = this->otherDrone.getPositionU();
    UTM otherPosNext = this->otherDrone.getNextPositionU();
    UTM ourPos = this->ourDrone.getPositionU();
    UTM ourPosNext = this->ourDrone.getNextPositionU();

    //TODO in between start.z and end.z
    return std::abs(otherPos.altitude-ourPos.altitude)<minAltDistance||
        std::abs(otherPosNext.altitude-ourPos.altitude) < minAltDistance||
        std::abs(otherPos.altitude-ourPosNext.altitude) < minAltDistance||
        std::abs(otherPosNext.altitude-ourPosNext.altitude) < minAltDistance;
}
bool simpleDroneDeconflict::isWithinSeachArea(){

    double maxSearchRadius = saftyMargin*(
                                this->otherSearchTime*this->otherDrone.getEstimatedVelocity()
                                + this->ourSearchTime*this->ourDrone.getEstimatedVelocity()
                                );
    return GPSdistanceMeters(this->otherDrone.getPosition(),this->ourDrone.getPosition())<maxSearchRadius;
}
point simpleDroneDeconflict::pointOfCollision(direction heading1, UTM pos1, direction heading2, UTM pos2){
    double a1 = heading1.north/heading1.east;
    double a2 = heading2.north/heading2.east;
    double b1 = pos1.north-a1*pos1.east;
    double b2 = pos2.north-a2*pos2.east;

    point result;
    result.x = (b1-b2)/(a1-a2);
    result.y = a1*result.x+b1;

    return result;
}
double simpleDroneDeconflict::time2point(point goal, direction heading, double velocity, UTM start){
    direction V = heading;
    V.east*=velocity;
    V.north*=velocity;

    double t1= (goal.x-start.east)/(V.east);
    double t2= (goal.y-start.north)/(V.north);
    if(std::isinf(t1)){
        t1 = -1;
    }
    if(std::isinf(t2)){
        t2= -1;
    }
    if(t1 == 0 && V.east < 0.000000001){
        t1=t2;
    }else if(t2 == 0 && V.north < 0.000000001){
        t2 = t1;
    }
    

    if(std::abs(t1-t2)>2){
        std::cout << "ERROR point not on line - tdif =" << std::abs(t1-t2) << std::endl;
        std::cout << "Calculating time from : " << start << endl;
        std::cout << "                   to : " << goal << endl;
        std::cout << "Using velocity        : " << V << endl;
        std::cout << "ETA1: " << t1 << " - ETA2: " << t2 << " - ETA: " << (t1+t2)/2 << endl;
        throw "[time2point] error to large time diff";
    }
        
    //}

     return (t1+t2)/2;
}
line simpleDroneDeconflict::getLine(direction heading, UTM pos){
    line result;
    result.a = heading.north/heading.east;
    result.c = pos.north-result.a*pos.east;
    result.b = -1;

    return result;
}
double simpleDroneDeconflict::line2pointDistance(line theLine,UTM pos){
    double top = std::abs(  theLine.a*pos.east+
                            theLine.b*pos.north+
                            theLine.c);
    double btn = std::sqrt( std::pow(theLine.a,2)+
                            std::pow(theLine.b,2));
    return top/btn;
}
point simpleDroneDeconflict::line2pointPoint(line theLine,UTM pos){
    point result;
    double top = theLine.b*(theLine.b*pos.east-theLine.a*pos.north)-theLine.a*theLine.c;
    double btn = std::pow(theLine.a,2)+std::pow(theLine.b,2);

    result.x = top/btn;
    top = top = theLine.a*(-theLine.b*pos.east+theLine.a*pos.north)-theLine.b*theLine.c;
    result.y = top/btn;

    return result;
}
bool simpleDroneDeconflict::crashDetected(){
    if(DEBUG) cout << "############## new crash detect #################" << endl;
    bool crashIsDetected = false;
    line firstPart = getLine(otherDrone.getCurHeading(),otherDrone.getPositionU());
    if(DEBUG) cout << "The Line: " << firstPart << endl;
    double ourTime = ourDrone.getTime();
    if(DEBUG) cout << "OurDrone Estimated velocity used in crash Detect" << ourDrone.getEstimatedVelocity();
    double ourTimeStep;
    try{
        ourTimeStep = UTMdistance(ourDronePath[0],ourDronePath[1])/ourDrone.getEstimatedVelocity();
    }catch(...){
        cout << "Point0: " << ourDronePath[0] << endl;
        cout << "Point1: " << ourDronePath[1] << endl;
        cout << "Speed : " << ourDrone.getEstimatedVelocity() << endl;
        throw "FIX Error line 439";
    }
    if(ourTimeStep == 0) ourTimeStep = this->ourSearchTime;
    bool nextWpReached = false;
    for(size_t i = 0; i < ourDronePath.size(); i++){

        //##################### FOR DEBUG PURPOSES ##############################################
        if(DEBUG){
            this->ourPositions.push_back(UTM2GPS(ourDronePath[i]));
            point a = line2pointPoint(firstPart,ourDronePath[i]);
            UTM aPoint = ourDronePath[i];
            aPoint.north = a.y;
            aPoint.east = a.x;
            double tCol = time2point(   a,
                                            otherDrone.getCurHeading(),
                                            otherDrone.getEstimatedVelocity(),
                                            otherDrone.getPositionU());
            /*cout << "############## DEBUG OUT ##############" << endl;
            cout << "tCol: " << tCol << endl;
            cout << "pos : " << UTM2GPS(aPoint) << endl;*/

            if(tCol>0){                                
                this->otherPositions.push_back(UTM2GPS(aPoint));
            }
        }
        //######################### REAL CODE ############################################
        double dist = line2pointDistance(firstPart,ourDronePath[i]);
        if(DEBUG) cout << "Distance between drones at: "  << UTM2point(ourDronePath[i]) << " and " << line2pointPoint(firstPart,ourDronePath[i]) << " = " << dist << endl;
        if(dist < this->minRadius*this->saftyMargin){
            if(DEBUG) cout << "Drones within collision radius" << endl;
            //########### THIS CODE IS IS ALSO RUN IN THE DEBUG CODE ABOVE ##################
            point collision = line2pointPoint(firstPart,ourDronePath[i]);
            double tCol = time2point(   collision,
                                        otherDrone.getCurHeading(),
                                        otherDrone.getEstimatedVelocity(),
                                        otherDrone.getPositionU());
            if(DEBUG) cout << "Time difference between drone visit: " << std::abs(tCol+otherDrone.getTime()-ourTime) << endl;
            if(tCol+otherDrone.getTime()>otherDrone.getEtaNextWP()){
                break;
            }
            else if (std::abs(tCol+otherDrone.getTime()-ourTime)<this->minTimeBetween*this->saftyMargin &&
                     tCol > 0)
            {
                if(DEBUG) cout << "Drones Are within collision time" << endl;
                double difHeight = otherDrone.getNextPositionU().altitude-otherDrone.getPositionU().altitude;
                double altitude =   difHeight*
                                    (tCol/(otherDrone.getEtaNextWP()-otherDrone.getTime()))+
                                    otherDrone.getPositionU().altitude;
                if(DEBUG) cout << "Cheacking altitude difference at collision: " << std::abs(altitude-ourDronePath[i].altitude) << endl;
                if(std::abs(altitude-ourDronePath[i].altitude)<this->minAltDistance*this->saftyMargin){
                    if(DEBUG) cout << "Collision detected at:" <<  ourDronePath[i] << endl;
                    crashIsDetected = true;
                    ourCrashSites.push_back(ourDronePath[i]);
                    isOurCrashSitesBeforeWaypointList.push_back(true);
                    UTM crashOther;
                    crashOther.altitude = altitude;
                    crashOther.east = collision.x;
                    crashOther.north = collision.y;
                    crashOther.zone = otherDrone.getPositionU().zone;
                    otherCrashSites.push_back(crashOther);

                }



            }

        }
        ourTime += ourTimeStep;
    }
    // REMOVE LATER
    return crashIsDetected;

    line secondPart = getLine(otherDrone.getNextHeading(),otherDrone.getNextPositionU());
    ourTime = ourDrone.getTime();
    for(size_t i = 0; i < ourDronePath.size(); i++){
        this->ourPositions.push_back(UTM2GPS(ourDronePath[i]));
        point a = line2pointPoint(secondPart,ourDronePath[i]);
        UTM aPoint = ourDronePath[i];
        aPoint.north = a.y;
        aPoint.east = a.x;
        this->otherPositions.push_back(UTM2GPS(aPoint));
        double dist = line2pointDistance(secondPart,ourDronePath[i]);
        if(DEBUG) cout << "Distance between drones at: "  << UTM2point(ourDronePath[i]) << " and " << line2pointPoint(firstPart,ourDronePath[i]) << " = " << dist << endl;
        if(dist < this->minRadius*this->saftyMargin){
            if(DEBUG) cout << "Drones within collision radius" << endl;
            point collision = line2pointPoint(secondPart,ourDronePath[i]);
            double tCol = time2point(   collision,
                                        otherDrone.getNextHeading(),
                                        otherDrone.getEstimatedVelocity(),
                                        otherDrone.getNextPositionU());
            if(DEBUG) cout << "Time difference between drone visit: " << std::abs(tCol+otherDrone.getTime()-ourTime) << endl;
            if(tCol+otherDrone.getTime()>otherSearchTime+std::time(nullptr)){
                if(DEBUG) cout << "Collision outside search time" << endl;
                break;
            }else if (std::abs(tCol+otherDrone.getTime()-ourTime)<this->minTimeBetween &&
                      tCol > 0)
            {
                if(DEBUG) cout << "Drones Are within collision time" << endl;
                double altitude = otherDrone.getNextPositionU().altitude;
                if(std::abs(altitude-ourDronePath[i].altitude)<this->minAltDistance){
                    if(DEBUG) cout << "Collision detected at:" <<  ourDronePath[i] << endl;
                    crashIsDetected = true;
                    ourCrashSites.push_back(ourDronePath[i]);
                    isOurCrashSitesBeforeWaypointList.push_back(false);
                    UTM crashOther;
                    crashOther.altitude = altitude;
                    crashOther.east = collision.x;
                    crashOther.north = collision.y;
                    crashOther.zone = otherDrone.getNextPositionU().zone;
                    otherCrashSites.push_back(crashOther);
                }
            }
        }
        ourTime += ourTimeStep;
    }
    return crashIsDetected;
}
bool simpleDroneDeconflict::takeOffCrashDetect(){

}
bool simpleDroneDeconflict::isOurCrashSitesBeforeWaypoint(size_t index){
    return this->isOurCrashSitesBeforeWaypointList[index];
}

std::vector<UTM> simpleDroneDeconflict::getOurCrashSites(){
    return this->ourCrashSites;
}
std::vector<UTM> simpleDroneDeconflict::getOtherCrashSites(){
    return this->otherCrashSites;
}

point UTM2LL(point utmCoord)
{
    std::string zone = "32U";
    point pointLL;
    double lat;
    double lon;

    Geo::UTMtoLL(utmCoord.y, utmCoord.x, zone, lat, lon);
    pointLL.x = lat;
    pointLL.y = lon;
    return pointLL;
}
