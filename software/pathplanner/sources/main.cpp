
<<<<<<< HEAD

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

    ros::spin();
=======
#include <iostream>

#include "headers/simulation.h"

int main() {
    Simulation simulation;
>>>>>>> develop
}
