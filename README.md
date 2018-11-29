# DronesForBlood
Repo for the groups documents and stuff for blood transport system for drones


# Installation instructions

1. Clone the develop branch to a local folder, as well as the required submodules, and checkout to the adequate Firmware branch:

    git clone -b develop https://github.com/DronesForBlood/DronesForBlood.git
    cd DronesForBlood
    git submodule update --init --recursive
    cd simulation/Firmware
    git checkout drones-for-blood

**NOTE:** The submodule instruction is assuming that you have a SSH key authentication enabled.

2. Install the Python dependencies
    - If a virtual enviroment is being used:
    
    ```bash
    cd <project-dir>
    pip3 install -r /<project-path>/requirements.txt 
    ```

3. Make and source the ROS project

    ```bash
    source /opt/ros/<ros-distro>/setup.bash
    cd <project-dir>/software/gcs
    catkin-make
    source ./devel/setup.bash
    ```
    

# Execution instructions

## Using Gazebo

There is a custom Gazebo world and launcher file prepared for using for simulating the drone. If Gazebo is installed, in one terminal run the following script:

    ```bash
    <path-to-main-dir>/simulation/launch_sim_sitl_gazebo.sh
    ```
    
Gazebo should be executed, and the iris drone with the IR lock, as well as the IR beacon, should be spawned in the world.

## Runing the GCS master, alongside the pathplanner, UTM, and the Mavlink Lora packages. UserLink is simulated

There is a main launch file that executes the required packages for doing an autonomous flight from A to B. The launch file is in the *gcs_master* package, and it is called *drones-for-blood.launch*. The launch file can be run with the following arugments:

- **takeoff-batt** Minimum battery level required for commanding a take off

- **critic-batt:** Threshold battery level. Below this point, a warning will be prompted. However, no special actions are performed yet when this situation happens

- **alt:** Set point altitude for the take-off and fly operations

- **sim:** If true, Mavlink will establish a communication with the Gazebo simulation, instead of doing it with the drone

Hence, if the user wants to make a simulated flight at 50 meters altitude, withe a minimum take-off battery of 80%, and a critical battery level of 20%, the following instruction has to be run:


    ```bash
    roslaunch gcs_master drones-for-blood.launch alt:=50 sim:=true takeoff-batt:=80 critic-batt:=20
    ```
    
**NOTE:** Some errors may be raised and the execution may fail if the system is launched before Gazebo or the actual drone is on and active.

