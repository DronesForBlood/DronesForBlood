# DronesForBlood
Repo for the groups documents and stuff for blood transport system for drones

# Dependencies
- Mavlink_lora (Is not public at this moment, but take contact to Crowdedlight if interested in using it, and he can relay request to owner of mavlink_lora)

# Installation instructions

1. Clone the develop branch to a local folder, as well as the required submodules, and checkout to the adequate Firmware branch. Finally, compile the Gazebo SITL project:

```bash
    git clone -b develop https://github.com/DronesForBlood/DronesForBlood.git
    cd DronesForBlood
    git submodule update --init --recursive
    cd simulation/Firmware
    git checkout drones-for-blood
    make posix_sitl_default gazebo
```


**NOTE:** The submodule instruction is assuming that you have a SSH key authentication enabled.

2. Install the Python and system dependencies
    - If a virtual enviroment is being used:
    
    ```bash
    sudo apt install qtlocation5-dev qt5positioning-dev
    cd <project-dir>
    pip3 install -r /<project-path>/software/gcs/requirements.txt 
    ```

3. Make and source the ROS project

    ```bash
    source /opt/ros/<ros-distro>/setup.bash
    cd <project-dir>/software/gcs
    catkin_make
    source ./devel/setup.bash
    ```

# Execution instructions

## Using Gazebo

There is a custom Gazebo world and launcher file prepared for using for simulating the drone. If Gazebo is installed, in one terminal run the following script:

```bash
/<path-to-main-dir>/simulation/launch_sim_sitl_gazebo.sh
```
    
Gazebo should be executed, and the iris drone with the IR lock, as well as the IR beacon, should be spawned in the world.

## Runing the GCS master, alongside the pathplanner, UTM, and the Mavlink Lora packages. UserLink is simulated

There is a main launch file that executes the required packages for doing an autonomous flight from A to B. The launch file is in the *gcs_master* package, and it is called *drones-for-blood.launch*. The launch file can be run with the following arugments:

- **takeoff-batt** Minimum battery level required for commanding a take off

- **critic-batt:** Threshold battery level. Below this point, a warning will be prompted. However, no special actions are performed yet when this situation happens

- **alt:** Set point altitude for the take-off and fly operations

- **sim:** If true, Mavlink will establish a communication with the Gazebo simulation, instead of doing it with the drone

- **flyzones:** If false, the restricted fly zones will be ignored when taking off, so the flight/simulation is faster (suitable for debugging).

Hence, if the user wants to make a simulated flight at 50 meters altitude, withe a minimum take-off battery of 80%, and a critical battery level of 20%, the following instruction has to be run:


```bash
roslaunch gcs_master drones-for-blood.launch alt:=50 sim:=true flyzones:=false takeoff-batt:=80 critic-batt:=20
```
    
   
**NOTE:** Some errors may be raised and the execution may fail if the system is launched before Gazebo or the actual drone is on and active.


## Running LoraGroundControl
The LoraGroundControl is dependent on Mavlink_Lora which publishes the package information on ros topics. Without a running instance 
of mavlink_lora the gui will not update or show any information. 

Launch the groundcontrol monitoring with:
```
roslaunch lora_ground_control groundcontrol.launch
```

**Note:** if the roscore is getting closed while LoraGroundControl is running, and a new roscore is started, then 
it is nessecary to restart LoraGroundControl so it can reconnect its listeners on the new roscore. 
# Troubleshooting

- __ModuleNotFoundError: No module named 'urllib2'__: when the pykml library is imported, it can not find the urrlib2 library. This is a bug in the _python3_ version of pykml.  As stated in [this blog](http://installfights.blogspot.com/2018/04/how-to-run-pykml-in-python3.html), the source code of the _pykml_ _parser.py_ module has to be editted:

    - With virtual environments, the file is at _~/.virtualenvs/<virtualenv-name>/lib/python3.6/site-packages/pykml/_
    
    - If not, it is in the system libraries folder e.g. _~/.local/lib/python3.6/site-packages/pykml/_
    
In the file, the line that says `import urrlib2` has to be changed by the following lines:

```python
try:
    # For Python 3.0 and later
    from urllib.request import urlopen
except ImportError:
    # Fall back to Python 2's urllib2
    from urllib2 import urlopen
```
    
- __[ERROR] [1543662066.324335031]: Unable to open port__: Unable to open tty v6 port when running mavlink_lora. Check if the socat is installed, and then run the commands from the launch_sim_sitl_gazebo.sh files again.

    ```bash
    sudo apt install socat
    ```
