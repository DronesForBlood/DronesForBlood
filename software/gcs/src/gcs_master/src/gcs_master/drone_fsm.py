#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries
# Third-party libraries
import numpy as np
import rospy
import std_msgs.msg
# Local libraries
from gcs_master import path_operations


class DroneFSM():

    PLANNER_TIMEOUT = 10     # Timeout, in seconds, for asking again for a path
    TIMEOUT = 5             # Timeout, in seconds, for asking again for commands
    MISSION_LENGTH = 4      # Number of waypoints sent to the drone

    def __init__(self, max_lowbatt_distance=100, takeoff_altitude=50,
                 takeoff_batt=80, fly_batt=30, hovering_time=10):
        """
        :param int max_lowbatt_distance: maximum distance in meters that
         can be covered by the drone after entering low battery mode.
        """
        # Flight constant
        self.TAKEOFF_ALTITUDE = takeoff_altitude
        self.MIN_TAKEOFF_BAT = takeoff_batt # Min battery level for taking off
        self.MIN_FLY_BAT = fly_batt         # Min battery level for flying
        self.HOVERING_TIME = hovering_time  # Hover time before starting landing
        # Threshold distances
        self.new_waypoint_distance = 5      # Distance for getting new waypoint
        self.destination_threshold_dist = 0.000006   # Dist. for start landing
        # FSM flags. Outputs
        self.ARM = False
        self.ASK_DOCKING = False
        self.TAKE_OFF = False
        self.LAND = False
        self.CLEAR_MISSION = False
        self.ACTIVATE_PLANNER = False
        self.CALCULATE_PATH = False
        self.UPLOAD_MISSION = False
        self.START_MISSION = False
        self.HOLD_POSITION = False
        self.CLEAR_MISSION = False
        self.EMERGENCY_LANDING = False
        # Drone parameters. FSM inputs
        self.altitude = None                # Absolute altitude
        self.relative_alt = None            # Relative altitude
        self.latitude = None                # Drone latitude
        self.longitude = None               # Drone longitude
        self.heading = None                 # Heading, in degrees
        self.cur_speed = 0                  # Drone current ground speed
        self.position = [None, None]        # Current position
        self.destination = [None, None]     # Destination location
        self.home_pos = [None, None]        # Home position
        self.route = []                     # List of all waypoints
        self.current_path = []              # List of immediate waypoints
        self.distance_to_dest = 0           # Distance to the final destination
        self.armed = False                  # Armed / Disarmed
        self.docking = False                # Docking station allows the takeoff
        self.taking_off = False             # Drone on taking-off operation
        self.landing = False                # Drone is landing
        self.holding_position = False       # Drone is holding position on air
        self.mode_updated = False           # The drone has updated the mode
        self.batt_level = 0                 # Reamining battery capacity
        self.takeoff_batt_ok = False        # Enough battery for taking off
        self.mission_cleared = False
        self.batt_ok = False                # Enough battery for flying
        self.comm_ok = False                # Comlink status
        self.planner_ready = False          # Pathplanner status
        self.gps_ok = False                 # GPS data is being received
        self.dest_unreachable = False       # UTM no-fly zone over destination
        self.emergency_stop = False         # Path planner requires to stop
        # User link related variables
        self.new_mission = False            # The user has requested new mission
        # Path related variables
        self.new_path = False               # If new path is available
        self.new_waypoint = False           # New waypoint is available
        self.mission_ready = False          # The drone received a mission
        self.mission_started = False        # The drone has started the mission
        self.current_mission = 0
        self.max_lowbatt_distance = max_lowbatt_distance
        # FSM parameters
        self.__state = "start"
        self.__state_timer = 0

    def update_fsm(self):
        """
        Update the state of the FSM, and the outputs afterwards.
        """
        self.update_state()
        self.update_outputs()
        return

    def update_state(self):
        """
        Update the state of the FSM, based on the state variables.
        """
        # TODO:Check for errors first!
        # if self.msg._connection_header["topic"] == "/mavlink/drone/error":
        #     if self.msg == "BATTERY_ERROR":                                         # Battery error
        #         self.__state = "emergency_landing"

        #     elif self.msg == "RC_LINK_LOSS":                                        # Commlink error
        #         self.__state = "recover_comm"

        # START state. Wait until a new operation is requested.
        if self.__state == "start":
            if (self.new_mission and self.comm_ok and self.takeoff_batt_ok
                    and self.gps_ok and self.docking):
                self.__state = "planner_setup"
                self.new_mission = False
                self.mission_ready = False
                self.docking = False
                self.state_to_log()
                # Restart timer for the new state, so it updates the flags asap.
                self.__state_timer = -100.0

        # PLANNER SETUP state. Wait until the path planner acknowledges that
        # it is ready for creating mission plans.
        elif self.__state == "planner_setup":
            if (self.planner_ready and self.comm_ok and self.takeoff_batt_ok
                    and self.mission_cleared):
                self.__state = "arm"
                self.mission_cleared = False
                self.planner_ready = False
                self.state_to_log()
                # Restart timer for the new state, so it updates the flags asap.
                self.__state_timer = 0.0

        # ARM state. Wait until mavlink acknowledges the drone implemented
        # the take-off command.
        elif self.__state == "arm":	
            if self.armed and self.taking_off:
                self.__state = "take_off"
                self.new_mission = False
                self.armed = False
                self.state_to_log()
                # Restart timer for the new state.
                self.__state_timer = 0.0

        # TAKE OFF state. Wait until mavlink acknowledges the drone took off,
        # and the path planner sent the first waypoint.
        elif self.__state == "take_off":
            if self.relative_alt>self.TAKEOFF_ALTITUDE-1 and self.new_path:
                self.__state = "fly"
                self.taking_off = False
                self.state_to_log()
                # Restart timer for the new state.
                self.__state_timer = 0.0

        # FLY state. If there are no more waypoints, landing has to start.
        elif self.__state == "fly":
            if not self.comm_ok:
                self.__state = "recover_comm"
                rospy.logwarn("Dronelink lost")
                self.state_to_log()
                self.__state_timer = 0.0
            elif self.emergency_stop:
                self.__state = "calculate_path"
                self.new_waypoint = False
                self.state_to_log()
                self.__state_timer = 0.0
            elif self.new_path:
                self.__state = "upload_mission"
                self.mission_ready = False
                self.mission_started = False
                self.new_path = False
                self.state_to_log()
                self.__state_timer = 0.0
            elif self.dest_unreachable:
                pass
            elif (self.current_mission == len(self.current_path)-1 and
                        self.distance_to_dest < self.destination_threshold_dist
                        ):
                    rospy.logdebug("Distance to dest: {}"
                                   "".format(self.distance_to_dest))
                    self.__state = "land"
                    self.mission_ready = False
                    self.mission_started = False
                    self.state_to_log()
                    rospy.loginfo("Hovering {} secs".format(self.HOVERING_TIME))
                    self.__state_timer = rospy.get_time()
            elif self.new_waypoint:
                    self.__state = "calculate_path"
                    self.new_waypoint = False
                    self.state_to_log()
                    self.__state_timer = 0.0

        # UPLOAD MISSION state
        elif self.__state == "upload_mission":
            if self.mission_ready:
                self.__state = "start_mission"
                self.mission_ready = False
                self.state_to_log()
                self.__state_timer = 0.0

        # START MISSION state
        elif self.__state == "start_mission":
            if self.mission_started:
                self.__state = "fly"
                self.mission_started = False
                self.state_to_log()

        # CALCULATE_PATH state
        elif self.__state == "calculate_path":
            if self.new_path and not (self.emergency_stop and
                                      not self.holding_position):
                self.__state = "upload_mission"
                self.emergency_stop = False
                self.new_path = False
                self.holding_position = False
                self.state_to_log()
                self.__state_timer = 0.0

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            if self.mode_updated:
                pass
            if self.holding_position:
                self.__state = "calculate_path"
                self.holding_position = False
                rospy.loginfo("Communication recovered")
                self.state_to_log()
                self.__state_timer = 0.0

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state
        elif self.__state == "land":
            if self.relative_alt<0.01:
                self.__state = "start"
                self.mission_ready = False
                self.landing = False
                self.state_to_log()
                self.__state_timer = 0.0

        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))

        return self.__state

    def update_outputs(self):
        """
        Update the output variables of the FSM.
        """
        # START state. Void. Wait until new operation is requested.
        if self.__state == "start":
            now = rospy.get_time()
            if now>self.__state_timer + self.TIMEOUT and not self.docking:
                self.ASK_DOCKING = True
                self.__state_timer = rospy.get_time()
            if all(self.home_pos):
                distance_to_home = np.linalg.norm(
                        np.array(self.home_pos)-np.array(self.position))
                if distance_to_home > 0.0001:
                    self.destination = self.home_pos
                    self.new_mission = True

        # PLANNER SETUP state
        elif self.__state == "planner_setup":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                self.ACTIVATE_PLANNER = True
                self.CLEAR_MISSION = True
                self.__state_timer = rospy.get_time()

        # ARM state
        elif self.__state == "arm":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                if not self.taking_off:
                    self.TAKE_OFF = True
                if not self.armed:
                    self.ARM = True
                self.__state_timer = rospy.get_time()

        # TAKE OFF state
        elif self.__state == "take_off":
            now = rospy.get_time()
            if  now > self.__state_timer + self.TIMEOUT + 10:
                self.CALCULATE_PATH = True
                self.__state_timer = rospy.get_time()
            pass

        # FLY state
        elif self.__state == "fly":
            # Check that there are no None values in the list
            if all(self.position):
                self.distance_to_dest = np.linalg.norm(
                        np.array(self.destination)-np.array(self.position))
            if self.dest_unreachable:
                self.destination = path_operations.get_new_goal(
                        self.position, self.destination, 1, 1, self.batt_level)
            pass

        # CALCULATE_PATH state
        elif self.__state == "calculate_path":
            now = rospy.get_time()
            if now > self.__state_timer + self.PLANNER_TIMEOUT:
                self.CALCULATE_PATH = True
                if self.emergency_stop:
                    self.HOLD_POSITION = True
                    rospy.loginfo("Emergency: Hold position")
                self.__state_timer = rospy.get_time()

        # UPLOAD_MISSION state
        elif self.__state == "upload_mission":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                self.UPLOAD_MISSION = True
                self.__state_timer = rospy.get_time()

        # START_MISSION state
        elif self.__state == "start_mission":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                self.START_MISSION = True
                self.__state_timer = rospy.get_time()

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                self.HOLD_POSITION = True
                self.__state_timer = rospy.get_time()

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            self.EMERGENCY_LANDING = True

        # LANDING state
        elif self.__state == "land":
            now = rospy.get_time()
            # After hovering, upload a mission with the land waypoint
            if now > self.__state_timer+self.HOVERING_TIME and not self.mission_ready:
                rospy.loginfo("Starting landing")
                self.LAND = True
                self.__state_timer = rospy.get_time()
            # When the mission is uploaded, send the start mission command
            if (now>self.__state_timer+self.TIMEOUT and self.mission_ready
                    and not self.mission_started):
                rospy.loginfo("Landing uploaded")
                self.START_MISSION = True
                self.__state_timer = rospy.get_time()


        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))
        return

    def check_battery(self):
        if self.batt_level > self.MIN_TAKEOFF_BAT:
            self.takeoff_batt_ok = True
        else:
            self.takeoff_batt_ok = False
            if self.__state == "start":
                rospy.logwarn("Low battery level: {}%. Take off not allowed"
                              "".format(self.batt_level))
        if self.batt_level > self.MIN_FLY_BAT:
            self.batt_ok = True
        else:
            self.batt_ok = False
            rospy.logwarn("WARNING: Very low battery level: {}%"
                          "".format(self.batt_level))
        return self.takeoff_batt_ok

    def get_state(self):
        """
        Returns the state of the FSM.
        """
        return self.__state

    def state_to_log(self):
        """
        Sends to log the state of the FSM. Returns the state as well.
        """
        rospy.loginfo("FSM state: {}".format(self.__state))
        return self.__state
