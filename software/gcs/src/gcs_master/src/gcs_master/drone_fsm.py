#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries
# Third-party libraries
import numpy as np
import rospy
import std_msgs.msg


class DroneFSM():

    TIMEOUT = 5             # Timeout, in seconds, for asking again for commands
    TAKEOFF_ALTITUDE = 10   # Altitude set point, in meters, after taking off
    MISSION_LENGTH = 4      # Number of waypoints sent to the drone
    MIN_TAKEOFF_BAT = 80    # Minimum battery level for taking off
    MIN_FLY_BAT = 30        # Battery warning raised below this level

    def __init__(self, max_lowbatt_distance=100):
        """
        :param int max_lowbatt_distance: maximum distance in meters that
         can be covered by the drone after entering low battery mode.
        """
        # Threshold distances
        self.new_waypoint_distance = 5      # Distance for getting new waypoint
        self.destination_threshold_dist = 0.000006   # Dist. for start landing
        # FSM flags. Outputs
        self.DISARM = False
        self.ARM = False
        self.TAKE_OFF = False
        self.LAND = False
        self.CALCULATE_PATH = False
        self.UPLOAD_MISSION = False
        self.START_MISSION = False
        self.EMERGENCY_LANDING = False
        # Drone parameters. FSM inputs
        self.altitude = None                # Absolute altitude
        self.relative_alt = None            # Relative altitude
        self.latitude = None                # Drone latitude
        self.longitude = None               # Drone longitude
        self.heading = None                 # Heading, in degrees
        self.position = [None, None]        # Current position
        self.destination = [None, None]     # Destination location
        self.route = []                     # List of all waypoints
        self.current_path = []              # List of immediate waypoints
        self.distance_to_dest = 0           # Distance to the final destination
        self.armed = False                  # Armed / Disarmed
        self.taking_off = False             # Drone on taking-off operation
        self.landing = False                # Drone is landing
        self.takeoff_batt_ok = False        # Enough battery for taking off
        self.batt_ok = False                # Enough battery for flying
        self.comm_ok = False                # Comlink status
        self.batt_level = 0                 # Reamining battery capacity
        # Path related variables
        self.new_path = False               # If new path is available
        self.new_waypoint = False           # New waypoint is available
        self.mission_ready = False          # The drone received a mission
        self.new_mission = False            # The drone has started the mission
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
            if self.new_mission and self.comm_ok and self.takeoff_batt_ok:
                self.__state = "arm"
                self.new_mission = False
                self.state_to_log()
                # Restart timer for the new state, so it updates the flags asap.
                self.__state_timer = 0.0

        # ARM state. Wait until mavlink acknowledges the drone implemented
        # the take-off command.
        elif self.__state == "arm":	
            if self.armed and self.taking_off:
                self.__state = "take_off"
                self.state_to_log()

        # TAKING OFF state. Wait until mavlink acknowledges the drone took off,
        # and the path planner sent the first waypoint.
        elif self.__state == "take_off":
            if self.relative_alt>self.TAKEOFF_ALTITUDE-1 and self.new_path:
                self.__state = "fly"
                self.taking_off = False
                self.state_to_log()
                # Restart timer for the new state.
                self.__state_timer = 0.0

        # FLYING state. If there are no more waypoints, landing has to start.
        elif self.__state == "fly":
            if self.new_path:
                self.__state = "upload_mission"
                self.new_path = False
                self.state_to_log()
                self.__state_timer = 0.0
            elif (self.current_mission == len(self.current_path)-1 and
                        self.distance_to_dest < self.destination_threshold_dist
                        ):
                    self.__state = "land"
                    self.state_to_log()
                    self.__state_timer = rospy.get_time()
            elif self.new_waypoint:
                    self.__state = "calculate_path"
                    self.new_waypoint = False
                    self.state_to_log()
                    # rospy.loginfo("FSM state: calculate_path (DEBUG: Transition"
                    #               " not applied)")
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
            if self.new_mission:
                self.__state = "fly"
                self.new_mission = False
                self.state_to_log()

        # CALCULATE_PATH state
        elif self.__state == "calculate_path":
            if self.new_path:
                self.__state = "upload_mission"
                self.new_path = False
                self.state_to_log()
                self.__state_timer = 0.0

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state
        elif self.__state == "land":
            if self.relative_alt<0.01:
                self.__state = "start"
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
            if self.armed and now > self.__state_timer + self.TIMEOUT:
                self.DISARM = True
                self.__state_timer = rospy.get_time()

        # ARM state
        elif self.__state == "arm":
            now = rospy.get_time()
            if  now > self.__state_timer + self.TIMEOUT:
                self.CALCULATE_PATH = True
                self.TAKE_OFF = True
                self.ARM = True
                self.__state_timer = rospy.get_time()

        # TAKING OFF state
        elif self.__state == "take_off":
            pass

        # FLYING state
        elif self.__state == "fly":
            # Check that there are no None values in the list
            if all(self.position):
                self.distance_to_dest = np.linalg.norm(
                        np.array(self.destination)-np.array(self.position))
            pass

        # CALCULATE_PATH state
        elif self.__state == "calculate_path":
            now = rospy.get_time()
            if now > self.__state_timer + self.TIMEOUT:
                self.CALCULATE_PATH = True
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
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            self.EMERGENCY_LANDING = True

        # LANDING state
        elif self.__state == "land":
            now = rospy.get_time()
            if now > self.__state_timer+self.TIMEOUT and not self.landing:
                self.LAND = True
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