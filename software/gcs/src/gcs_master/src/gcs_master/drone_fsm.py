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
    TAKEOFF_ALTITUDE = 20   # Altitude set point, in meters, after taking off
    MISSION_LENGTH = 4      # Number of waypoints sent to the drone

    def __init__(self, max_lowbatt_distance=100):
        """
        :param int max_lowbatt_distance: maximum distance in meters that
         can be covered by the drone after entering low battery mode.
        """
        # Threshold distances
        self.new_waypoint_distance = 5      # Distance for getting a new waypoint
        self.dest_reached_distance = 25     # Distance for starting landing
        # FSM flags. Outputs
        self.READY = False
        self.ARM = False
        self.TAKE_OFF = False
        self.LAND = False
        self.FLY = False
        self.CALCULATE_PATH = False
        self.NEW_MISSION = False
        self.EMERGENCY_LANDING = False
        self.WAYPOINT_REACHED = False
        # Drone parameters. FSM inputs
        self.altitude = None                # Absolute altitude
        self.relative_alt = None            # Relative altitude
        self.latitude = None                # Drone latitude
        self.longitude = None               # Drone longitude
        self.heading = None                 # Heading, in degrees
        self.position = [None, None]        # Current position
        self.destination = [None, None]     # Destination location
        self.route = []                     # List of all waypoints
        self.next_waypoint = [None, None]   # Coordinates of next waypoint
        self.distance_to_station = 0        # Remaining distance?
        self.ready = False
        self.armed = False                  # Armed / Disarmed
        self.taking_off = False             # Drone on taking-off operation
        self.acknowledge = False            # Mavlink acknowledge
        self.batt_ok = False                # Battery status
        self.comm_ok = False                # Comlink status
        self.on_air = False                 # Whether it is in the air
        # Path related variables
        self.new_path = False               # If new path is available
        self.new_waypoint = False           # New waypoint is available
        self.new_mission = False            # New operation is requested
        self.max_lowbatt_distance = max_lowbatt_distance
        self.current_mission = None
        self.current_path = []
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
            if self.new_mission:
                self.__state = "arm"
                self.new_mission = False
                # Restart timer for the new state, so it updates the flags asap.
                self.__state_timer = 0.0

        # ARM state. Wait until mavlink acknowledges the drone implemented
        # the take-off command.
        elif self.__state == "arm":	
            if self.armed and self.taking_off:
                self.__state = "taking_off"

        # TAKING OFF state. Wait until mavlink acknowledges the drone took off,
        # and the path planner sent the first waypoint.
        elif self.__state == "taking_off":
            if self.relative_alt>self.TAKEOFF_ALTITUDE and self.route:
                self.__state = "flying"
                self.taking_off = False
                # Atomic action. Send a new action when the taking off finishes
                self.NEW_MISSION = True
                # Restart timer for the new state.
                self.__state_timer = 0.0

        # FLYING state. If there are no more waypoints, landing has to start.
        elif self.__state == "flying":
            if self.new_waypoint:
                dist_to_destination = np.linalg.norm(
                        np.array(self.next_waypoint)-np.array(self.destination))
                if dist_to_destination < self.dest_reached_distance:
                    self.__state = "landing"
                self.new_waypoint = False

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            if self.comm_ok and self.on_air:
                self.__state = "new_path"
            elif self.comm_ok and not self.on_air:
                self.__state = "landed"
            #TODO: recovered comm timer.
        # NEW DESTINATION state
        elif self.__state == "new_destination":
            if self.distance_to_station <= self.max_lowbatt_distance:
                #TODO: Returning to the 'flying' state while having low battery
                # will provoke going to an endless loop.
                self.__state = "new_path"
            elif self.distance_to_station > self.max_lowbatt_distance:
                self.__state = "emergency_landing"
        # NEW PATH state
        elif self.__state == "new_path":
            #TODO: Wait until confirmation of new path
            if self.new_path:
                self.__state = "flying"
        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state
        # TODO NECESSARY? 
        elif self.__state == "landing":
            if False:
                self.__state = "landed"

        # LANDED state
        elif self.__state == "landed":
            pass
            # Something something, goto start

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
            pass

        # ARM state
        elif self.__state == "arm":
            now = rospy.get_time()
            if  now > self.__state_timer + self.TIMEOUT:
                self.CALCULATE_PATH = True
                self.TAKE_OFF = True
                self.ARM = True
                self.__state_timer = rospy.get_time()

        # TAKING OFF state
        elif self.__state == "taking_off":
            pass

        # FLYING state
        elif self.__state == "flying":
            if self.current_mission > 0:
                self.NEW_MISSION = True
                self.CALCULATE_PATH = True

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            self.EMERGENCY_LANDING = True

        # LANDING state
        elif self.__state == "landing":
            pass

        # LANDED state
        elif self.__state == "landed":
            pass

        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))
        return

    def get_state(self):
        return self.__state
