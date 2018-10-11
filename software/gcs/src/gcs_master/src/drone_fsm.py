#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries
import std_msgs.msg
import rospy

class DroneFSM():

    def __init__(self, max_lowbatt_distance=100):
        """
        :param int max_lowbatt_distance: maximum distance in meters that
         can be covered by the drone after entering low battery mode.
        """
        # FSM flags. Outputs
        self.ARMED = False
        self.TAKE_OFF = False
        self.LAND = False
        self.FLY = False
        self.CALCULATE_PATH = False
        self.EMERGENCY_LANDING = False
        # Drone parameters. FSM inputs
        self.height = 0				        # Altitude
        self.position = [None, None]	 	# Current position
        self.destination = [None, None]	 	# Next waypoint
        self.route = [] 			        # Entire path
        self.distance_to_station = 0	    # Remaining distance?
        self.ready = False
        self.armed = False			        # Armed / Disarmed
        self.acknowledge = False            # Drone acknowledg 
        self.batt_ok = False			    # Battery status
        self.comm_ok = False			    # Comlink status
        self.on_air = False			        # Whether it is in the air?
        self.new_waypoint = False		    # New waypoint is available?
        self.new_path = False			    # New path is available?
        self.max_lowbatt_distance = max_lowbatt_distance
        # FSM parameters
        self.__state = "start"
        self.msg = 0            # Message type, route, waypoint, position etc.

    def update_state(self):
        """
        Update the state of the FSM, based on the state variables.
        """
        # Check for errors first!
        if self.msg._connection_header["topic"] == "/mavlink/drone/error":
            if self.msg == "BATTERY_ERROR":                  # Battery error
                self.__state = "emergency_landing"

            elif self.msg == "RC_LINK_LOSS":                 # Commlink error
                self.__state = "recover_comm"

        # START state. Wait until a new path is requested.
        if self.__state == "start":
            if self.new_path:
                self.__state = "armed"

        # ARMED state. Wait until mavlink acknowledges the drone is  armed.
        elif self.__state == "armed":	
            if self.acknowledge:
                self.__state = "taking_off"
                self.acknowledge = False

        # TAKING OFF state. Wait until mavlink acknowledges the drone took off,
        # and the path planner sent the first waypoint.
        elif self.__state == "taking_off":
            if self.acknowledge and self.new_waypoint:
                self.__state = "flying"
                self.acknowledge = False
                self.new_waypoint = False

        # FLYING state. If there are no more waypoints, landing has to start.
        elif self.__state == "flying":
            if not self.route:
                self.__state = "landing"

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            #If we're here, drone has landed, unless we can abort?
            # Go to take-off state?
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state
        # TODO NECESSARY? 
        elif self.__state == "landing":
            #if self.msg = DRONE LANDED
            self.__state = "landed"

        # LANDED state
        elif self.__state == "landed":
            # Something something, goto start

        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))

        return self.__state

    def update_outputs(self):
        """
        Update the output variables of the FSM.
        """
        # START state
        if self.__state == "start":
            if self.route == 0: # If route is empty
                self.CALCULATE_PATH = True

        # ARMED state
        elif self.__state == "armed":
            self.ARMED = True

        # TAKING OFF state
        elif self.__state == "taking_off":
            self.CALCULATE_PATH = False
            self.TAKE_OFF = True

        # FLYING state
        elif self.__state == "flying":
            self.TAKE_OFF = False
            self.CALCULATE_PATH = False

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

