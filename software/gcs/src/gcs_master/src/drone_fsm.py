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
        self.READY = False
        self.TAKE_OFF = False
        self.LAND = False
        self.FLY = False
        self.CALCULATE_PATH = False
        self.EMERGENCY_LANDING = False

        # Drone parameters. FSM inputs
        self.altitude = 0				        # Altitude
        self.position = [None, None]	 	# Current position
        self.destination = [None, None]	 	# Next waypoint
        self.ack = False
        self.armed = False			        # Armed / Disarmed
        self.batt_ok = False			    # Battery status
        self.comm_ok = False			    # Comlink status
        self.on_air = False			        # Whether it is in the air

        # Path related vars
        self.new_path = False			    # If new path is available
        self.new_waypoint = False		    # If new waypoint is available

        # FSM parameters
        self.__state = "start"
        self.msg = 0            # Message type, route, waypoint, position etc.

    def update_state(self):
        """
        Update the state of the FSM, based on the state variables.
        """
        # Check for errors first!
        if self.msg._connection_header["topic"] == "/mavlink/drone/error":
            if self.msg == "BATTERY_ERROR":                                         # Battery error
                self.__state = "emergency_landing"

            elif self.msg == "RC_LINK_LOSS":                                        # Commlink error
                self.__state = "recover_comm"

        # START state
        if self.__state == "start":
            if self.msg._connection_header["topic"] == "/path":                     # If message comes from path topic, read route
                self.route = self.msg           		                            # Read route

            elif self.msg._connection_header["topic"] == "/mavlink/drone/ack":	    # Response from arm request
                if self.msg == 1:
                    self.msg = 0  		                                            # Clear message before next state.
                    self.__state = "armed"                                          # Proceed to next state.

        # ARMED state
        elif self.__state == "armed":	
            if self.msg._connection_header["topic"] == "/mavlink/drone/ack":        # If MavLink acknowledges, proceed to next state
                if self.msg == 1:
                    self.msg = 0		                                            # Clear msg before next state.
                    self.__state = "taking_off"

        # TAKING OFF state
        elif self.__state == "taking_off":
            if self.msg._connection_header["topic"] == "/mavlink/drone/ack":        # HOW TO DETECT TAKEOFF?
                if self.msg == 1: 
                    self.msg = 0
                    self.__state = "flying"

        # FLYING state
        elif self.__state == "flying":
            if self.msg._connection_header["topic"] == "/mavlink/drone/ack":        # If destination is reached, what message is sent?
                self.msg = 0
                self.__state = "landing"

            elif self.msg._connection_header["topic"] == "/path":                   # If message comes from path topic, read route
                self.route = self.msg                                               # Read route

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            #If we're here, drone has landed, unless we can abort?
            # Go to take-off state?
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state - NECESSARY? 
        elif self.__state == "landing":
            #if self.msg = DRONE LANDED
            self.__state = "landed"

        # LANDED state
        elif self.__state == "landed":
            self.route = 0
            if self.msg._connection_header["topic"] == "/userlink/start":
                print("Go to start")
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
            else:
                self.READY = True

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

