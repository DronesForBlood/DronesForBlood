#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries


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
        self.height = 0
        self.position = [None, None]
        self.destination = [None, None]
        self.route = []
        self.distance_to_station = 0
        self.armed = False
        self.batt_ok = False
        self.comm_ok = False
        self.on_air = False
        self.new_waypoint = False
        self.new_path = False
        self.max_lowbatt_distance = max_lowbatt_distance
        # FSM parameters
        self.__state = "start"

    def update_state(self):
        """
        Update the state of the FSM, based on the state variables.
        """
        # START state
        if self.__state == "start":
            if self.armed:
                self.__state = "armed"
        # ARMED state
        elif self.__state == "armed":
            if self.destination and self.batt_ok and self.comm_ok:
                self.__state = "new_plan"
        # NEW PLAN state
        elif self.__state == "new_plan":
            if self.route:
                self.__state = "taking_off"
        # TAKING OFF state
        elif self.__state == "taking_off":
            if self.height > 50:
                self.__state = "flying"
        # FLYING state
        elif self.__state == "flying":
            if not self.comm_ok:
                self.__state = "recover_comm"
            elif not self.batt_ok:
                self.__state = "new_destination"
            elif self.new_waypoint:
                self.CALCULATE_PATH = True
            elif not self.route:
                self.__state = "landing"
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
            pass
        # LANDING state
        elif self.__state == "landing":
            if self.height == 0:
                self.__state = "landed"
        # LANDED state
        elif self.__state == "landed":
            pass
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
            pass
        # ARMED state
        elif self.__state == "armed":
            self.ARMED = True
        # NEW PLAN state
        elif self.__state == "new_plan":
            self.CALCULATE_PATH = True
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
        # NEW DESTINATION state
        elif self.__state == "new_destination":
            pass
        # NEW PATH state
        elif self.__state == "new_path":
            self.CALCULATE_PATH = True
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
