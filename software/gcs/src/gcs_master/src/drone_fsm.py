#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries


class DroneFSM():

    def __init__(self):
        # Drone parameters
        self.height = 0
        self.position = [None, None]
        self.route = []
        self.distance_to_station = 0
        self.armed = False
        self.batt_ok = False
        self.comm_ok = False
        self.on_air = False
        # FSM parameters
        self.state = "start"

    def update(self):
        if self.state == "start":
            pass
        elif self.state == "armed":
            pass
        elif self.state == "taking_off":
            pass
        elif self.state == "flying":
            pass
        elif self.state == "recover_comm":
            pass
        elif self.state == "new_destination":
            pass
        elif self.state == "new_path":
            pass
        elif self.state == "emergency_landing":
            pass
        elif self.state == "landing":
            pass
        elif self.state == "landed":
            pass
        else:
            raise ValueError("Unrecognized state")
        return
