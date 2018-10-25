#!/usr/bin/env python3
"""
GCS top level node.

It communicates with the other nodes (mavlink, UI, and path planner)
through ROS topics. Hence asynchronous communication.

Implements a Finite-State-Machine (FSM) for determining and evolving the
state of the drone controlling system.

In order to ensure the correct working of the FSM, there must be mutexes
placed in order to avoid variable changes while evolving the states.
"""
# Standard libraries

# Third-party libraries
import rospy
import geometry_msgs.msg
import std_msgs.msg
import pymap3d as pm
# Local libraries
import drone_fsm


class GcsMasterNode():

    def __init__(self):
        # Subscribers configuration
        rospy.Subscriber('userlink/start', geometry_msgs.msg.Point, ui_callback, queue_size=1)
        rospy.Subscriber('pathplanner/waypoint/receive', mavlink_lora_mision_list, planner_callback, queue_size=1)
        rospy.Subscriber('mavlink/drone/ack', mavlink_lora_command_ack, dronelink_callback, queue_size=1)
        rospy.Subscriber('mavlink/drone/error', mavlink_lora_statustext, dronelink_callback, queue_size=1)
        rospy.Subscriber('mavlink/drone/position', mavlink_lora_pos, dronelink_callback, queue_size=1)

        # Publishers configuration
        self.waypoint_request = rospy.Publisher('pathplanner/waypoint/request', std_msgs.msg.bool, queue_size=1)
        self.mavlink_drone_arm = rospy.Publisher('mavlink/drone/arm', std_msgs.msg.bool, queue_size=1)
        self.mavlink_drone_takeoff = rospy.Publisher('mavlink/drone/takeoff', std_msgs.msg.bool, queue_size=1)
        self.mavlink_drone_Waypoint = rospy.Publisher('mavlink/drone/waypoint', geometry_msgs.msg.Point, queue_size=1)

        # Create an instance of the drone finite-state-machine class.
        self.state_machine = drone_fsm.DroneFSM()

    def update_flags(self):
        if self.state_machine.CALCULATE_PATH:
            self.path_request.publish(True)
            self.state_machine.CALCULATE_PATH = False

        if self.state_machine.ARMED:
            self.mavlink_drone_arm.publish(True)
            self.state_machine.ARMED = False

        if self.state_machine.TAKE_OFF:
            self.mavlink_drone_takeoff.publish(True)
            self.state_machine.TAKE_OFF = False

        if self.state_machine.FLY:
            print("hi")

        if self.state_machine.WAYPOINT_REACHED:
            self.waypoint_request.publish(True)
            self.state_machine.WAYPOINT_REACHED = False

        if self.state_machine.LAND:
            print("hi")

        if self.state_machine.EMERGENCY_LANDING:
            print("shits fucked")

    def ui_callback(self, data):
        self.state_machine.destination = [data.x, data.y]
        return

    def dronelink_callback(self, data):
        self.state_machine.batt_ok = True
        self.state_machine.comm_ok = True
        if data._connection_header["topic"] == "/mavlink/drone/error":
            # Differentiate between batt and comms error here.
            self.state_machine.batt_ok = False
            self.state_machine.comm_ok = False

        elif data._connection_header["topic"] == "/mavlink/drone/ack":
            if data[2] == 1:
                self.state_machine.ack = True
            else:
                self.state_machine.ack = False

        elif data._connection_header["topic"] == "/mavlink/drone/position":
            # data[0] = header
            # data[1] = time_usec
            self.state_machine.position = [data[2], data[3]]
            self.state_machine.altitude = data[4]
            # data[5] = relative_altitude
            # data[6] = heading
        return

    def planner_callback(self, data):
        self.state_machine.waypoint = [data[0][0], data[0][1]]
        self.state_machine.new_waypoint = True
        return

    def run(self):
        """ 
        Main loop. Update the FSM and publish variables.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Update the state of the FSM.
            self.state_machine.update_state()
            self.state_machine.update_outputs()
            # Publish the flags of the FSM.
            self.update_flags()
            # Finish the loop cycle.
            rate.sleep()
        return


def main():
    # Init node and publisher object
    rospy.init_node("gcs_master", anonymous=True)
    # Instantiate the gcs_master node class and run it
    gcs_master = GcsMasterNode()
    gcs_master.run()
    return

if __name__ == "__main__":
    main()
