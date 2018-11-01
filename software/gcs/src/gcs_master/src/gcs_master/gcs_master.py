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
# Local libraries
from gcs_master import drone_fsm
try:
    import mavlink_lora.msg
except ModuleNotFoundError:
    print("Mavlink module not found")


class GcsMasterNode():

    # Node variables
    HEARBEAT_PERIOD = 0.5       # Seconds
    HEARTBEAT_TIMEOUT = 1.5     # Seconds

    def __init__(self):
        # Subscribers configuration
        rospy.Subscriber("mavlink_heartbeat_rx",
                         mavlink_lora.msg.mavlink_lora_heartbeat,
                         self.heartbeat_callback, queue_size=1)
        rospy.Subscriber("dronelink/start", std_msgs.msg.Bool,
                         self.ui_start_callback, queue_size=1)
        rospy.Subscriber("mavlink_pos", mavlink_lora.msg.mavlink_lora_pos,
                         self.mavlink_pos_callback, queue_size=1)
        rospy.Subscriber("mavlink_interface/command/ack",
                         mavlink_lora.msg.mavlink_lora_command_ack,
                         self.mavlink_ack_callback, queue_size=1)

        # Publishers configuration
        self.heartbeat_pub = rospy.Publisher(
                "mavlink_heartbeat_tx",
                mavlink_lora.msg.mavlink_lora_heartbeat,
                queue_size=1)
        self.drone_arm_pub = rospy.Publisher(
                "mavlink_interface/command/arm_disarm",
                std_msgs.msg.Bool,
                queue_size=1)
        self.drone_takeoff_pub = rospy.Publisher(
                "mavlink_interface/command/takeoff",
                mavlink_lora.msg.mavlink_lora_command_takeoff,
                queue_size=1)
        self.calc_path_pub = rospy.Publisher(
                "gcs_master/calculate_path",
                std_msgs.msg.Bool,
                queue_size=1)

        # Create an instance of the drone finite-state-machine class.
        self.state_machine = drone_fsm.DroneFSM()

        # Timestamps variables. Sending time set to zero for forcing the sending
        # of a heartbeat in the first iteration.
        self.heartbeat_send_time = 0.0
        self.heartbeat_receive_time = rospy.get_time()

    def update_flags(self):

        if self.state_machine.ARM:
            self.drone_arm_pub.publish(True)
            self.state_machine.ARM = False

        if self.state_machine.TAKE_OFF:
            #TODO: Set the parameters to adequate values
            msg = mavlink_lora.msg.mavlink_lora_command_takeoff()
            msg.latitude = self.state_machine.latitude
            msg.longtitude = self.state_machine.longitude
            msg.altitude = (self.state_machine.altitude +
                            self.state_machine.TAKEOFF_ALTITUDE)
            msg.yaw_angle = float('NaN') # unchanged angle
            msg.pitch = 0
            self.drone_takeoff_pub.publish(msg)
            self.state_machine.TAKE_OFF = False

        if self.state_machine.CALCULATE_PATH:
            self.calc_path_pub.publish(True)
            self.state_machine.CALCULATE_PATH = False

        if self.state_machine.FLY:
            pass

        if self.state_machine.WAYPOINT_REACHED:
            pass

        if self.state_machine.LAND:
            pass

        if self.state_machine.EMERGENCY_LANDING:
            rospy.logwarn("shits fucked")

    def heartbeat_callback(self, data):
        self.state_machine.comm_ok = True
        self.heartbeat_receive_time = rospy.get_time()

    def mavlink_ack_callback(self, data):
        ## Check if command is acknowledged
        ack = False
        if data.result == 0:
            ack = True
        # Temporarily rejected
        elif data.result == 1:
            pass
        # Result denied
        elif data.result == 2:
            pass
        # Result unsupported
        elif data.result == 3:
            pass
        # Result failed
        elif data.result == 4:
            pass
        # Result in progress
        elif data.result == 5:
            pass

        ## Check command to be acknowledged
        if data.command == 22:
            if ack:
                self.state_machine.taking_off = True
        #TODO: write the correct command for the arm command
        if data.command == 99:
            if ack:
                self.state_machine.armed = True
        return

    def mavlink_pos_callback(self, data):
        self.state_machine.latitude = data.lat
        self.state_machine.longitude = data.lon
        self.state_machine.altitude = data.alt
        self.state_machine.relative_alt = data.relative_alt
        self.state_machine.heading = data.heading
        return

    def ui_start_callback(self, data):
        self.state_machine.mission = True
        return

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

    def send_heartbeat(self):
        """
        Broadcast a heartbeat containing the GCS basic information.
        """
        msg = mavlink_lora.msg.mavlink_lora_heartbeat()
        msg.type = 6
        msg.autopilot = 8
        msg.base_mode = 192
        msg.custom_mode = 0
        msg.system_status = 4
        msg.system_id = 255
        self.heartbeat_pub.publish(msg)
        self.heartbeat_send_time = rospy.get_time()
        rospy.logdebug("BIP")
        return

    def run(self):
        """ 
        Main loop. Update the FSM and publish variables.
        """
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            # Update the state of the FSM.
            self.state_machine.update_state()
            self.state_machine.update_outputs()
            # Publish the flags of the FSM.
            self.update_flags()
            # Publish the heartbeat with the adequate rate
            if now > self.heartbeat_send_time + self.HEARBEAT_PERIOD:
                self.send_heartbeat()
            # Check if the drone heartbeat times out.
            if now > self.heartbeat_receive_time + self.HEARTBEAT_TIMEOUT:
                rospy.logwarn("Dronelink lost")
                self.state_machine.comm_ok = False
                self.heartbeat_receive_time = rospy.get_time()
            # Finish the loop cycle.
            rate.sleep()
        return


def main():
    # Instantiate the gcs_master node class and run it
    gcs_master = GcsMasterNode()
    gcs_master.run()
    return
