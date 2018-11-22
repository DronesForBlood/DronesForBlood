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

        # Create an instance of the drone finite-state-machine class.
        self.state_machine = drone_fsm.DroneFSM()

        # Timestamps variables. Sending time set to zero for forcing the sending
        # of a heartbeat in the first iteration.
        self.heartbeat_send_time = 0.0
        self.heartbeat_receive_time = rospy.get_time()

        # Dronelink topic susbscribers
        rospy.Subscriber("dronelink/start", std_msgs.msg.Bool,
                         self.ui_start_callback, queue_size=1)
        rospy.Subscriber("dronelink/destination",
                         mavlink_lora.msg.mavlink_lora_pos,
                         self.ui_destination_callback, queue_size=1)
        # Pathplanner topic susbscribers
        rospy.Subscriber("pathplanner/mission_list",
                         mavlink_lora.msg.mavlink_lora_mission_list,
                         self.pathplanner_newplan_callback)
        # Mavlink topic susbscribers
        rospy.Subscriber("mavlink_heartbeat_rx",
                         mavlink_lora.msg.mavlink_lora_heartbeat,
                         self.heartbeat_callback, queue_size=1)
        rospy.Subscriber("mavlink_pos", mavlink_lora.msg.mavlink_lora_pos,
                         self.mavlink_pos_callback, queue_size=1)
        rospy.Subscriber("mavlink_interface/command/ack",
                         mavlink_lora.msg.mavlink_lora_command_ack,
                         self.mavlink_ack_callback, queue_size=1)
        rospy.Subscriber("mavlink_interface/mission/current",
                         std_msgs.msg.UInt16,
                         self.mavlink_currentmission_callback, queue_size=1)
        rospy.Subscriber("mavlink_interface/mission/ack",
                         std_msgs.msg.String,
                         self.mavlink_missionack_callback, queue_size=1)

        # Pathplanner topic publishers
        self.calc_path_pub = rospy.Publisher(
                "gcs_master/calculate_path",
                std_msgs.msg.Bool,
                queue_size=1)
        # Mavlink topic publishers
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
        self.new_mission_pub = rospy.Publisher(
                "mavlink_interface/mission/mavlink_upload_mission",
                mavlink_lora.msg.mavlink_lora_mission_list,
                queue_size=1)
        self.start_mission_pub = rospy.Publisher(
                "mavlink_interface/command/start_mission",
                mavlink_lora.msg.mavlink_lora_command_start_mission,
                queue_size=1)
        self.drone_land_pub = rospy.Publisher(
                "mavlink_interface/command/land",
                mavlink_lora.msg.mavlink_lora_command_land,
                queue_size=1)
        self.clear_mission_pub = rospy.Publisher(
                "mavlink_interface/mission/mavlink_clear_all",
                std_msgs.msg.Empty,
                queue_size=1)

    def update_flags(self):

        if self.state_machine.DISARM:
            self.drone_arm_pub.publish(False)
            self.state_machine.DISARM = False

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

        if self.state_machine.UPLOAD_MISSION:
            msg = mavlink_lora.msg.mavlink_lora_mission_list()
            msg.waypoints = self.state_machine.current_path
            self.new_mission_pub.publish(msg)
            rospy.logdebug("Published a new mission to Mavlink")
            self.state_machine.UPLOAD_MISSION = False

        if self.state_machine.START_MISSION:
            msg = mavlink_lora.msg.mavlink_lora_command_start_mission()
            msg.first_item = 0
            msg.last_item = 2
            self.start_mission_pub.publish(msg)
            self.state_machine.START_MISSION = False

        if self.state_machine.LAND:
            msg = mavlink_lora.msg.mavlink_lora_command_land()
            msg.lat = self.state_machine.latitude
            msg.lon = self.state_machine.longitude
            msg.altitude = (self.state_machine.altitude -
                            self.state_machine.relative_alt)
            msg.yaw_angle = float('NaN')
            msg.abort_alt = 5
            # 2: required precision land with irlock
            msg.precision_land_mode = 2
            self.drone_land_pub.publish(msg)
            self.state_machine.LAND = False

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
            rospy.logwarn("Command {} temp. rejected".format(data.command))
            rospy.logwarn(data.result_text)
        # Result denied
        elif data.result == 2:
            rospy.logwarn("Command {} denied".format(data.command))
            rospy.logwarn(data.result_text)
        # Result unsupported
        elif data.result == 3:
            rospy.logwarn("Command {}: result unsupported".format(data.command))
            rospy.logwarn(data.result_text)
        # Result failed
        elif data.result == 4:
            rospy.logwarn("Command {}: result failed".format(data.command))
            rospy.logwarn(data.result_text)
        # Result in progress
        elif data.result == 5:
            rospy.logwarn("Command {}: result in progress".format(data.command))
            rospy.logwarn(data.result_text)

        ## Check command to be acknowledged
        if data.command == 22:
            if ack:
                self.state_machine.taking_off = True
        if data.command == 400:
            if ack:
                if self.state_machine.get_state() == "start":
                    self.state_machine.armed = False
                    rospy.loginfo("Drone is disarmed")
                else:
                    self.state_machine.armed = True
        if data.command == 21:
            if ack:
                self.state_machine.landing = True
        if data.command == 73:
            if ack:
                self.state_machine.mission_ready = True
        if data.command == 300:
            if ack:
                self.state_machine.new_mission = True
        return

    def mavlink_pos_callback(self, data):
        self.state_machine.latitude = data.lat
        self.state_machine.longitude = data.lon
        self.state_machine.altitude = data.alt
        self.state_machine.relative_alt = data.relative_alt
        self.state_machine.heading = data.heading

        self.state_machine.position[0] = data.lat
        self.state_machine.position[1] = data.lon
        return

    def mavlink_currentmission_callback(self, data):
        # If first waypoint is reached, set the FSM flag to true
        if data.data != self.state_machine.current_mission:
            if data.data > self.state_machine.current_mission:
                self.state_machine.new_waypoint = True
                rospy.loginfo("Waypoint reached")
            self.state_machine.current_mission = data.data
        return

    def mavlink_missionack_callback(self, data):
        if data.data == "MAV_MISSION_ACCEPTED":
            self.state_machine.mission_ready = True
            rospy.loginfo("Mission upload accepted")
        else:
            rospy.logwarn("Mission acknowledge failed!")
        return

    def ui_start_callback(self, data):
        rospy.logdebug("Start command received")
        self.state_machine.new_mission = True
        return
    #TODO: Acknowledge back to the dronelink that the mission is getting started

    def ui_destination_callback(self, data):
        self.state_machine.destination = [data.lat, data.lon]
        return

    def pathplanner_newplan_callback(self, data):
        self.state_machine.route = data.waypoints
        self.state_machine.new_path = True
        if len(data.waypoints) <= self.state_machine.MISSION_LENGTH:
            rospy.logdebug("Taking the {} waypoints in the mission (ALL)"
                           "".format(len(data.waypoints)))
            self.state_machine.current_path = data.waypoints
        else:
            self.state_machine.current_path = (
                    data.waypoints[0:self.state_machine.MISSION_LENGTH])
            rospy.logdebug("Taking the first {} waypoints in the mission"
                           "".format(self.state_machine.MISSION_LENGTH))
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
        rospy.sleep(0.5)
        rate = rospy.Rate(100)
        # Update time for checking initial heartbeat
        self.heartbeat_send_time = rospy.get_time()

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
        self.clear_mission_pub.publish()
        return


def main():
    # Instantiate the gcs_master node class and run it
    gcs_master = GcsMasterNode()
    gcs_master.run()
    return
