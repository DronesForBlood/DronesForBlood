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
try:
    import utm.msg
except ModuleNotFoundError:
    print("UTM module not found")

class GcsMasterNode():

    # Node variables
    UTM_PERIOD = 1              # Seconds
    HEARBEAT_PERIOD = 0.5       # Seconds
    HEARTBEAT_TIMEOUT = 1.5     # Seconds
    BATTERY_CHECK_TIMEOUT = 5   # Seconds

    MAX_COMM_LOSES = 3          # Tries before entering recover_comm state

    def __init__(self, alt=50, takeoff_batt=80, fly_batt=30, hover_time=30):
        
        self.altitude = alt
        rospy.logdebug("ALTITUDE: {}".format(self.altitude))
        # Create an instance of the drone finite-state-machine class.
        self.state_machine = drone_fsm.DroneFSM(
                takeoff_altitude=alt, takeoff_batt=takeoff_batt,
                fly_batt=fly_batt, hovering_time=hover_time)

        # Timestamps variables. Sending time set to zero for forcing the sending
        # of a heartbeat in the first iteration.
        self.heartbeat_send_time = 0.0
        self.utm_send_time = 0.0
        self.battery_check_time = 0.0
        self.heartbeat_receive_time = rospy.get_time()

        # Userlink topic susbscribers
        rospy.Subscriber("userlink/start", std_msgs.msg.Int16MultiArray,
                         self.ui_start_callback, queue_size=1)
        rospy.Subscriber("userlink/destination",
                         mavlink_lora.msg.mavlink_lora_pos,
                         self.ui_destination_callback, queue_size=1)
        # Pathplanner topic susbscribers
        rospy.Subscriber("pathplanner/mission_list",
                         mavlink_lora.msg.mavlink_lora_mission_list,
                         self.pathplanner_newplan_callback)
        rospy.Subscriber("pathplanner/is_ready", std_msgs.msg.Bool,
                         self.pathplanner_isready_callback)
        # Mavlink topic susbscribers
        rospy.Subscriber("mavlink_status",
                         mavlink_lora.msg.mavlink_lora_status,
                         self.mavlink_status_callback, queue_size=1)
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

        # Userlink topic publishers
        self.userlink_ack_pub = rospy.Publisher(
                "gcs_master/destination_ack",
                std_msgs.msg.Bool,
                queue_size=1)
        # Pathplanner topic publishers
        self.calc_path_pub = rospy.Publisher(
                "gcs_master/calculate_path",
                std_msgs.msg.Bool,
                queue_size=1)
        self.activate_planner_pub = rospy.Publisher(
                "pathplanner/get_is_ready",
                mavlink_lora.msg.mavlink_lora_pos,
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
        self.set_mode_pub = rospy.Publisher(
                "mavlink_interface/command/set_mode",
                mavlink_lora.msg.mavlink_lora_command_set_mode,
                queue_size=1)
        # UTM topic publisher
        self.utm_data_pub = rospy.Publisher(
                "utm/add_tracking_data",
                utm.msg.utm_tracking_data,
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

        if self.state_machine.ACTIVATE_PLANNER:
            msg = mavlink_lora.msg.mavlink_lora_pos()
            msg.lat = self.state_machine.destination[0]
            msg.lon = self.state_machine.destination[1]
            self.activate_planner_pub.publish(msg)
            self.state_machine.ACTIVATE_PLANNER = False

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
            # Remove the hold position command, in case it was previously
            # set to true
            mode_msg = mavlink_lora.msg.mavlink_lora_command_set_mode()
            # base mode = 4 (AUTO). sub mode = 4 (MISSION)
            mode_msg.custom_mode = 4 << 16 | 4 << 24
            self.set_mode_pub.publish(mode_msg)
            msg = mavlink_lora.msg.mavlink_lora_command_start_mission()
            msg.first_item = 0
            msg.last_item = 2
            self.start_mission_pub.publish(msg)
            self.state_machine.START_MISSION = False

        if self.state_machine.HOLD_POSITION:
            msg = mavlink_lora.msg.mavlink_lora_command_set_mode()
            # base mode = 4 (AUTO). sub mode = 3 (LOITER)
            msg.custom_mode = 4 << 16 | 3 << 24
            self.set_mode_pub.publish(msg)
            self.state_machine.HOLD_POSITION = False

        if self.state_machine.EMERGENCY_LANDING:
            rospy.logwarn("shits fucked")

    def mavlink_status_callback(self, data):
        self.state_machine.batt_level = data.batt_remaining
        return

    def heartbeat_callback(self, data):
        sub_mode = data.custom_mode >> 24
        base_mode = (data.custom_mode >> 16) & 0xFF
        rospy.logdebug("Sub mode: {}. Base mode: {}".format(sub_mode, base_mode))
        if sub_mode == 2:
            # Mission mode
            pass
        elif sub_mode == 3:
            # Loiter mode
            self.state_machine.holding_position = True
        elif sub_mode == 5:
            # RTL mode
            pass
        if not self.state_machine.comm_ok:
            rospy.loginfo("Communication recovered")
        self.state_machine.comm_ok = True
        self.heartbeat_receive_time = rospy.get_time()
        return

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
        if data.command == 176:
            if ack:
                self.state_machine.mode_updated = True
        if data.command == 300:
            if ack:
                self.state_machine.mission_started = True
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
        self.state_machine.new_mission = True
        if all(coord is not None for coord in self.state_machine.destination):
            self.userlink_ack_pub.publish(True)
            rospy.loginfo("Destination received and acknowledged")
        else:
            self.userlink_ack_pub.publish(False)
            rospy.logwarn("At least one destination coordinate is not valid")
        return
    #TODO: Acknowledge back to the dronelink that the mission is getting started

    def ui_destination_callback(self, data):
        self.state_machine.destination = [data.lat, data.lon]
        return

    def pathplanner_newplan_callback(self, data):
        for wp in data.waypoints:
            wp.z = self.altitude
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

    def pathplanner_isready_callback(self, data):
        """
        The incomming boolean data specifies if the pathplanner is ready
        """
        if data.data:
            self.state_machine.planner_ready = True
        elif not data.data:
            self.state_machine.planner_ready = False
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


    def send_utm_data(self):
        """
        Publish drone data to the UTM system

        The data is gathered from different attributes of the drone FSM
        instance.
        """
        # Check if there is already a plan loaded
        if not self.state_machine.current_path:
            return
        # Get the next waypoint in the mission
        wp_next = self.state_machine.current_path[
                self.state_machine.current_mission]

        msg = utm.msg.utm_tracking_data()
        msg.uav_op_status 
        msg.pos_cur_lat_dd = self.state_machine.latitude
        msg.pos_cur_lng_dd = self.state_machine.longitude
        msg.pos_cur_alt_m = self.state_machine.altitude
        msg.pos_cur_hdg_deg = self.state_machine.heading
        #TODO: Specify the correct drone velocity
        msg.pos_cur_vel_mps = 10
        msg.pos_cur_gps_timestamp = rospy.get_time()
        msg.wp_next_lat_dd = wp_next.x
        msg.wp_next_lng_dd = wp_next.y
        msg.wp_next_alt_m = wp_next.z
        msg.wp_next_hdg_deg = 0.0
        #TODO: Specify the correct drone velocity
        msg.wp_next_vel_mps = 10
        #TODO: Estimate the epoch that the drone will reach the next wp
        msg.wp_next_eta_epoch = 0.0
        msg.uav_bat_soc = self.state_machine.batt_level

        # Publish the message
        self.utm_data_pub.publish(msg)
        return

    def run(self):
        """ 
        Main loop. Update the FSM and publish variables.
        """
        rospy.sleep(0.5)
        rate = rospy.Rate(100)
        # Update time for checking initial heartbeat
        self.heartbeat_send_time = rospy.get_time()
        self.state_machine.state_to_log()

        while not rospy.is_shutdown():
            now = rospy.get_time()
            # Check the ramaining battery level is over threshold
            if now > self.battery_check_time + self.BATTERY_CHECK_TIMEOUT:
                self.state_machine.check_battery()
                self.battery_check_time = rospy.get_time()
            # Update the state of the FSM.
            self.state_machine.update_state()
            self.state_machine.update_outputs()
            # Publish the flags of the FSM.
            self.update_flags()
            # Publish the drone tracking data to the UTM periodically
            if now > self.utm_send_time + self.UTM_PERIOD:
                self.send_utm_data()
            # Publish the heartbeat with the adequate rate
            if now > self.heartbeat_send_time + self.HEARBEAT_PERIOD:
                self.send_heartbeat()
            # Check if the drone heartbeat times out.
            if now > self.heartbeat_receive_time + self.HEARTBEAT_TIMEOUT:
                if self.state_machine.comm_ok:
                    self.state_machine.comm_ok = False
            # Finish the loop cycle.
            rate.sleep()
        self.clear_mission_pub.publish()
        return


def main(alt=50, batt1=80, batt2=30, hover_time=20):
    # Instantiate the gcs_master node class and run it
    gcs_master = GcsMasterNode(alt=alt, takeoff_batt=batt1,
                               fly_batt=batt2, hover_time=hover_time)
    gcs_master.run()
    return
