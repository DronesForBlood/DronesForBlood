#!/usr/bin/env python
# -*- coding: utf-8 -*-

# imports
import rospy
import struct
from std_msgs.msg import Int8
from monitor.srv import getstatus
from threading import Lock
from mavlink_lora.msg import mavlink_lora_set_position_target_local_ned, mavlink_lora_pos, mavlink_lora_heartbeat, mavlink_lora_statustext, mavlink_lora_attitude, mavlink_lora_status
from math import pi, sqrt, sin, cos, atan2
from gui import MonitorGUI
import threading
from dotmap import DotMap
from collections import deque

# defines
R = 6371000 # Assumed Earth radius in meter
DEG2RAD = pi/180.0
RAD2DEG = 180.0/pi
MAX_STATUSTEXT_MESSAGES = 20

class monitor:
    def __init__(self):

        # status variables
        self.last_update = 0
        self.update_interval = 5 #hz
        self.fetch_interval = 1 #hz

        self.status = "Init"
        self.alive = True
        self.mutex = Lock()

        self.monitor_nodes = []
        self.monitor_nodes_status = []
        self.monitor_nodes_name = []

        # drone status variables
        self.batt_volt = 0.0
        self.last_heard = 0
        self.last_heard_sys_status = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # Drone modes and offboard targets
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0
        self.statustext_buffer = deque()
        self.mav_mode = DotMap()
        # set defaults to handle if getting called before we get first msg
        self.mav_mode.base_mode = ""
        self.mav_mode.arm_status = ""

        # launch node
        rospy.init_node('monitor', disable_signals=True)

        # app wide global stop flag
        self.global_stop_event = threading.Event()

        # service
        rospy.Service("gcs/monitor/getstatus", getstatus, self.getstatus)

        # drone status to monitor
        rospy.Subscriber("mavlink_pos", mavlink_lora_pos, self.on_global_pos_msg)
        rospy.Subscriber("mavlink_attitude", mavlink_lora_attitude, self.on_mavlink_lora_attitude)
        rospy.Subscriber("mavlink_statustext", mavlink_lora_statustext, self.on_statustext_msg)
        rospy.Subscriber("mavlink_heartbeat", mavlink_lora_heartbeat, self.on_heartbeat_msg)
        rospy.Subscriber("mavlink_status", mavlink_lora_status, self.on_mavlink_lora_status)
        rospy.Subscriber("mavlink_interface/command/set_target_position_local_ned", mavlink_lora_set_position_target_local_ned, self.on_set_position_target_local_ned_msg)

        # Nodes to monitor
        #self.monitor_nodes.append(rospy.ServiceProxy("gcs/monitor/getstatus", getstatus))
        #self.monitor_nodes_status.append("n/a")
        #self.monitor_nodes_name.append("Monitor")

        # Keypress handler
        rospy.Subscriber("keypress", Int8, self.on_keypress)

        # Start timer for fetching
        rospy.Timer(rospy.Duration(self.fetch_interval), self.fetchStatus)

        self.rate = rospy.Rate(self.update_interval)
        rospy.sleep(1)  # wait until everything is running

    def fetchStatus(self, msg):

        # Mutex for async access to status/alive
        self.mutex.acquire()

        self.status = "Fetching Status"
        self.mutex.release()

        # loop though all nodes and get responses
        for idx, node in enumerate(self.monitor_nodes):
            # call service
            resp = node()

            # save 
            self.monitor_nodes_status[idx] = resp.status

            # check if any errors, send error sound/signal
            if resp.alive is False:
                do = "nothing" # todo

        self.mutex.acquire()
        self.status = "idle"

        # release mutex
        self.mutex.release()

    def getstatus(self, var):
        return self.alive, self.status

    def gcd_haversine(self, lat1, lon1, lat2, lon2):
        lat1 *= DEG2RAD
        lon1 *= DEG2RAD
        lat2 *= DEG2RAD
        lon2 *= DEG2RAD
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2.) ** 2 + sin(dlon / 2.) ** 2 * cos(lat1) * cos(lat2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return (R * c)

    def decode_base_mode(self, base_mode):
        armed =             bool(base_mode & 0x80)
        manual_input =      bool(base_mode & 0x40)
        HIL_simulation =    bool(base_mode & 0x20)
        stabilized_mode =   bool(base_mode & 0x10)
        guided_mode =       bool(base_mode & 0x08)
        auto_mode =         bool(base_mode & 0x04)
        test_mode =         bool(base_mode & 0x02)
        custom_mode =       bool(base_mode & 0x01)

        bm = DotMap()

        if manual_input:
            bm.base_mode = "MANUAL_INPUT"
        elif HIL_simulation:
            bm.base_mode = "HIL_SIMULATION"
        elif stabilized_mode:
            bm.base_mode = "STABILIZED"
        elif guided_mode:
            bm.base_mode = "GUIDED"
        elif auto_mode:
            bm.base_mode = "AUTO"
        elif test_mode:
            bm.base_mode = "TEST"
        elif custom_mode:
            bm.base_mode = "CUSTOM"

        # append armed state
        if armed:
            bm.arm_status = "ARMED"
        else:
            bm.arm_status = "DISARMED"

        return bm

    def on_global_pos_msg(self, msg):
        self.mutex.acquire()
        # Update current position
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        self.mutex.release()

    def on_mavlink_lora_attitude(self, msg):
        self.yaw = msg.yaw
        self.pitch = msg.pitch
        self.roll = msg.roll

    def on_statustext_msg(self, msg):
        self.mutex.acquire()
        # update status text
        self.statustext_buffer.append(msg.text)

        self.mutex.release()

    def on_heartbeat_msg(self, msg):
        self.mutex.acquire()
        # update base_mode and custom_mode, by decoding function
        self.mav_mode = self.decode_base_mode(msg.base_mode)

        # custom mode? maybe
        self.mutex.release()

    def on_set_position_target_local_ned_msg(self, msg):
        self.mutex.acquire()
        # update current target position. Also call function to update distance from current to next setpoint
        self.target_lat = msg.lat
        self.target_lon = msg.lon
        self.target_alt = msg.alt

        self.mutex.release()

    def on_mavlink_lora_status(self, msg):
        self.mutex.acquire()

        self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs / 1.0e9
        self.last_heard_sys_status = msg.last_heard_sys_status.secs + msg.last_heard_sys_status.nsecs / 1.0e9
        self.batt_volt = msg.batt_volt / 1000.0

        self.mutex.release()

    def update_display(self):
        self.mutex.acquire()

        # make string output for local variables
        batt_text = '%.1fV' % self.batt_volt
        pos_text = '%02.5f %03.5f' % (self.lat, self.lon)
        alt_text = '%.1fm ' % (self.alt)
        atti_text = 'Yaw: %03.1f Pitch: %03.1f Roll: %03.1f' % (self.yaw * 180 / pi, self.pitch * 180 / pi, self.roll * 180 / pi)

        dist_target_text = '%.1fm' % self.gcd_haversine(self.lat, self.lon, self.target_lat, self.target_lon)
        target_pos_text = '%02.5f %03.5f %.1fm' % (self.target_lat, self.target_lon, self.target_alt)

        # update last_update - When was it last updated. Has to be updated here due to time flow
        now = rospy.get_time()
        t = now - self.last_heard
        if t < 86400:
            last_update_text = '%ds' % t
        else:
            last_update_text = 'Never'

        # update last status
        t_sys_status = now - self.last_heard_sys_status
        if t_sys_status < 86400:
            last_heard_status_text = '%ds' % t_sys_status
        else:
            last_heard_status_text = 'Never'

        data = DotMap()
        data.last_heard = last_update_text
        data.last_sys_status = last_heard_status_text
        data.base_mode = self.mav_mode.base_mode
        data.armed = self.mav_mode.arm_status
        data.battery = batt_text
        data.curr_pos = pos_text
        data.curr_alt = alt_text
        data.target_pos = target_pos_text
        data.dist_target = dist_target_text
        data.attitude = atti_text
        data.statustext = self.statustext_buffer


        # update screen gui
        self.gui.update(data)

        # delete status text buffer, before next update
        self.statustext_buffer.clear()

        # release mutex
        self.mutex.release()

    def initial_display(self):
        # initial values
        data = DotMap()

        data.last_heard = "Never"
        data.last_sys_status = "Never"
        data.base_mode = "Unknown"
        data.armed = "Unknown"
        data.battery = "Unknown"
        data.curr_pos = "Unknown"
        data.curr_alt = "Unknown"
        data.target_pos = "Unknown"
        data.dist_target = "Unknown"
        data.attitude = "Unknown"
        data.statustext = self.statustext_buffer

        # update screen gui
        self.gui.update(data)

    def on_keypress(self, msg): # TODO get to work, instead of ctrl+c
        if msg.data == ord('q'):
            self.global_stop_event.set()
            rospy.signal_shutdown('User quit')

    def run(self):
        # create gui and start it
        self.gui = MonitorGUI(self.global_stop_event)
        self.gui.run()

        # seed gui with initial values
        self.initial_display()

        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            self.update_display()
            # sleep the defined interval
            self.rate.sleep()


if __name__ == '__main__':

    node = monitor()
    node.run()


# Idea : 
# Every 10 sec run service call for status on every node. Update own variables and ui, handle error statuses