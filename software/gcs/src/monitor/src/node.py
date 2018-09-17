#!/usr/bin/env python
# -*- coding: utf-8 -*-

# imports
import rospy
import struct
from std_msgs.msg import Int8
from monitor.srv import getstatus
from threading import Lock

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

        # launch node
        rospy.init_node('monitor', disable_signals=True)

        # service
        rospy.Service("gcs/monitor/getstatus", getstatus, self.getstatus)

        # Nodes to monitor
        #self.monitor_nodes.append(rospy.ServiceProxy("gcs/monitor/getstatus", getstatus))
        #self.monitor_nodes_status.append("n/a")
        #self.monitor_nodes_name.append("Monitor")

        # shutdown handler
        rospy.on_shutdown(self.shutdown)

        # Start timer for fetching
        rospy.Timer(rospy.Duration(self.fetch_interval), self.fetchStatus())

        self.rate = rospy.Rate(self.update_interval)
        rospy.sleep(1)  # wait until everything is running
    
    def fetchStatus(self):

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
                None # todo 

        self.mutex.acquire()
        self.status = "idle"

        # release mutex
        self.mutex.release()

    def getstatus(self, var):
        return self.alive, self.status

    def update_display (self):
        now = rospy.get_time()
        print('\033[2J') # clear screen
        print('') # go home

        # update last_update - When was it last updated. Has to be updated here due to time flow
        t = now - self.last_update
        if t < 86400:
            last_update_text = '%ds' % t
        else:
            last_update_text = 'Never'

        # Print
        print('\033[1HLast heard:         %s' % last_update_text)

        self.mutex.acquire()

        print("\033[{0}H{1}:           {2}".format(str(2), "monitor", self.status))

        for idx,value in enumerate(self.monitor_nodes):
            print("\033[{0}H{1}:           {2}".format(str(idx * 2 + 4), self.monitor_nodes_name[idx], self.monitor_nodes_status))

        self.mutex.release()

        # print '\033[2H%s: %s' % last_heard_status_text
        # print '\033[4H%s:           %s' % batt_text
        # print '\033[6H%s:           %s' % pos_text
        # print '\033[7H%s:           %s' % alt_text
        # print '\033[8H%s:           %s' % home_text
        # print '\033[10H%s:          %s' % atti_text
        # print '\033[12H%s:          %s' %
        print('\033[?25l') # hide cursor

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff
            self.update_display()
            # sleep the defined interval
            self.rate.sleep()

    def shutdown(self):
        print("Monitor shut down successfully")


if __name__ == '__main__':

    node = monitor()
    node.run()


# Idea : 
# Every 10 sec run service call for status on every node. Update own variables and ui, handle error statuses