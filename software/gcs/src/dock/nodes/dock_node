#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64

class dock_node:

    def __init__(self):

        rospy.init_node('dock_node', anonymous=True)

        # Subscribers
        rospy.Subscriber("docklink/statusRequest",
                         Bool,
                         self.sendStatus,
                         queue_size=1)

        # Publishers
        self.status_pub = rospy.Publisher(
            "docklink/statusPublish",
            Bool,
            queue_size=1)

    def run(self):
        rospy.spin()
        return

    def sendStatus(self, msg):
        newMsg = 1
        self.status_pub.publish(newMsg)

if __name__ == '__main__':
    dock_node = dock_node()
    dock_node.run()
    print("Dock Node running ...")
