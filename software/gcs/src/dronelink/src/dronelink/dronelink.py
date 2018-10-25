#!/usr/bin/env python3
"""
Dronelink main module. Creates a websocket and starts comm with server.
"""
# Standard libraries
# Third-party libraries
import geometry_msgs.msg
import std_msgs.msg
import rospy
# Local libraries


class Dronelink(object):

    def __init__(self):
        # Subscribers configuration
        rospy.Subscriber("/dfb/master/current_pos", geometry_msgs.msg.Point,
                         self.curent_pos_callback, queue_size=1)
        # Publishers configuration
        self.start_publisher = rospy.Publisher("dronelink/start",
                                               std_msgs.msg.Bool, queue_size=1)
        self.dest_publisher = rospy.Publisher("dfb/gcs/dronelink/destination",
                                              geometry_msgs.msg.Point,
                                              queue_size=1)
        # Location variables
        self.current_pos = [None, None, None]
        self.destination = [None, None, None]

    def curent_pos_callback(self, data):
        """
        Get the current position of the drone from the gcs_master.
        """
        self.current_pos = [data.x, data.y, data.z]
        return

    def run(self):
        """
        Main loop. Check server periodically.
        """
        rate =rospy.Rate(10)
        while not rospy.is_shutdown():
            self.start_publisher.publish(True)
            rate.sleep()
        return


def main():
    dronelink = Dronelink()
    dronelink.run()
    return

