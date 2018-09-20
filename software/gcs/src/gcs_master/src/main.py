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
from geometry_msgs.msg import Point
# Local libraries
import drone_fsm


class GcsMasterNode():

    def __init__(self):
        # Subscribers configuration
        rospy.Subscriber("/dfb/ui", Point, self.ui_callback, queue_size=1)
        rospy.Subscriber("/dfb/gcs/dronelink", Point, self.dronelink_callback,
                         queue_size=1)
        rospy.Subscriber("/dfb/gcs/path_planner", Point, self.planner_callback,
                         queue_size=1)
        # Publishers configuration
        self.pos_publisher = rospy.Publisher("dfb/gcs/master/current_position",
                                             Point, queue_size=1)
        # Create an instance of the drone finite-state-machine class.
        self.state_machine = drone_fsm.DroneFSM()

    def ui_callback(self):
        return

    def dronelink_callback(self):
        return

    def planner_callback(self):
        return

    def run(self):
        """ 
        Main loop. Update the FSM and publish variables.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Update the state of the FSM.
            self.state_machine.update()
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
