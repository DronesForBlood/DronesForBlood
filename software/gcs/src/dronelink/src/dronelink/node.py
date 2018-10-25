# parameters
mavlink_lora_pos_sub_topic = '/mavlink_pos'

# imports
import rospy
import struct
from std_msgs.msg import Int8
from mavlink_lora.msg import mavlink_lora_pos

class dronelink:
    def __init__(self):

        # Variables....

        # launch node
        rospy.init_node('dronelink', disable_signals=True)

        # Subs
        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)

        # Pubs

        # shutdown handler
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(update_interval)
        rospy.sleep(1)  # wait until everything is running

    def on_mavlink_lora_pos(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

        # what to do maaaaan


    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff

            # sleep the defined interval
            self.rate.sleep()

    def shutdown(self):
        print("Dronelink shut down successfully")


if __name__ == '__main__':
    node = dronelink()
    node.run()
