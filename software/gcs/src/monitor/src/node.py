# imports
import rospy
import struct
from std_msgs.msg import Int8

class monitor:
    def __init__(self):

        # status variables
		self.last_updated = 0

        # launch node
        rospy.init_node('monitor', disable_signals=True)

        # Services - Nodes to monitor

        # shutdown handler
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(update_interval)
        rospy.sleep(1)  # wait until everything is running
    
    def fetchStatus:
        do = "something"
        # fix me

    def update_display (self):
		now = rospy.get_time()
		print '\033[2J' # clear screen
		print '', # go home

		# update last_update - When was it last updated. Has to be updated here due to time flow
		t = now - self.last_update
		if t < 86400:
			last_update_text = '%ds' % t
		else:
			last_update_text = 'Never'


		print '\033[1HLast heard:         %s' % last_heard_text
		print '\033[2HLast system status: %s' % last_heard_status_text
		print '\033[4HBattery:            %s' % batt_text
		print '\033[6HPosition:           %s' % pos_text
		print '\033[7HAltitude:           %s' % alt_text  
		print '\033[8HDistance:           %s' % home_text
		print '\033[10HAttitude:           %s' % atti_text
		print '\033[12HPress: h to set home position, q to quit'
		print '\033[?25l' # hide cursor

    def run(self):
        # loop until shutdown
        while not (rospy.is_shutdown()):
            # do stuff

            # sleep the defined interval
            self.rate.sleep()

    def shutdown(self):
        print("Monitor shut down successfully")


if __name__ == '__main__':
    node = monitor()
    node.run()


# Idea : 
# Every 10 sec run service call for status on every node. Update own variables and ui, handle error statuses