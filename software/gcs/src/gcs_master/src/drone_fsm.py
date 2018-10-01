#!/usr/bin/env python3
"""
Finite-State-Machine (FSM) class implementing the drone behaviour.
"""
# Standard libraries

from std_msgs import Empty
import rospy

class DroneFSM():

    def __init__(self, max_lowbatt_distance=100):
        """
        :param int max_lowbatt_distance: maximum distance in meters that
         can be covered by the drone after entering low battery mode.
        """
        # FSM flags. Outputs
        self.ARMED = False
        self.TAKE_OFF = False
        self.LAND = False
        self.FLY = False
        self.CALCULATE_PATH = False
        self.EMERGENCY_LANDING = False
        # Drone parameters. FSM inputs
        self.height = 0				# Altitude
        self.position = [None, None]	 	# Current position
        self.destination = [None, None]	 	# Next waypoint
        self.route = [] 			        # Entire path
        self.distance_to_station = 0	        # Remaining distance?
        self.armed = False			        # Armed / Disarmed
        self.batt_ok = False			# Battery status
        self.comm_ok = False			# Comlink status
        self.on_air = False			# Whether it is in the air?
        self.new_waypoint = False		# ?? If new waypoint is available?
        self.new_path = False			# ?? If new path is available?
        self.max_lowbatt_distance = max_lowbatt_distance
        # FSM parameters
        self.__state = "start"
	self.__substate = 0
        self.msg = 0            # Message type, route, waypoint, position etc.

        # ROS Communication
        rospy.init_node('GCS', anonymous=False)

        # Pathplanner Topics
        self.path_request = rospy.Publisher('pathreq', Boolean, queue_size=10)
	self.path_sub = rospy.Subscriber('path', String, main)	

	# MavLink Topics
	self.mavlink_drone_arm = rospy.Publisher('mavlink/drone/arm', Boolean, queue_size=1) # True = Arm, False = Disarm
	self.mavlink_drone_takeoff = rospy.Publisher('mavlink/drone/takeoff', std_msgs.msg.Empty, queue_size=1)
	self.mavlink_drone_start_mission = rospy.Publisher('mavlink/drone/start', std_msgs.msg.Empty, queue_size=1)
	self.mavlink_drone_error = rospy.Publisher('mavlink/drone/error', std_msgs.msg.Empty, queue_size=1)

	self.mavlink_mission_upload = rospy.Publisher('mavlink/mission/mavlink_upload_mission', ???, queue_size=1)
	self.mavlink_mission_clear_all = rospy.Publisher('mavlink/mission/clear_all', std_msgs.msg.Empty, queue_size=1)
	self.mavlink_mission_download = rospy.Publisher('mavlink/mission/download', std_msgs.msg.Empty, queue_size=1)	
			 
	self.mavlink_mission_ack = rospy.Subscriber('mavlink/mission/ack', ??) # Mission related acknowledge (Plan uploaded etc.)
	self.mavlink_mission_response = rospy.Subscriber('mavlink/mission/response', ??) # MavLink route download channel	
	self.mavlink_drone_ack = rospy.Subscriber('mavlink/drone/ack', ??)	# Drone related acknowledge (Arm/Disarm etc.)

    def main(self, data):
        # Read message
        self.msg = data

        # Run / Update state machine
        update_state()                        # Get into correct state
        #update_outputs()                     # Output based on current state

        rospy.spin()                  # Keeps node from exiting until shutdown

    def update_state(self):
        """
        Update the state of the FSM, based on the state variables.
        """
        # Check for errors first!
        if self.msg._connection_header["topic"] == "/mavlink/drone/error":
            if self.msg = "BATTERY_ERROR":                  # Battery error
                self.__state = "emergency_landing"

            elif self.msg = "RC_LINK_LOSS":                 # Commlink error
                self.__state = "recover_comm"

            elif self.msg = "SYSTEM ERROR":                # Undefined system error
                self.__state = "Shit's fucked yo"

        # START state
        if self.__state == "start":
	    if self.msg._connection_header["topic"] == "/path": # If message comes from path topic, read route        
                self.route = self.msg           		 # Read route
		self.mavlink_mission_clear_all.publish(std_msg.msg.Empty)  # Clear current route
 	    
	    # New Route workflow
	    # Clear current route
	    # Upload new route
	    # Download route from drone
	    # Compare the uploaded and downloaded - Upload issues can go undetected without this step. 
	    # Start/Resume mission

    	    elif self.msg._connection_header["topic"] == "/mavlink/mission/ack": # If response if is mission ack
		# Substates
		if self.__substate = 0 	# Route has been received
		    if self.msg = 1:	# Waypoint clear ack. 	    
			self.msg = 0			
			self.__substate = 10		
		    else: 
			self.mavlink_mission_clear_all.publish(std_msg.msg.Empty)  # Repeat command	
	
		elif self.__substate = 10 # Waypoints have been cleared.
		    if self.msg = 1: 	  # Upload command ack. 
		        self.msg = 0
			self.__substate = 20
		    else: 
		        self.mavlink_mission_upload.publish(self.msg)  # Send route to MavLink				
		    
		elif self.__substate = 20 # Route has been uploaded.
		    self.mavlink_mission_download(std_msg.msg.Empty)	   # Download accepted route
		
	    elif self.msg._connection_header["topic"] == "/mavlink/mission/response": # Route has been downloaded
		if self.route = self.msg:   # Compare routes!!
		    self.mavlink_drone_arm.publish(True)	
		else: 
		    print ("Incorrect path uploaded!")			    
	  
	    elif self.msg._connection_header["topic"] == "/mavlink/drone/ack":	 # Reponse from arm request
		if self.msg == 1:
		    self.msg = 0  		  # Clear message before next state. 
		    self.armed = True
		    self.__substate = 0		  # Reset substate
		    self.__state = "armed"        # Proceed to next state.
		else:
		    pass

	    else: # This will be replaced by a UserLink command
                path_request.publish(True)        # Ask for new path. Will happend when this state is entered first time.

        # ARMED state
        elif self.__state == "armed":	
            if self.msg._connection_header["topic"] == "/mavlink/drone/ack":     # If MavLink acknowledges, proceed to next state
	        if self.msg == 1:	    
		    self.msg = 0		# Clear msg before next state.
                    self.__state = "taking_off"
      	        else:
		    pass # What to do?
	     else:           		      # Ask MavLink to take off/start mission
       	        self.mavlink_drone_takeoff.publish(str("")) # Empty string, find correct format

        # TAKING OFF state - NECESSARY?
        elif self.__state == "taking_off":
            if self.msg._connection_header["topic"] == "/mavlink/drone/ack":             # HOW TO DETECT TAKEOFF?
                if self.msg == 1: 
		    self.msg = 0
		    self.__state = "flying"
            else:
                pass

        # FLYING state
        elif self.__state == "flying":
            if self.msg = 3:   # If destination is reached
                print ("Destination reached")
		self.msg = 0
                self.__state = "landing"

            elif self.msg._connection_header["topic"] == "/path":  # If message comes from path topic, read route
                self.route = self.msg    # Read route
		self.__substate = 0
                self.mavlink_mission_clear_all.publish(std_msg.msg.Empty)  # Clear current route

	    elif self.msg._connection_header["topic"] == "/mavlink/mission/ack": # If response if is mission ack
		if self.__substate = 0 	# Route has been received
		    if self.msg = 1:	# Waypoint clear ack. 	    
			self.msg = 0			
			self.__substate = 10		
		    else: 
			self.mavlink_mission_clear_all.publish(std_msg.msg.Empty)  # Repeat command	
	
		elif self.__substate = 10 # Waypoints have been cleared.
		    if self.msg = 1: 	  # Upload command ack. 
		        self.msg = 0
			self.__substate = 20
		    else: 
		        self.mavlink_mission_upload.publish(self.msg)  # Send route to MavLink				
		    
		elif self.__substate = 20 # Route has been uploaded.
		    self.mavlink_mission_download(std_msg.msg.Empty)	   # Download accepted route
		
	    elif self.msg._connection_header["topic"] == "/mavlink/mission/response": # Route has been downloaded
		if self.route = self.msg:   # Compare routes!!
		    # Some sort of resume command? Needs testing
		else: 
		    print ("Incorrect path uploaded!")		

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            #If we're here, drone has landed, unless we can abort?
            # Go to take-off state?
            pass

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            # What now?
            pass

        # LANDING state - NECESSARY? 
        elif self.__state == "landing":
            #if self.msg = DRONE LANDED
                self.__state = "landed"
	    #else:
		#pass

        # LANDED state
        elif self.__state == "landed":
            self.route = null     # Remove previous route
	    if self.msg._connection_header["topic"] == "/userlink/start":
		if self.msg = 1:
		    print("Go to start") 
		    # Something something, goto start
	    else:
            	pass

        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))

        return self.__state

    def update_outputs(self):
        """
        Update the output variables of the FSM.
        """
        # START state
        if self.__state == "start":
            pass

        # ARMED state
        elif self.__state == "armed":
            self.ARMED = True

        # NEW PLAN state
        elif self.__state == "new_plan":
            self.CALCULATE_PATH = True

        # TAKING OFF state
        elif self.__state == "taking_off":
            self.CALCULATE_PATH = False
            self.TAKE_OFF = True

        # FLYING state
        elif self.__state == "flying":
            self.TAKE_OFF = False
            self.CALCULATE_PATH = False

        # RECOVER COMM state
        elif self.__state == "recover_comm":
            pass

        # NEW DESTINATION state
        elif self.__state == "new_destination":
            pass

        # NEW PATH state
        elif self.__state == "new_path":
            self.CALCULATE_PATH = True

        # EMERGENCY LANDING state
        elif self.__state == "emergency_landing":
            self.EMERGENCY_LANDING = True

        # LANDING state
        elif self.__state == "landing":
            pass

        # LANDED state
        elif self.__state == "landed":
            pass

        # Non-valid state
        else:
            raise ValueError("Unrecognized state '{}'".format(self.__state))
        return

