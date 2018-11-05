#!/usr/bin/env python2
"""
This module is meant to test the drone_fsm module from gcs_master.

It has to be run with python2, as the library rostest is only compatible
with that version.
"""
# Standard libraries 
import unittest
# Third-party libraries
import rospy
# Local libraries
from gcs_master.drone_fsm import DroneFSM
try:
    import mavlink_lora.msg
except ModuleNotFoundError:
    print("Mavlink module not found")


## DroneFSM state transitions test suite.
class TestFSMTransitions(unittest.TestCase):
    def setUp(self):
        self.dronefsm = DroneFSM()
        self.dronefsm.TAKEOFF_ALTITUDE = 25

    def tearDown(self):
        del self.dronefsm

    def test_default_state(self):
        """
        Test the default outputs and state.
        """
        self.assertFalse(self.dronefsm.ARM)
        self.assertFalse(self.dronefsm.TAKE_OFF)
        self.assertFalse(self.dronefsm.LAND)
        self.assertFalse(self.dronefsm.FLY)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.NEW_MISSION)
        self.assertFalse(self.dronefsm.EMERGENCY_LANDING)
        self.assertFalse(self.dronefsm.WAYPOINT_REACHED)
        self.assertEqual(self.dronefsm.get_state(), "start")
    
    def test_start_to_arm_transition(self):
        """
        Test the transition from the start to the armed state.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "arm")
        self.assertFalse(self.dronefsm.new_mission)
        self.assertTrue(self.dronefsm.ARM)
        self.assertTrue(self.dronefsm.CALCULATE_PATH)
        self.assertTrue(self.dronefsm.TAKE_OFF)
    
    def test_start_to_disarmed_transition(self):
        """
        Test the transition:
        """
        self.assertTrue(False)

    def test_start_to_takingoff_transition(self):
        """
        Test the transition: start-->arm-->taking_off.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "taking_off")

    def test_start_to_flying_transition(self):
        """
        Test the transition: start-->...-->taking_off-->flying.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        self.dronefsm.relative_alt = 30
        self.dronefsm.update_fsm()
        # Add a mission of 1 item to the FSM route
        item = mavlink_lora.msg.mavlink_lora_mission_item_int()
        item.x = 10
        item.y = 10
        item.z = 50
        msg = mavlink_lora.msg.mavlink_lora_mission_list()
        # More than oone waypoint is needed to avoid a transition to landing.
        msg.waypoints.append(item)
        msg.waypoints.append(item)
        self.dronefsm.route = msg.waypoints
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "flying")
        self.assertTrue(self.dronefsm.new_path)
        self.assertFalse(self.dronefsm.taking_off)

    def test_start_to_upload_mission_from_flying(self):
        """
        Test the transition: start-->...-->flying-->upload_mission.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        self.dronefsm.relative_alt = 30
        self.dronefsm.update_fsm()
        # Add a mission of 1 item to the FSM route
        item = mavlink_lora.msg.mavlink_lora_mission_item_int()
        item.x = 10
        item.y = 10
        item.z = 50
        msg = mavlink_lora.msg.mavlink_lora_mission_list()
        # More than oone waypoint is needed to avoid a transition to landing.
        msg.waypoints.append(item)
        msg.waypoints.append(item)
        self.dronefsm.route = msg.waypoints
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "upload_mission")
        self.assertTrue(self.dronefsm.NEW_MISSION)
    
    def test_start_to_calculate_path(self):
        """
        Test the transition: start-->...-->flying-->calculate_path.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        self.dronefsm.relative_alt = 30
        self.dronefsm.update_fsm()
        # Add a mission of 1 item to the FSM route
        item = mavlink_lora.msg.mavlink_lora_mission_item_int()
        item.x = 10
        item.y = 10
        item.z = 50
        msg = mavlink_lora.msg.mavlink_lora_mission_list()
        # More than oone waypoint is needed to avoid a transition to landing.
        msg.waypoints.append(item)
        msg.waypoints.append(item)
        self.dronefsm.route = msg.waypoints
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Note: The new_path variable will never set to false in a real example.
        # It is forced here for getting to the state faster.
        self.dronefsm.new_path = False
        self.dronefsm.new_waypoint = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "calculate_path")
        self.assertTrue(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.new_path)
    
    def test_start_to_upload_mission_from_calculate_path(self):
        """
        Test the transition: start-->...-->calc_path-->upload_mission.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        self.dronefsm.relative_alt = 30
        self.dronefsm.update_fsm()
        # Add a mission of 1 item to the FSM route
        item = mavlink_lora.msg.mavlink_lora_mission_item_int()
        item.x = 10
        item.y = 10
        item.z = 50
        msg = mavlink_lora.msg.mavlink_lora_mission_list()
        # More than oone waypoint is needed to avoid a transition to landing.
        msg.waypoints.append(item)
        msg.waypoints.append(item)
        self.dronefsm.route = msg.waypoints
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Note: The new_path variable will never set to false in a real example.
        # It is forced here for getting to the state faster.
        self.dronefsm.new_path = False
        self.dronefsm.new_waypoint = True
        self.dronefsm.update_fsm()
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "upload_mission")
        self.assertTrue(self.dronefsm.NEW_MISSION)
        self.assertFalse(self.dronefsm.new_mission)

    def test_start_to_flying_from_upload_mission(self):
        """
        Test the transition: start-->...-->upload_mission-->flying.
        """
        # Actions for getting to the desired state.
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = True
        self.dronefsm.taking_off = True
        self.dronefsm.update_fsm()
        self.dronefsm.relative_alt = 30
        self.dronefsm.update_fsm()
        # Add a mission of 1 item to the FSM route
        item = mavlink_lora.msg.mavlink_lora_mission_item_int()
        item.x = 10
        item.y = 10
        item.z = 50
        msg = mavlink_lora.msg.mavlink_lora_mission_list()
        # More than oone waypoint is needed to avoid a transition to landing.
        msg.waypoints.append(item)
        msg.waypoints.append(item)
        self.dronefsm.route = msg.waypoints
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Note: The new_path variable will never set to false in a real example.
        # It is forced here for getting to the state faster.
        self.dronefsm.new_path = False
        self.dronefsm.new_waypoint = True
        self.dronefsm.update_fsm()
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        self.dronefsm.new_mission = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "flying")
        self.assertFalse(self.dronefsm.new_mission)
        self.assertFalse(self.dronefsm.new_path)

    def test_start_to_recovercomm_transition(self):
        """
        Test the transition: start-->...-->flying-->recover_comm.
        """
        self.assertFalse(True)

    def test_start_to_calculate_path_from_recovercomm(self):
        """
        Test the transition: start-->...-->recover_comm-->calculate_path
        """
        self.assertFalse(True)

    def test_start_to_landing(self):
        """
        Test the transition: start-->...-->flying-->landing.
        """
        # Actions for getting to the desired state.
        self.assertFalse(True)

    def test_start_to_start(self):
        """
        Test the transition: start-->...-->landing-->start.
        """
        self.assertFalse(True)


## DroneFSM class test
class TestFSMBehaviours(unittest.TestCase):
    def setUp(self):
        self.dronefsm = DroneFSM()

    def tearDown(self):
        del self.dronefsm

    def test_new_waypoint(self):
        # Actions for getting to the desired state.
        self.dronefsm.armed = True
        self.dronefsm.update_fsm()
        self.dronefsm.destination = [0, 0]
        self.dronefsm.batt_ok = True
        self.dronefsm.comm_ok = True
        self.dronefsm.update_fsm()
        self.dronefsm.route = [[0,0], [50,50], [100,100]]
        self.dronefsm.update_fsm()
        self.dronefsm.height = 55
        self.dronefsm.update_fsm()
        # Action for reaching a new waypoint
        self.new_waypoint = True
        self.dronefsm.update_fsm()
        # Check the expected behaviour
        self.assertTrue(self.dronefsm.CALCULATE_PATH)


if __name__ == '__main__':
    rospy.init_node("test_fsm_node", anonymous=True)
    import rostest
    rostest.rosrun("gcs_master", "fsm_test", TestFSMTransitions)
