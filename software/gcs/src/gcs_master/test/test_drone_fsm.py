#!/usr/bin/env python2
"""
This module is meant to test the drone_fsm module from gcs_master.

It has to be run with python2, as the library rostest is only compatible
with that version.
"""
# Standard libraries 
import unittest
import sys
# Third-party libraries
import rospy
# Local libraries
from gcs_master.drone_fsm import DroneFSM


## DroneFSM state transitions test suite.
class TestFSMTransitions(unittest.TestCase):
    def setUp(self):
        self.dronefsm = DroneFSM()

    def tearDown(self):
        del self.dronefsm

    def test_default_state(self):
        """
        Test the default outputs and state.
        """
        self.assertFalse(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.TAKE_OFF)
        self.assertFalse(self.dronefsm.LAND)
        self.assertFalse(self.dronefsm.FLY)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.EMERGENCY_LANDING)
        self.assertEqual(self.dronefsm.get_state(), "start")
    
    def test_start_to_armed_transition(self):
        """
        Test the transition from the start to the armed state.
        """
        # Actions for getting to the desired state.
        self.dronefsm.armed = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "armed")
        self.assertTrue(self.dronefsm.ARMED)
    
    def test_start_to_disarmed_transition(self):
        """
        Test the transition: start-->armed-->start.
        """
        # Actions for getting to the desired state.
        self.dronefsm.armed = True
        self.dronefsm.update_fsm()
        self.dronefsm.armed = False
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "start")
        self.assertFalse(self.dronefsm.ARMED)

    def test_start_to_newplan_transition(self):
        """
        Test the transition: start-->armed-->new_plan.
        """
        # Actions for getting to the desired state.
        self.dronefsm.armed = True
        self.dronefsm.update_fsm()
        self.dronefsm.destination = [0, 0]
        self.dronefsm.batt_ok = True
        self.dronefsm.comm_ok = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "new_plan")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertTrue(self.dronefsm.CALCULATE_PATH)

    def test_start_to_takingoff_transition(self):
        """
        Test the transition: start-->armed-->new_plan-->taking_off.
        """
        # Actions for getting to the desired state.
        self.dronefsm.armed = True
        self.dronefsm.update_fsm()
        self.dronefsm.destination = [0, 0]
        self.dronefsm.batt_ok = True
        self.dronefsm.comm_ok = True
        self.dronefsm.update_fsm()
        self.dronefsm.route = [[0,0], [50,50], [100,100]]
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "taking_off")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertTrue(self.dronefsm.TAKE_OFF)
        
    def test_start_to_flying_transition(self):
        """
        Test the transition: start-->...-->taking_off-->flying.
        """
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
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "flying")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_recovercomm_transition(self):
        """
        Test the transition: start-->...-->flying-->recover_comm.
        """
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
        self.dronefsm.comm_ok = False
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "recover_comm")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_landed_from_lostcomm_transition(self):
        """
        Test the transition: start-->...-->recover_comm-->landed.
        """
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
        self.dronefsm.comm_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.comm_ok = True
        self.dronefsm.on_air = False
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "landed")
        self.assertFalse(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_newpath_from_lostcomm_transition(self):
        """
        Test the transition: start-->...-->recover_comm-->new_path.
        """
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
        self.dronefsm.comm_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.comm_ok = True
        self.dronefsm.on_air = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "new_path")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertTrue(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_flying_from_lostcomm_transition(self):
        """
        Test transition: start-->...-->recover_comm-->new_path-->flying.
        """
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
        self.dronefsm.comm_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.comm_ok = True
        self.dronefsm.on_air = True
        self.dronefsm.update_fsm()
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "flying")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_newdest_transition(self):
        """
        Test the transition: start-->...-->flying-->new_destination.
        """
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
        self.dronefsm.batt_ok = False
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "new_destination")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_emergencylanding_transition(self):
        """
        Test transition: start-->...-->new_destination-->emerg_landing.
        """
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
        self.dronefsm.batt_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.distance_to_station = 110
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "emergency_landing")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)
        self.assertTrue(self.dronefsm.EMERGENCY_LANDING)

    def test_start_to_newpath_from_lowbatt_transition(self):
        """
        Test the transition: start-->...-->new_destination-->new_path.
        """
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
        self.dronefsm.batt_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.distance_to_station = 90
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "new_path")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertTrue(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_newpath_from_lowbatt_transition(self):
        """
        Test the transition: start-->...-->new_destination-->new_path.
        """
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
        self.dronefsm.batt_ok = False
        self.dronefsm.update_fsm()
        self.dronefsm.distance_to_station = 90
        self.dronefsm.update_fsm()
        self.dronefsm.new_path = True
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "flying")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_landing_transition(self):
        """
        Test the transition: start-->...-->flying-->landing.
        """
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
        self.dronefsm.route = []
        self.dronefsm.update_fsm()
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "landing")
        self.assertTrue(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)

    def test_start_to_landed_transition(self):
        """
        Test the transition: start-->...-->landing-->landed.
        """
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
        self.dronefsm.route = []
        self.dronefsm.update_fsm()
        self.dronefsm.height = 0.1
        # Check the state and relevant outputs.
        self.assertEqual(self.dronefsm.get_state(), "landed")
        self.assertFalse(self.dronefsm.ARMED)
        self.assertFalse(self.dronefsm.CALCULATE_PATH)
        self.assertFalse(self.dronefsm.TAKE_OFF)


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
