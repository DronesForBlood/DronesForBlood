#!/usr/bin/env python3
# Standard libraries 
import unittest
import sys
# Local libraries
from src.drone_fsm import DroneFSM


## DroneFSM class test
class TestDroneFSM(unittest.TestCase):
    def setUp(self):
        self.dronefsm = DroneFSM()

    def test_one_equals_one(self):
        self.assertEqual(1, 1, "1!=1")

if __name__ == '__main__':
    unittest.main()
