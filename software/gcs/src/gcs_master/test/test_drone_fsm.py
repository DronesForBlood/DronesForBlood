#!/usr/bin/env python3
# Standard libraries 
import unittest
import sys

PKG='test_foo'


## A sample python unit test
class TestBareBones(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    unittest.main()
