#!/usr/bin/env python3

import unittest
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
from my_publisher_node import MyPublisherNode

class TestMyPublisherNode(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_node')
        self.node = MyPublisherNode('test_node')

    def tearDown(self):
        self.node.on_shutdown()

    def test_callback(self):
        # Test that the range value is updated when the callback function is called
        range_msg = Range()
        range_msg.range = 10.0
        self.node.callback(range_msg)
        self.assertEqual(self.node.range, 10.0)

    def test_rightwheel(self):
        # Test that the right wheel encoder value is updated when the callback function is called
        wheel_msg = WheelEncoderStamped()
        wheel_msg.data = 100
        self.node.rightwheel(wheel_msg)
        self.assertEqual(self.node.right, 100)

    def test_leftwheel(self):
        # Test that the left wheel encoder value is updated when the callback function is called
        wheel_msg = WheelEncoderStamped()
        wheel_msg.data = 50
        self.node.leftwheel(wheel_msg)
        self.assertEqual(self.node.left, 50)

if __name__ == '__main__':
    unittest.main()
