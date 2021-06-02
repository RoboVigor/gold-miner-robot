#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from node_bridge_ros.srv import send_jointCalibration, send_jointCalibrationRequest


class JointCalibrationServer():

    def __init__(self):
        self._node = rospy.init_node('joint_calibration', log_level=rospy.INFO)
        # subsciber
        self._position = None
        rospy.wait_for_message('/joint_states', JointState)
        rospy.Subscriber("/joint_states", JointState, self._on_joint_state_received)
        # service proxy
        self._rate = rospy.Rate(1)
        rospy.wait_for_service('send_jointCalibration')
        self._calibration_data = send_jointCalibrationRequest()
        self._send_jointCalibration = rospy.ServiceProxy('send_jointCalibration', send_jointCalibration)
        # loop
        while True:
            if self._position:
                self._nb_calibrate()
                self._rate.sleep()

    def _on_joint_state_received(self, states):
        self._position = states.position
    
    def _nb_calibrate(self):
        position_factor = 1/3.14/2*8192
        self._calibration_data.base_joint_position = int(self._position[0]*position_factor)
        self._calibration_data.shoulder_joint_position = int(self._position[1]*position_factor)
        self._calibration_data.elbow_joint_position = int(self._position[2]*position_factor)
        self._calibration_data.wrist_joint_1_position = int(-1*self._position[3]*position_factor)
        self._calibration_data.wrist_joint_2_position = int(self._position[4]*position_factor)
        self._send_jointCalibration(self._calibration_data)
        rospy.loginfo(self._calibration_data)


if __name__ == '__main__':
    server = JointCalibrationServer()
    rospy.spin()
