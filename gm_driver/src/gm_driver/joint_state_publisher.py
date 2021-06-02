#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from node_bridge_ros.msg import jointState
from sensor_msgs.msg import JointState
from serial_imu.msg import Imu_0x91_msg


class JointStatePublisher():

    def __init__(self, fake_execution=False):
        if not fake_execution:
            self._node = rospy.init_node(
                'gm_joint_state_publisher', log_level=rospy.INFO)
        self._pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self._euler_data1 = {'yaw':0, 'roll':0, 'pitch':0}
        self._euler_data2 = {'yaw':0, 'roll':0, 'pitch':0}
        self._position = [0, 0, 0, 0, 0]
        rospy.Subscriber("/imu_raw_data1",
                         Imu_0x91_msg, self._on_imu1_received)
        rospy.Subscriber("/imu_raw_data2",
                         Imu_0x91_msg, self._on_imu2_received)
        if not fake_execution:
            rospy.Subscriber("/node_bridge/jointState",
                             jointState, self._on_nb_received)

    def _on_nb_received(self, data):
        position_factor = 1/8192*3.14*2
        self._position = [data.base_joint_position*position_factor, data.shoulder_joint_position*position_factor,
                                    data.elbow_joint_position*position_factor, data.wrist_joint_1_position*position_factor, data.wrist_joint_2_position*position_factor]
        self.publish_position()

    def publish_position(self):
        jointstate_data = JointState()
        jointstate_data.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        jointstate_data.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint_1', 'wrist_joint_2']
        jointstate_data.position = [0]*5
        jointstate_data.position[0] = 0
        jointstate_data.position[1] = 3.14/180*37
        jointstate_data.position[2] = self._euler_data2['pitch']-jointstate_data.position[1]
        jointstate_data.position[3] = -1*self._position[3]
        jointstate_data.position[4] = self._euler_data2['pitch']-self._euler_data1['pitch']
        self._pub.publish(jointstate_data)

    def _on_imu1_received(self, data):
        deg2rad = 1/180*3.14
        self._euler_data1['roll'] =  data.eul_r * deg2rad
        self._euler_data1['pitch'] =  data.eul_p * deg2rad
        self._euler_data1['yaw'] =  data.eul_y * deg2rad
        self.publish_position()

    def _on_imu2_received(self, data):
        deg2rad = 1/180*3.14
        self._euler_data2['roll'] =  data.eul_r * deg2rad
        self._euler_data2['pitch'] =  data.eul_p * deg2rad
        self._euler_data2['yaw'] =  data.eul_y * deg2rad
        self.publish_position()


if __name__ == '__main__':
    publisher = JointStatePublisher()
    rospy.spin()
