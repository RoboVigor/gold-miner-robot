#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from node_bridge_ros.msg import jointInfo
from sensor_msgs.msg import JointState


class JointStatePublisher():

    def __init__(self, fake=False):
        if not fake:
            self._node = rospy.init_node(
                'gm_joint_state_publisher', log_level=rospy.INFO)
        self._pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        if not fake:
            rospy.Subscriber("/node_bridge/jointInfo",
                             jointInfo, self._on_nb_received)

    def _on_nb_received(self, data):
        position_factor = 1/128*3.14
        jointstate_data.position = [data.base_joint_position*position_factor, data.shoulder_joint_position*position_factor,
                                    data.elbow_joint_position*position_factor, data.wrist_joint_1_position*position_factor, data.wrist_joint_2_position*position_factor]
        self.publish_position(jointstate_data)

    def publish_position(self, position):
        jointstate_data = JointState()
        jointstate_data.name = [
            'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        jointstate_data.position = position
        self._pub.publish(jointstate_data)


if __name__ == '__main__':
    publisher = JointStatePublisher()
    rospy.spin()
