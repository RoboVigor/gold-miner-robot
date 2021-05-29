#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from node_bridge_ros.msg import jointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from gm_driver.joint_state_publisher import JointStatePublisher
from node_bridge import bridge, protocol


class JointTrajectoryActionServer():

    def __init__(self, fake_execution=False):
        self._node = rospy.init_node(
            'joint_trajectory_action', log_level=rospy.INFO)
        self._feedback = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        self._server = actionlib.SimpleActionServer(
            'default_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action)
        self._action_name = rospy.get_name()
        self._position = None
        self._position = [0, 0, 0, 0, 0]
        rospy.Subscriber("/node_bridge/jointState",
                         jointState, self._on_nb_received)
        self._nb_bridge = bridge.NodeBridge('serial', port=0)
        self._control_data = protocol.create_protocol_data('chassisData')
        self._packet_sequence = 0
        # fake_execution
        self._fake_execution = fake_execution
        if fake_execution:
            self._pub = JointStatePublisher(fake_execution=True)

    def _on_trajectory_action(self, goal):
        self._joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        num_points = len(trajectory_points)
        # rospy.loginfo(self._joint_names)
        # rospy.loginfo(trajectory_points)
        rospy.loginfo([x.time_from_start.to_sec() for x in trajectory_points])
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
        else:
            if self._check_start_point(trajectory_points[0].positions) is not None:
                return
            rospy.loginfo("%s: Executing Trajectory" % (self._action_name,))
            rate = rospy.Rate(125)
            total_start_time = rospy.get_time()
            for i in range(num_points):
                last_goal = trajectory_points[max(i-1, 0)]
                current_goal = trajectory_points[i]
                last_goal_time = last_goal.time_from_start.to_sec()
                goal_time = current_goal.time_from_start.to_sec()
                goal_duration = goal_time - last_goal_time
                start_time = rospy.get_time()
                time_passed = 0
                while time_passed < goal_duration:
                    linear_position = self._calc_ramp(
                        trajectory_points[i-1].positions,
                        current_goal.positions,
                        time_passed / goal_duration
                    )
                    self._nb_set_position(linear_position)
                    rate.sleep()
                    time_passed = rospy.get_time() - start_time
                    self._update_feedback(current_goal.positions, time_passed)
                rospy.loginfo("%s: Finish executing position %i at %f" %
                              (self._action_name, i, rospy.get_time()-total_start_time))
            self._set_succeeded()

    def _check_start_point(self, start_point):
        if not self._position:
            return self._set_aborted(self._result.INVALID_GOAL,
                                     'Joint states not received')
        elif sum(self._calc_array_diff(start_point, self._position)) > 3.14*10/180:
            return self._set_aborted(self._result.INVALID_GOAL,
                                     'Starting point not match')

    def _calc_array_diff(self, array_a, array_b):
        return [array_a[i] - array_b[i] for i in range(len(array_a))]

    def _calc_ramp(self, array_a, array_b, progress):
        return [array_a[i]*(1-progress) + array_b[i]*progress for i in range(len(array_a))]

    def _update_feedback(self, goal_position, time_passed):
        self._feedback.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._feedback.joint_names = self._joint_names
        self._feedback.desired.positions = goal_position
        self._feedback.actual.positions = self._position
        self._feedback.error.positions = self._calc_array_diff(
            self._feedback.desired.positions, self._feedback.actual.positions)
        time_from_start = rospy.Duration.from_sec(time_passed)
        self._feedback.desired.time_from_start = time_from_start
        self._feedback.actual.time_from_start = time_from_start
        self._feedback.error.time_from_start = time_from_start
        self._server.publish_feedback(self._feedback)

    def _set_aborted(self, code, reason):
        rospy.logerr("%s: %s." % (self._action_name, reason))
        self._result.error_code = code
        self._result.error_string = reason
        self._server.set_aborted(self._result)
        self._nb_stop()
        return self._result

    def _set_succeeded(self):
        rospy.loginfo("%s: Joint Trajectory Action Succeeded" %
                      (self._action_name))
        self._result.error_code = self._result.SUCCESSFUL
        self._server.set_succeeded(self._result)
        self._nb_stop()

    def _on_nb_received(self, data):
        position_factor = 1/8192*3.14
        self._position = [data.base_joint_position*position_factor, data.shoulder_joint_position*position_factor,
                          data.elbow_joint_position*position_factor, data.wrist_joint_1_position*position_factor, data.wrist_joint_2_position*position_factor]

    def _nb_set_position(self, goal_position):
        if self._fake_execution:
            self._position = goal_position
            self._pub.publish_position(goal_position)
        else:
            self._control_data['base_joint_position'] = goal_position[0]
            self._control_data['shoulder_joint_position'] = goal_position[1]
            self._control_data['elbow_joint_position'] = goal_position[2]
            self._control_data['wrist_joint_1_position'] = goal_position[3]
            self._control_data['wrist_joint_2_position'] = goal_position[4]
            control_packet = protocol.pack('jointControl', self._control_data, seq=self._packet_sequence)
            self._packet_sequence += 1
            self._bridge.send(control_packet)

    def _nb_stop(self):
        pass


if __name__ == '__main__':
    publisher = JointTrajectoryActionServer()
    rospy.spin()
