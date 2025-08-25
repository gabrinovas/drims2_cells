#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperCloseClient(Node):
    def __init__(self):
        super().__init__('gripper_close_client')

        # Crea action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_right_controller/follow_joint_trajectory'
        )

        # Aspetta che l'action server sia disponibile
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Invia goal di chiusura pinza
        self.send_goal()

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        # Specifica i nomi dei giunti della pinza
        goal_msg.trajectory.joint_names = ['right_gripper_finger_joint']

        # Definisci un punto target
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # chiusura pinza (adatta il valore in base al tuo robot)
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.effort = [0.0]
        
        point.time_from_start.sec = 2

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending goal to close gripper: {goal_msg}...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action finished, error_code: {result.error_code}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    node = GripperCloseClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
