#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action import GoalResponse, CancelResponse

from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperBridge(Node):
    def __init__(self):
        super().__init__('tiago_right_gripper_node')

        # Action server exposed to clients
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Action client to the real gripper
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_left_controller/follow_joint_trajectory'
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal: {goal_request.command.position:.3f}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Wait for the remote action server to be available
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action server not available!')
            goal_handle.abort()
            return GripperCommand.Result(position=0.0, effort=0.0, stalled=False, reached_goal=False)

        # Convert GripperCommand to FollowJointTrajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [goal_handle.request.command.position]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points = [point]

        # Send goal to the underlying FollowJointTrajectory action server
        self.get_logger().info(f'Sending to FollowJointTrajectory: {point.positions[0]:.3f}')
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        goal_response = await send_goal_future

        if not goal_response.accepted:
            self.get_logger().error('Goal rejected by FollowJointTrajectory')
            goal_handle.abort()
            return GripperCommand.Result(position=0.0, effort=0.0, stalled=False, reached_goal=False)

        # Wait for result
        result_future = goal_response.get_result_async()
        result = await result_future

        # At this point you could extract more detailed info from result
        # and fill GripperCommand.Result more precisely if needed
        self.get_logger().info('Goal completed successfully.')
        goal_handle.succeed()

        return GripperCommand.Result(
            position=goal_handle.request.command.position,
            effort=goal_handle.request.command.max_effort,
            stalled=False,
            reached_goal=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
