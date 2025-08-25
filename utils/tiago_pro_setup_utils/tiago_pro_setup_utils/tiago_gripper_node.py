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
            '/gripper_right_controller/follow_joint_trajectory'
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
        goal_msg.trajectory.joint_names = ['gripper_right_finger_joint']

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

# #!/usr/bin/env python3

# import time
# import threading

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint


# class TiagoRightGripper(Node):
#     def __init__(self, name='tiago_right_gripper_node'):
#         super().__init__(name)

#         self.logger = self.get_logger()

#         # Action client per il gripper destro
#         self.gripper_client = ActionClient(
#             self, FollowJointTrajectory,
#             "/gripper_right_controller/follow_joint_trajectory"
#             ,
#         )

#     def gripper_action(self, position, duration=2.0):
#         msg = FollowJointTrajectory.Goal()
#         msg.trajectory.joint_names = ["gripper_right_finger_joint"]

#         point = JointTrajectoryPoint()
#         point.positions = [position]
#         point.time_from_start.sec = int(duration)
#         point.time_from_start.nanosec = 0

#         msg.trajectory.points = [point]

#         self.logger.info(f"Sending goal: {position}")
#         return self.gripper_client.send_goal_async(msg)

#     def close_gripper(self):
#         self.logger.info("Closing gripper")
#         return self.gripper_action(0.0)

#     def open_gripper(self):
#         self.logger.info("Opening gripper")
#         return self.gripper_action(0.8)


# def main():
#     rclpy.init()
#     node = TiagoRightGripper()
#     node.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

#     # Spin in background
#     # spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     # spin_thread.start()

#     # Attendi action server
#     node.logger.info("Waiting for gripper action server...")
#     while not node.gripper_client.wait_for_server(timeout_sec=1.0):
#         node.logger.info("Still waiting...")

#     # Test: chiudi e apri gripper
#     # future = node.close_gripper()
#     # rclpy.spin_until_future_complete(node, future)
#     # node.logger.info("Closing completed.")

#     future = node.open_gripper()
#     rclpy.spin_until_future_complete(node, future)

#     node.logger.info("Opening completed.")

#     node.destroy_node()
#     rclpy.shutdown()
#     # spin_thread.join()


# if __name__ == "__main__":
#     main()
