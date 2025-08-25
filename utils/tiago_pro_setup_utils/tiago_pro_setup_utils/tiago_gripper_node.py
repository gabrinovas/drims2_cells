#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TiagoRightGripper(Node):
    def __init__(self, name='tiago_right_gripper_node'):
        super().__init__(name)

        self.logger = self.get_logger()

        # Action client per il gripper destro
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_right_controller/follow_joint_trajectory"
            ,
        )

    def gripper_action(self, position, duration=2.0):
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ["gripper_right_finger_joint"]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = 0

        msg.trajectory.points = [point]

        self.logger.info(f"Sending goal: {position}")
        return self.gripper_client.send_goal_async(msg)

    def close_gripper(self):
        self.logger.info("Closing gripper")
        return self.gripper_action(0.0)

    def open_gripper(self):
        self.logger.info("Opening gripper")
        return self.gripper_action(0.8)


def main():
    rclpy.init()
    node = TiagoRightGripper()
    node.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    # Spin in background
    # spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    # spin_thread.start()

    # Attendi action server
    node.logger.info("Waiting for gripper action server...")
    while not node.gripper_client.wait_for_server(timeout_sec=1.0):
        node.logger.info("Still waiting...")

    # Test: chiudi e apri gripper
    # future = node.close_gripper()
    # rclpy.spin_until_future_complete(node, future)
    # node.logger.info("Closing completed.")

    future = node.open_gripper()
    rclpy.spin_until_future_complete(node, future)

    node.logger.info("Opening completed.")

    node.destroy_node()
    rclpy.shutdown()
    # spin_thread.join()


if __name__ == "__main__":
    main()
