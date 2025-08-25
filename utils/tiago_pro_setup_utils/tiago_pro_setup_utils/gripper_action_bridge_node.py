#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TiagoGripper(Node):
    def __init__(self, name='tiago_gripper_node'):
        super().__init__(name)

        # Callback groups
        self.cb_client = MutuallyExclusiveCallbackGroup()
        self.cb_topic = MutuallyExclusiveCallbackGroup()

        self.logger = self.get_logger()
        self.joint_states = {}

        # Subscribe to joint_states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.cb_topic
        )

        # Setup action clients
        self.grippers = {
            "left": ActionClient(
                self, FollowJointTrajectory,
                "/gripper_left_controller/follow_joint_trajectory",
                callback_group=self.cb_client
            ),
            "right": ActionClient(
                self, FollowJointTrajectory,
                "/gripper_right_controller/follow_joint_trajectory",
                callback_group=self.cb_client
            )
        }

    def joint_state_callback(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            self.joint_states[name] = position

    def gripper_action(self, arm, point: JointTrajectoryPoint, vel_factor=0.1, sleep_time=0.1):
        if arm not in ['left', 'right']:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False

        action_client = self.grippers[arm]
        msg = FollowJointTrajectory.Goal()

        joint_names = [
            "gripper_right_finger_joint",
            "gripper_left_finger_joint"
        ]

        # Attendi che i joint siano disponibili
        while not all(j in self.joint_states for j in joint_names):
            self.logger.info("Waiting for joint states...")
            time.sleep(0.1)

        joint_positions = [self.joint_states[name] for name in joint_names]
        joint_goals = list(point.positions)

        self.logger.info(f"Current: {joint_positions}, Goal: {joint_goals}")

        # Calcola durata in base alla distanza
        distances = [abs(joint_positions[i] - joint_goals[i]) for i in range(2)]
        duration = max(distances) / vel_factor * 2

        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        msg.trajectory.joint_names = joint_names
        msg.trajectory.points = [point]
        self.logger.info(f"Goal: {msg}")
        future = action_client.send_goal_async(msg)

        self.logger.info(f"Goal sent, waiting ~{duration:.2f}s")
        time.sleep(duration + sleep_time)

        return future
    def move_gripper (self, arm, left_finger, right_finger, vel_factor = 0.1, sleep_time = 0.1):
        point = JointTrajectoryPoint()
        point.positions = [right_finger, left_finger]
        self.logger.info(f"Moving {arm} gripper to: {right_finger},{left_finger}")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        
    def close_gripper(self, arm, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        self.logger.info("Closing gripper")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        return future

    def open_gripper(self, arm, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        self.logger.info("Opening gripper")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        return future

def main():
    rclpy.init()
    tiago = TiagoGripper()

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(tiago,), daemon=True)
    spin_thread.start()

    # Attendi che il server action sia pronto
    tiago.logger.info("Waiting for action server...")
    while not tiago.grippers["right"].wait_for_server(timeout_sec=1.0):
        tiago.logger.info("Still waiting for gripper action server...")
        time.sleep(0.5)

    # Prepara punto
    point = JointTrajectoryPoint()
    point.positions = [20.0, 1.0]

    # Invia comando
    arm = 'right'


    future = tiago.close_gripper(arm)
    future = tiago.open_gripper(arm)

    # Aspetta il risultato
    rclpy.spin_until_future_complete(tiago, future)

    tiago.logger.info("Goal completed.")

    # Shutdown pulito
    tiago.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint


# class GripperCloseClient(Node):
#     def __init__(self):
#         super().__init__('gripper_close_client')

#         # Crea action client
#         self._action_client = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/gripper_right_controller/follow_joint_trajectory'
#         )

#         # Aspetta che l'action server sia disponibile
#         self.get_logger().info('Waiting for action server...')
#         self._action_client.wait_for_server()

#         # Invia goal di chiusura pinza
#         self.send_goal()

#     def send_goal(self):
#         goal_msg = FollowJointTrajectory.Goal()

#         # Specifica i nomi dei giunti della pinza
#         goal_msg.trajectory.joint_names = ['right_gripper_finger_joint']

#         # Definisci un punto target
#         point = JointTrajectoryPoint()
#         point.positions = [0.0]  # chiusura pinza (adatta il valore in base al tuo robot)
#         point.velocities = [0.0]
#         point.accelerations = [0.0]
#         point.effort = [0.0]
        
#         point.time_from_start.sec = 2

#         goal_msg.trajectory.points.append(point)

#         self.get_logger().info(f'Sending goal to close gripper: {goal_msg}...')
#         self._send_goal_future = self._action_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error(f'Goal rejected :(')
#             return

#         self.get_logger().info('Goal accepted :)')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'Action finished, error_code: {result.error_code}')
#         rclpy.shutdown()

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f'Received feedback: {feedback}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = GripperCloseClient()
#     rclpy.spin(node)


# if __name__ == '__main__':
#     main()
