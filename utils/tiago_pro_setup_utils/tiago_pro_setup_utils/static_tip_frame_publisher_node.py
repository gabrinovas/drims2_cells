#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tip_frame_publisher_node')

        # Declare the parameter 'tip_z' with a default value of 0.15 meters
        self.declare_parameter('tip_z', 0.15)
        tip_z = self.get_parameter('tip_z').get_parameter_value().double_value

        # Create the static transform broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # Create and populate the static transform message
        static_transform = TransformStamped()

        # Set header: timestamp and frame IDs
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'gripper_right_base_link'
        static_transform.child_frame_id = 'tip'

        # Set translation: z from parameter, x and y are 0
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = tip_z

        # Set rotation: identity quaternion (no rotation)
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        # Broadcast the static transform
        self.broadcaster.sendTransform(static_transform)
        self.get_logger().info(f'Static transform from gripper_right_base_link to tip published with z={tip_z:.3f}.')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
