#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

class MergedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('merged_joint_state_publisher')
        
        self.arm_joint_state = None
        self.gripper_joint_state = None
        self.lock = threading.Lock()
        
        # Subscribe to both sources
        self.arm_sub = self.create_subscription(
            JointState,
            'arm_joint_states',  # You'd need to remap joint_state_broadcaster output
            self.arm_callback,
            10
        )
        
        self.gripper_sub = self.create_subscription(
            JointState,
            'gripper_joint_states',  # Driver publishes here instead
            self.gripper_callback, 
            10
        )
        
        # Publisher for merged states
        self.merged_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.get_logger().info("Merged joint state publisher started")
    
    def arm_callback(self, msg):
        with self.lock:
            self.arm_joint_state = msg
            self.publish_merged()
    
    def gripper_callback(self, msg):
        with self.lock:
            self.gripper_joint_state = msg
            self.publish_merged()
    
    def publish_merged(self):
        if self.arm_joint_state and self.gripper_joint_state:
            merged = JointState()
            merged.header.stamp = self.get_clock().now().to_msg()
            
            # Combine arm and gripper joints
            merged.name = (self.arm_joint_state.name + 
                         self.gripper_joint_state.name)
            merged.position = (self.arm_joint_state.position + 
                             self.gripper_joint_state.position)
            merged.velocity = (self.arm_joint_state.velocity + 
                             self.gripper_joint_state.velocity)
            merged.effort = (self.arm_joint_state.effort + 
                           self.gripper_joint_state.effort)
            
            self.merged_pub.publish(merged)

def main():
    rclpy.init()
    node = MergedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()