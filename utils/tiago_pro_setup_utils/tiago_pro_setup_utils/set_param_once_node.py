#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue

class SetParamOnce(Node):
    def __init__(self):
        super().__init__('set_move_group_param_once')
        self.cli = self.create_client(SetParametersAtomically, '/move_group/set_parameters_atomically')
        self.param_name = 'ompl.arm_right_torso.longest_valid_segment_fraction'
        self.param_value = 0.001

        self.timer = self.create_timer(0.5, self.try_set)

    def try_set(self):
        if not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Wait for /move_group/set_parameters_atomically...')
            return

        # build request
        req = SetParametersAtomically.Request()
        pv = ParameterValue()
        pv.type = 3                  # DOUBLE
        pv.double_value = float(self.param_value)

        p = Parameter()
        p.name = self.param_name
        p.value = pv

        req.parameters = [p]

        future = self.cli.call_async(req)
        future.add_done_callback(self.done)
        self.timer.cancel()

    def done(self, future):
        try:
            resp = future.result()
            if resp.result.successful:
                self.get_logger().info('Parameter set successfully.')
            else:
                self.get_logger().error(f'Failed: {resp.result.reason}')
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SetParamOnce()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
