#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

def pv_string(val: str) -> ParameterValue:
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_STRING
    pv.string_value = val
    return pv

def pv_double(val: float) -> ParameterValue:
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_DOUBLE
    pv.double_value = val
    return pv

def pv_bool(val: bool) -> ParameterValue:
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_BOOL
    pv.bool_value = val
    return pv

def pv_string_array(vals) -> ParameterValue:
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_STRING_ARRAY
    pv.string_array_value = list(vals)
    return pv

class SetControllerParamsOnce(Node):
    """
    Loads parameters into the *controller node* in a single atomic call.
    NOTE: The controller must already be loaded (use the spawner with -t).
    """
    def __init__(self, controller_name: str = 'gripper_action_controller'):
        super().__init__('set_controller_params_once')
        self.controller_name = controller_name
        self.srv_name = f'/{controller_name}/set_parameters_atomically'
        self.cli = self.create_client(SetParametersAtomically, self.srv_name)

        # Parameters expected by GripperActionController (do NOT include "type" here)
        self.param_items = [
            ("joint",                        pv_string("gripper_right_finger_joint")),
            ("command_interfaces",           pv_string_array(["position"])),
            ("state_interfaces",             pv_string_array(["position"])),
            ("max_effort",                   pv_double(10.0)),
            ("goal_tolerance",               pv_double(0.01)),
            ("allow_stalling",               pv_bool(False)),
            ("stall_velocity_threshold",     pv_double(0.001)),
            ("stall_timeout",                pv_double(1.0)),
            ("action_monitor_rate",          pv_double(20.0)),
        ]

        # Try periodically until the controller node is up
        self.timer = self.create_timer(0.5, self.try_set)

    def try_set(self):
        if not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(f'Waiting for {self.srv_name} ...')
            return

        # Build one atomic request with ALL parameters
        req = SetParametersAtomically.Request()
        for name, value in self.param_items:
            p = Parameter()
            p.name = name            # bare parameter name (no node prefix)
            p.value = value
            req.parameters.append(p)

        self.get_logger().info(f'Calling {self.srv_name} with {len(req.parameters)} parameters...')
        future = self.cli.call_async(req)
        future.add_done_callback(self.done)
        self.timer.cancel()

    def done(self, future):
        try:
            resp = future.result()
            if resp.result.successful:
                self.get_logger().info('All parameters set successfully (atomic).')
            else:
                self.get_logger().error(f'Atomic set failed: {resp.result.reason}')
        except Exception as e:
            self.get_logger().error(f'Exception while setting parameters: {e}')
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SetControllerParamsOnce(controller_name='gripper_action_controller')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
