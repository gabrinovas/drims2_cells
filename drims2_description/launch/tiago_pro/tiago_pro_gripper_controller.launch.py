from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    ctrl_new = 'gripper_action_controller'
    ctrl_old = 'gripper_right_controller'   # controller that we want to deactivate
    cm = '/controller_manager'

    # 1) Load-only of the new controller (specify plugin type)
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[ctrl_new, '--controller-manager', cm, '--load-only'],
    )

    # 2) Run your ROS node that sets ALL parameters atomically for /gripper_action_controller
    #    (Assumes you installed it as a console_script: drims2_description set_controller_params_once)
    param_setter = Node(
        package='tiago_pro_setup_utils',
        executable='set_controller_params_once_node',
        output='screen',
        # If your script accepts CLI args, add them here, e.g.:
        # arguments=['--controller-name', ctrl_new],
    )

    # 3) Set the new controller to "inactive" (configured but not active yet)
    set_inactive = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', ctrl_new, 'inactive',
             '--controller-manager', cm],
        output='screen'
    )

    # 4) Switch controllers: deactivate the old one and activate the new one
    do_switch = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', ctrl_old,
            '--activate',   ctrl_new,
            '--strict',
            '--controller-manager', cm
        ],
        output='screen'
    )

    # Chain the steps: spawner -> param_setter node -> set_inactive -> switch
    chain = []
    chain.append(RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=[param_setter])))
    chain.append(RegisterEventHandler(OnProcessExit(target_action=param_setter, on_exit=[set_inactive])))
    chain.append(RegisterEventHandler(OnProcessExit(target_action=set_inactive, on_exit=[do_switch])))

    return LaunchDescription([spawner] + chain)
