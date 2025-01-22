import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("turbobot_description"),
                    "urdf",
                    "turbobot.urdf",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time":False}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
            "robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("turbobot_controller"),
                "config",
                "turbobot_controllers.yaml",
            ),
        ],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
            # delayed_controller_manager
        ]
    )