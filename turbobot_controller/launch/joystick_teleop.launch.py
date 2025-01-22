import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    
    turbobot_controller_pkg = get_package_share_directory('turbobot_controller')
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True")


    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("turbobot_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("turbobot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )


    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "turbobot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(turbobot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(turbobot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(turbobot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="turbobot_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # cmd_vel_interpolator_node = Node(
    #     package="turbobot_controller",
    #     executable="cmd_vel_interpolator",
    #     name="cmd_vel_interpolator",
    #     parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    # )

    return LaunchDescription(
        [   
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
            # cmd_vel_interpolator_node
        ]
    )
