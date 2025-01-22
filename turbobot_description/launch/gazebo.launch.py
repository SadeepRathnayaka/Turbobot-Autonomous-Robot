import os
from pathlib import Path
from os import pathsep
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    turbobot_description = get_package_share_directory("turbobot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")


    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        turbobot_description, "urdf", "turbobot.urdf"
                                        ),
                                      description="Absolute path to robot urdf file"
    )


    model_path  =  str(Path(turbobot_description).parent.resolve())
    model_path += pathsep + os.path.join(turbobot_description, "models")
    env_var     = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path     = PathJoinSubstitution([
                     turbobot_description,
                     "worlds",
                     PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])


    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )


    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")),
        launch_arguments={
            "world": world_path
        }.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "turbobot",
                                   "-topic", "robot_description",
                                  ],
                        output="screen"
    )


    # controller = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("turbobot_controller"),
    #         "launch",
    #         "controller.launch.py"
    #     ),
    # )


    # joystick_controller = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("turbobot_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    # )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(turbobot_description, "rviz", "display.rviz")],
    )


    return LaunchDescription([
        env_var,
        world_name_arg,
        model_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        # controller,
        rviz_node,
        # joystick_controller
    ])