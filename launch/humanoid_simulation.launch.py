"""
launch/humanoid_simulation.launch.py
─────────────────────────────────────
Master launch file — starts:
  1. Gazebo with custom world
  2. robot_state_publisher (URDF → TF)
  3. Fall Detection Node
  4. Anti-Disturbance Controller Node
  5. Balance Monitor Node
  6. RViz with pre-configured layout
"""

import os
from ament_python import get_package_share_directory  # type: ignore (ROS runtime only)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare("humanoid_balance_robot")

    # ── Launch Arguments ────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Launch RViz visualization"
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="true",
        description="Launch Gazebo simulation"
    )
    paused_arg = DeclareLaunchArgument(
        "paused", default_value="false",
        description="Start Gazebo paused"
    )

    use_rviz   = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")

    # ── URDF via xacro ──────────────────────────────────────────────────────
    urdf_path = PathJoinSubstitution([pkg, "urdf", "humanoid_upper_body.urdf"])
    robot_description = Command([FindExecutable(name="cat"), " ", urdf_path])

    # ── robot_state_publisher ───────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 50.0,
            "use_tf_static": True,
        }],
    )

    # ── joint_state_publisher (for RViz sliders without Gazebo) ─────────────
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition("false"),  # disabled when Gazebo is running
    )

    # ── Gazebo ──────────────────────────────────────────────────────────────
    world_file = PathJoinSubstitution([pkg, "worlds", "disturbance_test.world"])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
            "pause": LaunchConfiguration("paused"),
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "humanoid_upper_body",
            "-x", "0.0", "-y", "0.0", "-z", "0.5",
        ],
        output="screen",
    )

    # ── Fall Detection Node ─────────────────────────────────────────────────
    fall_detection_node = Node(
        package="humanoid_balance_robot",
        executable="fall_detection_node",
        name="fall_detection_node",
        output="screen",
        parameters=[PathJoinSubstitution([pkg, "config", "ros2_controllers.yaml"])],
        remappings=[("/imu/torso", "/imu/torso")],
    )

    # ── Anti-Disturbance Controller ─────────────────────────────────────────
    controller_node = Node(
        package="humanoid_balance_robot",
        executable="anti_disturbance_controller",
        name="anti_disturbance_controller",
        output="screen",
        parameters=[PathJoinSubstitution([pkg, "config", "ros2_controllers.yaml"])],
    )

    # ── Balance Monitor ─────────────────────────────────────────────────────
    monitor_node = Node(
        package="humanoid_balance_robot",
        executable="balance_monitor",
        name="balance_monitor",
        output="screen",
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([pkg, "rviz", "humanoid_balance.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # ── Delayed start for nodes (wait for Gazebo to initialize) ─────────────
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[fall_detection_node, controller_node, monitor_node],
    )

    return LaunchDescription([
        use_rviz_arg,
        use_gazebo_arg,
        paused_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_launch,
        spawn_entity,
        delayed_nodes,
        rviz_node,
    ])
