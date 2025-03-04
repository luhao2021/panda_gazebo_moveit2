"""
A launch file for running the motion planning python api tutorial
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda", package_name="moveit_resources_panda_gazebo_moveit_config"
        )
        .robot_description(file_path="config/panda_gazebo.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers2.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("panda_gazebo_motion_planning_python_api")
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .to_moveit_configs()
    )

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="motion_planning_python_api_tutorial.py",
        description="Python API tutorial file name",
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="panda_gazebo_motion_planning_python_api",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("panda_gazebo_motion_planning_python_api"),
        "config",
        "motion_planning_python_api_tutorial.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'panda_world.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_gazebo_moveit_config"),
        "config",
        "panda_gazebo_controllers.yaml",
    )

    # Load controllers
    controllers = [
        "joint_state_broadcaster",
        "panda_traj_controller",
        "panda_gripper_traj_controller",
    ]
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager'
            ],
            parameters=[{'use_sim_time': True}],
        ))

    return LaunchDescription(
        [
            example_file,
            moveit_py_node,
            robot_state_publisher,
            rviz_node,
            static_tf,
            gazebo_empty_world,
            spawn,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=controller_nodes,
                )
            ), 
        ]
    )
