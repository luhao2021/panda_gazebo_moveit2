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
        MoveItConfigsBuilder("moveit_resources_panda_gazebo",)
        .robot_description(file_path="config/panda_gazebo.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers2.yaml")
        .to_moveit_configs()
    )
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
        #arguments=['--ros-args', '--log-level', 'debug'],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("panda_gazebo_demo") + "/launch/move_group.rviz"
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
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'use_sim_time': True}, moveit_config.robot_description],
    )

    # Gazebo Sim
    #world = PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'worlds', 'table.world'])
    world = os.path.join(
        get_package_share_directory("moveit_resources_panda_gazebo_moveit_config"),
        "config",
        "table.world",
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

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription(
        [
            bridge,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
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
