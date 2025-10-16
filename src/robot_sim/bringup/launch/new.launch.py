import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_gazebo = get_package_share_directory('gazebo')
    pkg_project_description = get_package_share_directory('model_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add Qbert assets folder to Gazebo search path
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path
    )

    urdf_file = os.path.join(pkg_project_description, 'models', 'Qbert', 'qbert.urdf')
    robot_desc = Command(['xacro ', urdf_file])

    arguments = {'gz_args': PathJoinSubstitution([
        pkg_project_gazebo,
        'worlds',
        'qbert_world.sdf'
    ])}

    # Launch Gazebo with custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=arguments.items(),
    )

    # Publish robot state and joint states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[{'robot_description': robot_desc}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'qbert.rviz')],
    )
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base'],
        output='screen'
    )
    joint_state_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    ros2_control_path = os.path.join(pkg_project_description, 'config', 'qbert_controllers.yaml')
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[ros2_control_path]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["j2_position_controller", "--param-file", ros2_control_path],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
        output='screen'
    )

    joint_state = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_base',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base'],
        output='screen'
    )

    # Spawn robot immediately
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', 'qbert',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )


    # Bridge between ROS and Gazebo
    gazebo_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    nodeList = [
        gz_sim,
        gazebo_bridge_node,
        gazebo_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        static_tf_node,
        joint_state_node
    ]

    return LaunchDescription(nodeList)
