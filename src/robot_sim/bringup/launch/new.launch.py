import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FileContent

from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_gazebo = get_package_share_directory('gazebo')
    pkg_project_description = get_package_share_directory('model_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Add model path so Gazebo can find base_plate ---
    model_path = os.path.join(pkg_project_description, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path
    )

    # Load the SDF or URDF file from "description" package
    # sdf_file = os.path.join(pkg_project_description, 'models', 'Qbert', 'qbert.sdf')
    # with open(sdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    urdf_file = os.path.join(pkg_project_bringup, 'config', 'qbert.urdf')
    urdf = FileContent(urdf_file)

    # Launch Gazebo with your custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'qbert_world.sdf'
        ])}.items(),
    )

    # Publish robot state
    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': Command(['xacro ', urdf_file])}  # Process URDF with xacro
        ]
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

    # RViz (disabled by default)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_project_description, 'rviz', 'qbert.rviz')],
        parameters=[{
            "use_sim_time": True,
        }],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Bridge between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_gazebo, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument(
            'rviz',
            default_value='false',  # disabled by default
            description='Open RViz.'
        ),
        bridge,
        robot_state,
        joint_state,
        static_tf,
        rviz,
    ])
