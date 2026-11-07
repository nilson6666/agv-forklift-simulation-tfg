from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ======================================================================== #
    # 1. ENVIRONMENT SETUP & VARIABLES
    # ======================================================================== #

    robot_description_path = get_package_share_directory('agv_description')
    robot_package = FindPackageShare('agv_description')
    robot_name = 'agv_mecanum'
    robot_urdf_file_name = 'my_agv.urdf.xacro'
    rviz_config_file_name = 'agv_rviz.rviz'

    parent_of_share_path = os.path.dirname(robot_description_path)

    # --- Set GZ_SIM_RESOURCE_PATH ---
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.pathsep,
            parent_of_share_path,
            os.path.pathsep,
            os.path.join(robot_description_path, 'models')
        ]
    )

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # headless:=true -> gz sim server without GUI (-s). RViz is not affected.
    # The Gazebo GUI uses ~175% CPU and starves the gpu_lidars (11Hz -> 2.3Hz).
    headless_declare = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo server-only, without the GUI'
    )
    headless = LaunchConfiguration('headless')

    # slam:=true disables the static map->odom transform: slam_toolbox
    # (and AMCL with Nav2) publish that TF themselves and would conflict
    # with it directly.
    slam_declare = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Disable the static map->odom TF (SLAM/AMCL publishes it)'
    )
    slam = LaunchConfiguration('slam')

    # world:=magatzem.sdf for SLAM/Nav2 (file name inside worlds/)
    world_declare = DeclareLaunchArgument(
        'world',
        default_value='mon_net.sdf',
        description='World SDF file name inside the worlds/ directory'
    )
    world_file = PathJoinSubstitution([robot_package, 'worlds',
                                       LaunchConfiguration('world')])

    # ======================================================================== #
    # 2. ROBOT DESCRIPTION & RVIZ
    # ======================================================================== #

    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            robot_package, 'urdf', robot_urdf_file_name
        ]),
        description='Path to the URDF/xacro file for the robot description.'
    )

    rviz_config_path_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=PathJoinSubstitution([
            robot_package, 'rviz', rviz_config_file_name
        ]),
        description='Path to the RViz configuration file.'
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb'
        }
    )

    # ======================================================================== #
    # 3. GAZEBO LAUNCH
    # ======================================================================== #

    gz_sim_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-s -r' if '", headless, "' == 'true' else '-r'"]),
                ' ',
                world_file
            ],
            'use_sim_time': use_sim_time,
            'on_exit_shutdown': 'True'
        }.items()
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ======================================================================== #
    # 4. ROS <-> GAZEBO BRIDGE
    # ======================================================================== #

    bridge_config_file = os.path.join(robot_description_path, 'params', 'agv_bridge.yaml')

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': bridge_config_file}
        ],
        output='screen'
    )

    # ======================================================================== #
    # 5. STATIC TRANSFORMS (TF)
    # ======================================================================== #

    # Front-right lidar: from the real URDF link to the internal frame that
    # Gazebo Sim writes in the LaserScan (gz-sim does not allow choosing a
    # frame_id for gpu_lidar, unlike cameras with <optical_frame_id>; it
    # always uses <model>/<canonical_link>/<sensor>, where the canonical
    # link is the model root, not the link the sensor is attached to).
    front_lidar_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_lidar_frame_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'front_right_laser_link',
                   'agv_mecanum/base_footprint_link/front_right_lidar']
    )

    # Rear-left lidar
    rear_lidar_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_lidar_frame_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1',
                   'rear_left_laser_link',
                   'agv_mecanum/base_footprint_link/rear_left_lidar']
    )

    # map → odom (only when NO SLAM/AMCL is publishing it)
    map_odom_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        condition=UnlessCondition(slam)
    )

    # ======================================================================== #
    # 6. LASER SCAN MERGER — merges /front_lidar/scan + /rear_lidar/scan -> /scan
    # ======================================================================== #

    laser_scan_merger_container = ComposableNodeContainer(
        name='laser_scan_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='laser_scan_merger',
                parameters=[
                    {'laser_1_topic': '/front_lidar/scan'},
                    {'laser_2_topic': '/rear_lidar/scan'},
                    {'merged_scan_topic': '/scan'},
                    {'target_frame': 'base_footprint_link'},
                    {'angle_min': -3.1415},
                    {'angle_max': 3.1415},
                    {'angle_increment': 0.0087},
                    {'range_min': 0.1},
                    {'range_max': 20.0},
                    # tolerance (s): synchroniser age penalty AND TF lookup
                    # timeout. The 0.01 s default is too short on slow
                    # machines and makes the transform fail ("No data to copy").
                    {'tolerance': 0.1},
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
        output='screen',
    )

    # ======================================================================== #
    # RETURN
    # ======================================================================== #

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_path_arg,
        use_sim_time_declare,
        headless_declare,
        world_declare,
        slam_declare,
        set_gz_sim_resource_path,
        front_lidar_tf_publisher_node,
        rear_lidar_tf_publisher_node,
        map_odom_publisher_node,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        ros_gz_bridge,
        laser_scan_merger_container,
        rviz2_node
    ])