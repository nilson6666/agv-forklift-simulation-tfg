"""Nav2 for the AGV forklift (guided navigation with AMCL).

Run AFTER the simulation:
  ros2 launch agv_description gz_empty.launch.py headless:=true \
      world:=magatzem.sdf slam:=true
  ros2 launch agv_description nav2.launch.py

(`slam:=true` is mandatory: it removes the static map->odom, which AMCL
publishes here.)

The nodes are launched by hand instead of including nav2_bringup's
bringup_launch.py: the Jazzy bringup also starts smoother_server,
waypoint_follower, velocity_smoother, collision_monitor and
docking_server — none of them is needed here and every node costs CPU.
Minimal set: map_server + amcl (localisation) and planner + controller +
behaviors + bt_navigator (navigation), with a single lifecycle_manager.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_package = FindPackageShare('agv_description')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file_declare = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([robot_package, 'params',
                                            'nav2.yaml']),
        description='Nav2 parameters file'
    )
    params_file = LaunchConfiguration('params_file')

    map_declare = DeclareLaunchArgument(
        'map',
        # magatzem_amb_trailer.yaml: copy of magatzem.yaml with the real
        # trailer collision (gz physics box, not the visual mesh the lidar
        # sees) painted as a permanent obstacle. Needed because at its scan
        # height (z≈0.21m) the lidar only sees legs/wheels — not the solid
        # front wall the tines hit during docking — and the trailer is
        # always present in this scenario. `magatzem.yaml` (building only)
        # is kept for other uses (SLAM, mapping).
        default_value=PathJoinSubstitution([robot_package, 'maps',
                                            'magatzem_amb_trailer.yaml']),
        description='Map yaml for map_server/AMCL'
    )
    map_yaml = LaunchConfiguration('map')

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file,
                    {'yaml_filename': map_yaml,
                     'use_sim_time': use_sim_time}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        use_sim_time_declare,
        params_file_declare,
        map_declare,
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
