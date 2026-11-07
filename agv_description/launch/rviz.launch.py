from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_package = FindPackageShare('agv_description')
    robot_urdf_file_name = 'my_agv.urdf.xacro'
    rviz_config_file_name = 'agv_rviz.rviz'

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

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
        description='Path to the RViz config file.'
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

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_path_arg,
        use_sim_time_declare,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
