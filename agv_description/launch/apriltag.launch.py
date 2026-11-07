"""AprilTag detectors on the robot's 2 cameras: fork (fine docking) and
mast (coarse height estimate before fine docking).

Run with the sim already up (gz_empty.launch.py):
  ros2 launch agv_description apriltag.launch.py

gz-sim cameras are ideal pinhole cameras without distortion, so
image_raw can be used directly as image_rect (no image_proc needed).

- Fork: detections on /apriltag/detections, TF
  fork_camera_optical_link -> tag36h11:<id>.
- Mast: detections on /apriltag/detections_mast, TF
  mast_camera_optical_link -> mast_tag:<id> (frame RENAMED in
  params/apriltag_mast.yaml — publishing the same tag36h11:<id> as the
  fork would make both compete for one TF frame with different parents,
  and it would jump unpredictably).
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
                                            'apriltag.yaml']),
        description='apriltag_ros parameters file (fork camera)'
    )
    params_file = LaunchConfiguration('params_file')

    mast_params_file_declare = DeclareLaunchArgument(
        'mast_params_file',
        default_value=PathJoinSubstitution([robot_package, 'params',
                                            'apriltag_mast.yaml']),
        description='apriltag_ros parameters file (mast camera)'
    )
    mast_params_file = LaunchConfiguration('mast_params_file')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_fork',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/camera_fork/image_raw'),
            ('camera_info', '/camera_fork/camera_info'),
            ('detections', '/apriltag/detections'),
        ],
    )

    apriltag_mast_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_mast',
        output='screen',
        parameters=[mast_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/camera_top/image_raw'),
            ('camera_info', '/camera_top/camera_info'),
            ('detections', '/apriltag/detections_mast'),
        ],
    )

    return LaunchDescription([
        use_sim_time_declare,
        params_file_declare,
        mast_params_file_declare,
        apriltag_node,
        apriltag_mast_node,
    ])
