"""Launch the Thaiquiri simulation with a shared simulation clock."""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def _shutdown_if_running(event, context):
    # Ctrl-C already initiated shutdown; do not shut the ROS adapter down twice.
    if context.is_shutdown:
        return []
    return [launch.actions.EmitEvent(event=launch.events.Shutdown())]


def generate_launch_description():
    """Create controller, sensor-processing, and optional mapping actions."""
    use_rtabmap = LaunchConfiguration('rtabmap', default=True)

    package_dir = get_package_share_directory('thaiquiri_webots')
    robot_description_path = os.path.join(
        package_dir, 'resource', 'thaiquiri.urdf')

    with open(robot_description_path, 'r') as desc_file:
        robot_description = desc_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'robot_description': robot_description}]
    # )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'drone',
            'rtabmap_args': '--delete_db_on_start --Reg/Strategy 1 --Grid/3D true',
            'database_path': LaunchConfiguration('database_path'),
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),

            'stereo': 'true',
            # The direct stereo callback does not carry scan clouds. Packaging
            # stereo as RGBDImage enables RTAB-Map's RGBD + scan synchronizer.
            'rgbd_sync': 'true',
            'subscribe_rgbd': 'true',
            'approx_rgbd_sync': 'false',
            'left_image_topic': '/drone/left/image_rect_color',
            'right_image_topic': '/drone/right/image_rect',
            'left_camera_info_topic': '/drone/left/camera_info',
            'right_camera_info_topic': '/drone/right/camera_info',

            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/drone/lidar_livox/point_cloud',

            'qos': '2',
            'qos_scan': '2',

            'visual_odometry': 'false',
            'icp_odometry': 'true',
            'odom_args': LaunchConfiguration('icp_odom_args'),
            # OdometryROS only subscribes to the IMU when this is enabled.
            'imu_topic': '/imu/data',
            'wait_imu_to_init': 'true',
            'qos_imu': '2',

            'frame_id': 'base_footprint',
            'vo_frame_id': 'odom',
            # Consume Odometry messages, including tracking-loss covariance.
            # vo_frame_id above still publishes odom -> base_footprint TF.
            'odom_frame_id': '',
            'map_frame_id': 'map',

            'approx_sync': 'True',
            'approx_sync_max_interval': '0.05',
            'rviz': 'False',

            'queue_size': '20',
        }.items(),
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([
            package_dir, 'webots_world', 'worlds', LaunchConfiguration('world')]),
        ros2_supervisor=True,
        gui=LaunchConfiguration('gui'),
    )

    drone_driver = WebotsController(
        robot_name='drone',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
        ]
    )

    fix_camera_info = Node(
        package='thaiquiri_webots',
        executable='fix_camera_info',
        name='fix_camera_info',
        output='screen',
        namespace='drone',
    )

    bgra_to_bgr_node = Node(
        package='thaiquiri_webots',
        executable='bgra_to_bgr',
        name='bgra_to_bgr_node',
        output='screen',
        namespace='drone',
    )

    imu_merger_node = Node(
        package='thaiquiri_webots',
        executable='imu_merger_node',
        name='imu_merger_node',
        output='screen',
        namespace='drone',
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[
                {'use_mag': False},  # Webots no suele tener magnetómetro por defecto
                # Si quieres que publique el frame transform
                {'publish_tf': False},
                {'gain': ParameterValue(LaunchConfiguration('imu_filter_gain'), value_type=float)},
                {'world_frame': 'enu'}  # East-North-Up, el estándar de ROS
        ],
        remappings=[
            # Se suscribe al tópico que genera nuestro merger
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )

    stereo_proc_share = get_package_share_directory('stereo_image_proc')
    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            stereo_proc_share, 'launch', 'stereo_image_proc.launch.py')),
        launch_arguments={
            'namespace': 'drone',
            'approximate_sync': 'True',
        }.items()
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=drone_driver,
        nodes_to_start=[GroupAction(actions=[rtabmap], condition=IfCondition(use_rtabmap))]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rtabmap', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='mine_thaiquiri.wbt',
                              description='World filename, e.g. cerro_rico.wbt'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument(
            'imu_filter_gain', default_value='0.01',
            description='Madgwick gain for moving flight; 0.1 restores the old tuning'),
        DeclareLaunchArgument(
            'icp_max_translation', default_value='0.2',
            description='Maximum accepted ICP correction (m); vary for diagnosis'),
        DeclareLaunchArgument(
            'icp_odom_args',
            # The 40-layer LiDAR needs more than the default 2,000-point
            # local map. Keep sampling and map subtraction at the same scale;
            # allow correspondence search beyond the default 5-10 cm.
            default_value=[
                '--Icp/PointToPlane true --Icp/PointToPlaneK 20 '
                '--Icp/PointToPlaneRadius 0 --Icp/VoxelSize 0.1 '
                '--Icp/MaxCorrespondenceDistance 0.5 '
                '--Icp/MaxTranslation ', LaunchConfiguration('icp_max_translation'),
                ' '
                '--Icp/Iterations 20 --Icp/Epsilon 0.001 '
                '--Icp/CorrespondenceRatio 0.01 '
                '--Odom/ScanKeyFrameThr 0.4 '
                '--OdomF2M/ScanSubtractRadius 0.1 '
                '--OdomF2M/ScanMaxSize 15000'],
            description='RTAB-Map core arguments applied only to ICP odometry'),
        webots,
        # The clock producer must use wall time for its stepping timer.
        webots._supervisor,
        SetParameter(name='use_sim_time', value=True),
        drone_driver,
        robot_state_publisher,
        fix_camera_info,
        bgra_to_bgr_node,
        GroupAction(actions=[stereo_proc]),
        # joint_state_publisher,
        imu_merger_node,
        imu_filter_node,
        waiting_nodes,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=_shutdown_if_running,
            )
        )
    ])
