from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Use camera in the system'
    )
    
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU in the system'
    )
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Use LIDAR in the system'
    )

    # Get launch configuration values
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    use_lidar = LaunchConfiguration('use_lidar')

    # Camera driver
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',  # Updated executable name
        name='camera',
        parameters=[{
            'enable_depth': True,
            'enable_color': True,
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/depth/image')
        ],
        condition=IfCondition(use_camera)  # Add condition to optionally disable
    )

    # IMU driver
    imu_node = Node(
        package='bno055',
        executable='bno055_node',  # Updated executable name - common in newer packages
        name='imu',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'imu_link'
        }],
        condition=IfCondition(use_imu)
    )

    # LIDAR driver - using more standard package name
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',  # Updated executable name
        name='lidar',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        condition=IfCondition(use_lidar)
    )

    # Depth image to point cloud converter
    depth_to_pointcloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz',  # Updated executable name
        name='depth_to_pointcloud',
        remappings=[
            ('image_rect', '/camera/depth/image'),
            ('camera_info', '/camera/depth/camera_info'),
            ('points', '/camera/depth/points')
        ]
    )

    # Robot localization - EKF fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        parameters=[os.path.join(os.getcwd(), 'src', 'config', 'ekf_config.yaml')],  # External config file
        # Alternative inline parameters:
        # parameters=[{
        #     'frequency': 30.0,
        #     'sensor_timeout': 0.1,
        #     'two_d_mode': False,
        #     'publish_tf': True,
        #     'map_frame': 'map',
        #     'odom_frame': 'odom',
        #     'base_link_frame': 'base_link',
        #     'world_frame': 'odom',
        #     'odom0': '/wheel/odometry',
        #     'odom0_config': [True, True, True,   # x, y, z
        #                     False, False, False,  # roll, pitch, yaw
        #                     False, False, False,  # vx, vy, vz
        #                     False, False, True,   # vroll, vpitch, vyaw
        #                     False, False, False], # ax, ay, az
        #     'imu0': '/imu/data',
        #     'imu0_config': [False, False, False,  # orientation x, y, z
        #                    True, True, True,      # orientation roll, pitch, yaw
        #                    False, False, False,   # angular velocity x, y, z
        #                    True, True, True,      # angular velocity roll, pitch, yaw
        #                    True, True, True],     # linear acceleration x, y, z
        # }]
    )

    # Vision processor node
    vision_processor_node = Node(
        package='your_package_name',  # Replace with actual package name
        executable='vision_processor',  # Replace with actual executable name
        name='vision_processor',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'output_topic': '/camera/image_processed'
        }]
    )

    # IMU processor node
    imu_processor_node = Node(
        package='your_package_name',  # Replace with actual package name
        executable='imu_processor',  # Replace with actual executable name
        name='imu_processor',
        parameters=[{
            'imu_topic': '/imu/data',
            'output_topic': '/imu/processed'
        }]
    )

    # Visual-inertial odometry node
    vio_node = Node(
        package='your_package_name',  # Replace with actual package name
        executable='visual_inertial_odometry',  # Replace with actual executable name
        name='visual_inertial_odometry',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'imu_topic': '/imu/data',
            'output_pose_topic': '/vio/pose'
        }]
    )

    return LaunchDescription([
        use_camera_arg,
        use_imu_arg,
        use_lidar_arg,
        camera_node,
        imu_node,
        lidar_node,
        depth_to_pointcloud_node,
        ekf_node,
        vision_processor_node,
        imu_processor_node,
        vio_node
    ])