# Data Model for Module 2: The Digital Twin

This module does not have a traditional database schema. The data model is defined by the structure of the Unified Robot Description Format (URDF) files and the ROS 2 message types used for communication.

## URDF Data Model

The core data model is the `SimpleHumanoid` robot, which is extended with sensor plugins.

- **`simple_humanoid.urdf`**: Base robot model from Module 1.
- **`simple_humanoid_lidar.urdf`**: Extends the base model with a LiDAR sensor.
- **`simple_humanoid_depth.urdf`**: Extends the base model with a depth camera.
- **`simple_humanoid_full.urdf`**: Extends the base model with all sensors (LiDAR, depth camera, IMU) and the `gazebo_ros_control` plugin.

## ROS 2 Message Data Model

The following ROS 2 message types are used for sensor data and control commands:

- **`sensor_msgs/LaserScan`**: For LiDAR data from the `/scan` topic.
- **`sensor_msgs/PointCloud2`**: For depth camera data from the `/camera/depth/points` topic.
- **`sensor_msgs/Imu`**: For IMU data from the `/imu` topic.
- **`trajectory_msgs/JointTrajectory`**: For sending joint commands to the `/joint_command` topic.
- **`sensor_msgs/JointState`**: For reading joint states from the `/joint_states` topic.
