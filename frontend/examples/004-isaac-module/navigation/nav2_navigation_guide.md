# Nav2 Navigation System Guide

## Overview
This guide provides instructions for configuring Nav2 for humanoid robot navigation in Isaac Sim. The navigation system will include path planning, obstacle avoidance, and trajectory execution capabilities optimized for humanoid kinematics.

## Nav2 Architecture for Humanoid Robots

### 1. Key Nav2 Components
- **Navigation2 Stack**: Core navigation framework
- **SLAM Toolbox**: Simultaneous localization and mapping
- **Nav2 Planner**: Global and local path planners
- **Controller**: Trajectory controllers for humanoid kinematics
- **Recovery Behaviors**: Failure recovery mechanisms
- **Lifecycle Manager**: Component lifecycle management

### 2. Humanoid-Specific Considerations
- **Kinematic Constraints**: Accounting for humanoid joint limits
- **Footstep Planning**: For bipedal robots
- **Balance Maintenance**: Keeping humanoid robot stable during navigation
- **Obstacle Avoidance**: Considering full robot body vs simple circular base

## Configuration Files

### 1. Main Navigation Configuration
```yaml
# config/navigation.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_delay: 0.5
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    interrupt_on_battery: True
    battery_topic: /battery
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_remaining_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "nav2_mppi_controller::MPPICtrl"
      time_steps: 24
      control_freq: 20.0
      horizon: 1.5
      dt: 0.05
      vx_samples: 15
      vy_samples: 5
      wz_samples: 25
      auxiliary_ux: 0.6
      auxiliary_uy: 0.0
      auxiliary_uz: 0.0
      reference_cte_gain: 0.0
      reference_heading_gain: 0.0
      reference_velocity_gain: 25.0
      xy_error_threshold: 0.2
      speed_scaling_factor: 0.25
      control_cost_gain: 0.05
      goal_cost_gain: 1.0
      obstacle_cost_gain: 5.0
      reference_cost_gain: 1.0
      max_linear_speed: 0.5
      min_linear_speed: 0.1
      max_angular_speed: 1.0
      min_angular_speed: 0.1
      max_vel_x: 0.5
      min_vel_x: 0.1
      max_vel_y: 0.0
      min_vel_y: 0.0
      max_vel_theta: 1.0
      min_vel_theta: 0.1
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
      robot_radius: 0.4  # Humanoid robot radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.4  # Humanoid robot radius
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries/Spin", "nav2_recoveries/BackUp", "nav2_recoveries/Wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
      sim_frequency: 50
      angle_instep: 0.05
      angle_tolerance: 0.05
      max_rotational_vel: 1.0
      min_rotational_vel: 0.1
    backup:
      plugin: "nav2_recoveries/BackUp"
      sim_frequency: 50
      trans_stopped_velocity: 0.05
      rotational_stopped_velocity: 0.05
      backup_vel: -0.1
    wait:
      plugin: "nav2_recoveries/Wait"
      sim_frequency: 50
      wait_duration: 5.0

robot_state_publisher:
  ros__parameters:
    use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### 2. Humanoid-Specific Navigation Configuration
```yaml
# config/humanoid_navigation.yaml
humanoid_navigation:
  # Humanoid-specific parameters
  robot_dimensions:
    width: 0.6      # Width of humanoid robot
    depth: 0.4      # Depth of humanoid robot
    height: 1.7      # Height of humanoid robot
    base_height: 0.8 # Height of base link from ground

  # Kinematic constraints for humanoid
  kinematic_constraints:
    max_linear_velocity: 0.5      # m/s
    min_linear_velocity: 0.05     # m/s
    max_angular_velocity: 0.8     # rad/s
    min_angular_velocity: 0.05    # rad/s
    max_linear_acceleration: 0.5  # m/s²
    max_angular_acceleration: 1.0 # rad/s²

  # Balance maintenance parameters
  balance_maintenance:
    enable_balance_check: true
    balance_threshold: 0.1        # Maximum CoM deviation allowed
    step_size_limit: 0.3          # Maximum step size for bipedal robots
    zmp_margin: 0.05              # Zero Moment Point safety margin

  # Footstep planning (for bipedal robots)
  footstep_planning:
    enable_footstep_planning: true
    foot_separation: 0.2          # Distance between feet when standing
    step_height: 0.05             # Height to lift foot during stepping
    step_duration: 1.0            # Time to complete each step
    max_step_width: 0.3           # Maximum lateral step width
    max_step_length: 0.4          # Maximum forward step length

  # Navigation parameters tuned for humanoid
  navigation_tuning:
    goal_tolerance: 0.3           # Distance to goal before considering reached
    xy_goal_tolerance: 0.3        # XY plane tolerance
    yaw_goal_tolerance: 0.2       # Yaw angle tolerance (rad)
    oscillation_timeout: 30.0     # Time to wait before considering oscillation
    oscillation_distance: 0.5     # Distance threshold for oscillation detection

  # Costmap inflation for humanoid safety
  costmap_inflation:
    inflation_radius: 0.6         # Radius to inflate obstacles for humanoid safety
    cost_scaling_factor: 5.0      # Factor for scaling obstacle costs

  # Recovery behaviors for humanoid
  recovery_behaviors:
    - name: "clear_local_costmap"
      type: "clear_costmap"
      params:
        reset_distance: 1.0
    - name: "humanoid_spin"
      type: "spin"
      params:
        spin_dist: 1.57           # 90 degrees in radians
        linear_x_max: 0.1
        angular_z_max: 0.5
    - name: "backup_and_turn"
      type: "backup"
      params:
        backup_dist: -0.3
        backup_speed: 0.1
    - name: "wait_recovery"
      type: "wait"
      params:
        wait_duration: 5.0
```

## Launch Files

### 1. Navigation Stack Launch
```xml
<!-- launch/navigation_launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    map_topic = LaunchConfiguration('map_topic')

    # Parameters
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('isaac_navigation'),
                'config',
                'navigation.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Topic to subscribe to for the map'),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': os.path.join(
                            get_package_share_directory('isaac_navigation'),
                            'maps',
                            'simple_room.yaml')}]),

        # Local costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'local_cmd_vel')]),

        # Global costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'global_cmd_vel')]),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # Planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # Controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]),

        # BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # Recovery server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        # Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}])
    ])
```

### 2. Isaac Sim Navigation Integration
```python
# navigation/isaac_sim_nav_integration.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class IsaacSimNavigationIntegration(Node):
    def __init__(self):
        super().__init__('isaac_sim_navigation_integration')

        # Publishers for Isaac Sim
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers from Isaac Sim sensors
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Navigation goal publisher
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state tracking
        self.current_pose = None
        self.current_twist = None

        self.get_logger().info('Isaac Sim Navigation Integration initialized')

    def odom_callback(self, msg):
        """Handle odometry messages from Isaac Sim"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def scan_callback(self, msg):
        """Handle laser scan messages from Isaac Sim"""
        # In a real implementation, this would provide scan data to Nav2
        # For now, we just log the data
        range_count = len(msg.ranges)
        valid_ranges = sum(1 for r in msg.ranges if r > msg.range_min and r < msg.range_max)

        self.get_logger().debug(f'Laser scan: {range_count} ranges, {valid_ranges} valid')

    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to Isaac Sim robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = float(linear_x)
        cmd_msg.angular.z = float(angular_z)

        self.cmd_vel_pub.publish(cmd_msg)

    def send_navigation_goal(self, x, y, theta):
        """Send navigation goal to Nav2"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        # Convert theta to quaternion
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.orientation.x = quat[0]
        goal_msg.pose.orientation.y = quat[1]
        goal_msg.pose.orientation.z = quat[2]
        goal_msg.pose.orientation.w = quat[3]

        self.nav_goal_pub.publish(goal_msg)
        self.get_logger().info(f'Sent navigation goal: ({x}, {y}, {theta})')

def main(args=None):
    rclpy.init(args=args)
    nav_integration = IsaacSimNavigationIntegration()

    # Example: Send a navigation goal after 5 seconds
    timer = nav_integration.create_timer(5.0, lambda: nav_integration.send_navigation_goal(2.0, 2.0, 0.0))

    try:
        rclpy.spin(nav_integration)
    except KeyboardInterrupt:
        pass
    finally:
        nav_integration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Navigation Behavior Trees

### 1. Custom Behavior Tree for Humanoid Navigation
```xml
<!-- bt_xml/humanoid_navigate_to_pose_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
      </RateController>
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">
        <PipelineSequence name="Navigate">
          <FollowPath path="{path}" controller_id="FollowPath"/>
        </PipelineSequence>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
          <Spin spin_dist="1.57" time_allowance="10"/>
          <BackUp backup_dist="0.30" backup_speed="0.1" time_allowance="10"/>
          <Wait wait_duration="5"/>
        </ReactiveFallback>
      </RecoveryNode>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

### 2. Humanoid-Specific Recovery Behaviors
```python
# navigation/recovery_behaviors.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
import math

class HumanoidRecoveryBehaviors(Node):
    def __init__(self):
        super().__init__('humanoid_recovery_behaviors')

        # Publisher for velocity commands during recovery
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Humanoid Recovery Behaviors initialized')

    def balance_recovery(self):
        """Perform balance recovery for humanoid robot"""
        # This would implement specific balance recovery for humanoid robots
        # using center of mass control, step adjustment, etc.
        self.get_logger().info('Performing balance recovery...')

        # Send commands to restore balance
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def footstep_adjustment_recovery(self):
        """Adjust footsteps for humanoid robot"""
        # This would implement footstep adjustment recovery
        # for bipedal humanoid robots
        self.get_logger().info('Adjusting footsteps for recovery...')

    def step_back_recovery(self):
        """Perform a step back to recover from navigation failure"""
        self.get_logger().info('Performing step back recovery...')

        # Send command to step back carefully
        cmd = Twist()
        cmd.linear.x = -0.1  # Move backward slowly
        cmd.angular.z = 0.0

        # Execute for a short duration
        self.cmd_vel_pub.publish(cmd)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.0))

        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    recovery_node = HumanoidRecoveryBehaviors()

    try:
        rclpy.spin(recovery_node)
    except KeyboardInterrupt:
        pass
    finally:
        recovery_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Path Planning Algorithms

### 1. Humanoid-Specific Path Planner
```python
# navigation/humanoid_path_planner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial import KDTree
import math

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_path', 10)
        self.path_viz_pub = self.create_publisher(Marker, '/path_visualization', 10)

        # Parameters for humanoid-specific planning
        self.robot_width = 0.6  # meters
        self.robot_depth = 0.4  # meters
        self.max_step_width = 0.3  # maximum lateral step for humanoid
        self.max_step_length = 0.4  # maximum forward step for humanoid

        self.get_logger().info('Humanoid Path Planner initialized')

    def plan_path(self, start_pose, goal_pose, costmap):
        """
        Plan path considering humanoid kinematic constraints
        """
        # In a real implementation, this would use a path planning algorithm
        # like A*, D* Lite, or RRT* with humanoid-specific constraints

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Generate a simple path (in real implementation, use proper algorithm)
        path_points = self.generate_humanoid_feasible_path(start_pose, goal_pose)

        for point in path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0

            # Set orientation to face along the path
            if len(path_points) > 1:
                idx = path_points.index(point)
                if idx < len(path_points) - 1:
                    next_point = path_points[idx + 1]
                    angle = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
                    # Convert angle to quaternion (simplified)
                    pose_stamped.pose.orientation.z = math.sin(angle / 2)
                    pose_stamped.pose.orientation.w = math.cos(angle / 2)

            path_msg.poses.append(pose_stamped)

        return path_msg

    def generate_humanoid_feasible_path(self, start_pose, goal_pose):
        """
        Generate a path that respects humanoid kinematic constraints
        """
        # This is a simplified example
        # In real implementation, this would use proper path planning

        start = (start_pose.position.x, start_pose.position.y)
        goal = (goal_pose.position.x, goal_pose.position.y)

        # Calculate straight-line path with intermediate waypoints
        # respecting maximum step sizes
        path = [start]

        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Break path into steps that respect humanoid constraints
        step_size = min(self.max_step_length, self.max_step_width)
        num_steps = max(1, int(distance / step_size))

        for i in range(1, num_steps + 1):
            fraction = i / num_steps
            x = start[0] + dx * fraction
            y = start[1] + dy * fraction
            path.append((x, y))

        path.append(goal)

        return path

    def validate_path_for_humanoid(self, path):
        """
        Validate that the path is feasible for humanoid robot
        """
        # Check that path respects kinematic constraints
        # Check for obstacles considering humanoid dimensions
        # Check for balance constraints

        is_valid = True
        reasons = []

        # Check step sizes
        for i in range(len(path) - 1):
            step_dx = path[i+1][0] - path[i][0]
            step_dy = path[i+1][1] - path[i][1]
            step_distance = math.sqrt(step_dx*step_dx + step_dy*step_dy)

            if step_distance > self.max_step_length:
                is_valid = False
                reasons.append(f"Step {i} too long: {step_distance:.2f}m > {self.max_step_length}m")

        return is_valid, reasons

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidPathPlanner()

    # Example usage
    timer = planner.create_timer(1.0, lambda: example_path_planning(planner))

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

def example_path_planning(planner):
    """Example of path planning"""
    from geometry_msgs.msg import Pose

    start = Pose()
    start.position.x = 0.0
    start.position.y = 0.0

    goal = Pose()
    goal.position.x = 5.0
    goal.position.y = 5.0

    path_msg = planner.plan_path(start, goal, None)

    # Validate path
    path_coords = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
    is_valid, reasons = planner.validate_path_for_humanoid(path_coords)

    if is_valid:
        planner.get_logger().info(f'Valid path with {len(path_msg.poses)} waypoints generated')
        planner.path_pub.publish(path_msg)
    else:
        planner.get_logger().warn(f'Path validation failed: {reasons}')

if __name__ == '__main__':
    main()
```

## Navigation Performance Monitoring

### 1. Navigation Performance Metrics
```python
# navigation/performance_monitor.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, Bool
from builtin_interfaces.msg import Time
import time
import math
from collections import deque

class NavigationPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('navigation_performance_monitor')

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)

        # Publishers
        self.success_pub = self.create_publisher(Bool, '/navigation_success', 10)
        self.duration_pub = self.create_publisher(Float32, '/navigation_duration', 10)
        self.distance_pub = self.create_publisher(Float32, '/navigation_distance', 10)
        self.efficiency_pub = self.create_publisher(Float32, '/navigation_efficiency', 10)

        # Navigation tracking
        self.current_goal = None
        self.start_time = None
        self.start_position = None
        self.current_position = None
        self.path_length = 0.0
        self.executed_path = deque(maxlen=1000)

        self.get_logger().info('Navigation Performance Monitor initialized')

    def goal_callback(self, msg):
        """Track navigation goal"""
        self.current_goal = msg
        self.start_time = self.get_clock().now()
        self.start_position = (
            self.current_position.position.x if self.current_position else msg.pose.position.x,
            self.current_position.position.y if self.current_position else msg.pose.position.y
        )
        self.path_length = 0.0
        self.executed_path.clear()

        self.get_logger().info(f'New navigation goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def odom_callback(self, msg):
        """Track robot position"""
        self.current_position = msg.pose.pose

        # Add to executed path for efficiency calculation
        if self.start_position:
            current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.executed_path.append(current_pos)

            # Calculate actual distance traveled
            if len(self.executed_path) > 1:
                prev_pos = self.executed_path[-2]
                step_distance = math.sqrt(
                    (current_pos[0] - prev_pos[0])**2 +
                    (current_pos[1] - prev_pos[1])**2
                )
                self.path_length += step_distance

        # Check if goal is reached
        if self.current_goal:
            goal = self.current_goal.pose.position
            distance_to_goal = math.sqrt(
                (msg.pose.pose.position.x - goal.x)**2 +
                (msg.pose.pose.position.y - goal.y)**2
            )

            if distance_to_goal < 0.5:  # Goal tolerance
                self.navigation_completed()

    def path_callback(self, msg):
        """Track planned path length"""
        # Calculate planned path length for efficiency
        planned_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position
            segment_length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            planned_length += segment_length

        # Compare with actual path for efficiency
        if self.path_length > 0 and planned_length > 0:
            efficiency = planned_length / self.path_length  # Lower is better
            eff_msg = Float32()
            eff_msg.data = efficiency
            self.efficiency_pub.publish(ef_msg)

    def navigation_completed(self):
        """Handle navigation completion"""
        if self.start_time:
            duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            # Publish results
            success_msg = Bool()
            success_msg.data = True
            self.success_pub.publish(success_msg)

            duration_msg = Float32()
            duration_msg.data = duration
            self.duration_pub.publish(duration_msg)

            distance_msg = Float32()
            distance_msg.data = self.path_length
            self.distance_pub.publish(distance_msg)

            self.get_logger().info(f'Navigation completed in {duration:.2f}s, distance: {self.path_length:.2f}m')

            # Reset for next navigation
            self.current_goal = None
            self.start_time = None
            self.start_position = None

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands"""
        # Could be used to detect oscillations or stuck conditions
        pass

def main(args=None):
    rclpy.init(args=args)
    monitor = NavigationPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### 1. Navigation System Test
```python
# test/navigation_system_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import math

class NavigationSystemTest(Node):
    def __init__(self):
        super().__init__('navigation_system_test')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_pub = self.create_publisher(String, '/cancel_goal', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Test parameters
        self.test_goals = [
            (2.0, 2.0, 0.0),   # x, y, theta
            (4.0, 1.0, 1.57),  # x, y, theta
            (-1.0, 3.0, 3.14), # x, y, theta
            (0.0, 0.0, 0.0)    # Return to start
        ]

        self.current_goal_index = 0
        self.current_position = None
        self.navigation_active = False

        # Timer for test sequence
        self.test_timer = self.create_timer(10.0, self.run_next_test)

        self.get_logger().info('Navigation System Test initialized')

    def odom_callback(self, msg):
        """Track robot position"""
        self.current_position = msg.pose.pose

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.navigation_active = True

    def run_next_test(self):
        """Run next navigation test"""
        if self.current_goal_index < len(self.test_goals):
            goal = self.test_goals[self.current_goal_index]

            # Send navigation goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'

            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.position.z = 0.0

            # Convert theta to quaternion
            theta = goal[2]
            from math import sin, cos
            goal_msg.pose.orientation.z = sin(theta / 2.0)
            goal_msg.pose.orientation.w = cos(theta / 2.0)

            self.goal_pub.publish(goal_msg)

            self.get_logger().info(f'Sent test goal {self.current_goal_index + 1}: ({goal[0]}, {goal[1]}, {theta:.2f})')

            self.current_goal_index += 1
        else:
            # All tests completed
            self.test_timer.cancel()
            self.get_logger().info('All navigation tests completed')

def main(args=None):
    rclpy.init(args=args)
    test_node = NavigationSystemTest()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Common Issues

#### 1. Navigation Failures
- **Path Planning Failures**: Check costmap inflation, ensure proper map coverage
- **Local Minima**: Increase costmap resolution, adjust inflation parameters
- **Oscillation**: Tune controller parameters, adjust recovery behaviors

#### 2. Humanoid-Specific Issues
- **Balance Problems**: Adjust kinematic constraints, implement balance controller
- **Step Size Limitations**: Modify footstep planner parameters
- **Kinematic Constraints**: Update robot model and controller parameters

#### 3. Performance Issues
- **Slow Navigation**: Optimize path planning, reduce controller frequency
- **High CPU Usage**: Check costmap resolution, optimize algorithms
- **Timing Issues**: Verify simulation time synchronization

## Next Steps
After configuring the navigation system:
1. Test navigation in various Isaac Sim environments
2. Validate path planning and obstacle avoidance
3. Implement sim-to-real transfer techniques (Phase 3)
4. Integrate perception and navigation systems

## Resources
- [Nav2 Documentation](https://navigation.ros.org/)
- [Isaac ROS Navigation](https://nvidia-isaac-ros.github.io/packages/isaac_ros_navigation/index.html)
- [ROS 2 Navigation Tutorials](https://docs.ros.org/en/humble/Tutorials/Navigation-Basics/index.html)
- [Humanoid Robot Navigation](https://arxiv.org/abs/2004.05476)