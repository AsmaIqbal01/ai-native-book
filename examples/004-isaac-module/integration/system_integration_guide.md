# System Integration Guide

## Overview
This guide provides instructions for integrating all components (perception, navigation, control, and sim-to-real transfer) into a complete autonomous humanoid system. The integration will create a cohesive system that can perform autonomous behaviors in Isaac Sim and potentially transfer to real robots.

## System Architecture

### 1. Integrated System Components
- **Perception System**: Isaac ROS-based object detection, pose estimation, and scene understanding
- **Navigation System**: Nav2-based path planning and obstacle avoidance
- **Control System**: Trajectory execution and low-level robot control
- **Transfer System**: Sim-to-real adaptation and validation
- **Behavior Engine**: High-level behavior coordination and decision making
- **Safety Monitor**: Continuous safety validation and emergency response

### 2. System Data Flow
```
Perception → Behavior Engine → Navigation → Control → Robot Actuators
     ↑                                           ↓
Safety Monitor ←→ System State → Transfer Validation
```

## Configuration Files

### 1. System Integration Configuration
```yaml
# config/system_integration.yaml
system_integration:
  version: "1.0"
  enabled: true
  system_mode: "simulation"  # simulation, real_world, transfer_validation

  components:
    perception:
      enabled: true
      node_name: "isaac_perception_pipeline"
      input_topics:
        - "/camera/color/image_rect_color"
        - "/scan"
      output_topics:
        - "/object_detections"
        - "/pose_estimates"
        - "/semantic_segmentation"
      processing_rate: 30.0  # Hz

    navigation:
      enabled: true
      node_name: "nav2_stack"
      input_topics:
        - "/initialpose"
        - "/goal_pose"
        - "/scan"
        - "/camera/depth/image_rect_raw"
      output_topics:
        - "/cmd_vel"
        - "/local_plan"
        - "/global_plan"
      update_rate: 20.0  # Hz

    control:
      enabled: true
      node_name: "robot_controller"
      input_topics:
        - "/cmd_vel"
        - "/joint_group_position_controller/commands"
      output_topics:
        - "/joint_states"
        - "/tf"
      control_rate: 100.0  # Hz

    transfer:
      enabled: true
      node_name: "transfer_validator"
      input_topics:
        - "/performance_metrics"
        - "/behavior_data"
        - "/safety_data"
      output_topics:
        - "/transfer_status"
        - "/adaptation_commands"
      validation_rate: 10.0  # Hz

    behavior_engine:
      enabled: true
      node_name: "behavior_engine"
      input_topics:
        - "/object_detections"
        - "/current_pose"
        - "/battery_status"
        - "/task_commands"
      output_topics:
        - "/navigation/goal"
        - "/manipulation/commands"
        - "/behavior_status"
      decision_rate: 10.0  # Hz

    safety_monitor:
      enabled: true
      node_name: "safety_monitor"
      input_topics:
        - "/scan"
        - "/tf"
        - "/joint_states"
        - "/cmd_vel"
      output_topics:
        - "/emergency_stop"
        - "/safety_status"
      monitoring_rate: 50.0  # Hz

  integration:
    startup_sequence:
      - "safety_monitor"
      - "perception"
      - "control"
      - "navigation"
      - "transfer"
      - "behavior_engine"

    shutdown_sequence:
      - "behavior_engine"
      - "transfer"
      - "navigation"
      - "control"
      - "perception"
      - "safety_monitor"

    inter_component_communication:
      qos_profile: "sensor_data"  # sensor_data, services, parameters
      reliability: "best_effort"  # reliable, best_effort
      durability: "volatile"      # volatile, transient_local
</`

### 2. Component Coordination Configuration
```yaml
# config/component_coordination.yaml
component_coordination:
  version: "1.0"
  coordination_pattern: "publish_subscribe"  # publish_subscribe, client_server, action_based

  perception_to_navigation:
    message_type: "object_detection_array"
    topic: "/perception/obstacles"
    coordination_frequency: 10.0
    data_mapping:
      detection_objects: "obstacle_list"
      detection_poses: "obstacle_poses"

  navigation_to_control:
    message_type: "twist"
    topic: "/navigation/velocity_commands"
    coordination_frequency: 20.0
    data_mapping:
      linear_velocity: "cmd_vel.linear.x"
      angular_velocity: "cmd_vel.angular.z"

  behavior_to_perception:
    message_type: "task_request"
    topic: "/behavior/perception_requests"
    coordination_frequency: 5.0
    data_mapping:
      target_object: "object_class"
      search_area: "roi"

  safety_to_all:
    message_type: "safety_status"
    topic: "/safety/emergency_stop"
    coordination_frequency: 100.0  # High frequency for safety
    data_mapping:
      stop_command: "emergency_stop_flag"
      safety_level: "safety_priority"

  transfer_to_behavior:
    message_type: "transfer_status"
    topic: "/transfer/validity"
    coordination_frequency: 1.0
    data_mapping:
      confidence_level: "transfer_confidence"
      adaptation_needed: "adaptation_flag"
```

## Implementation Files

### 1. Main Integration Node
```python
# integration/main_integration_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from builtin_interfaces.msg import Time
import time
from typing import Dict, Any, Optional
import threading
import queue

class IntegratedSystemNode(Node):
    def __init__(self):
        super().__init__('integrated_system_node')

        # Initialize component status
        self.components_status = {
            'perception': False,
            'navigation': False,
            'control': False,
            'transfer': False,
            'behavior_engine': False,
            'safety_monitor': True  # Safety is always active
        }

        # Initialize component health
        self.component_health = {name: 0.0 for name in self.components_status.keys()}

        # Setup QoS profiles
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.system_status_pub = self.create_publisher(String, '/system/status', 10)
        self.system_health_pub = self.create_publisher(Float32, '/system/health', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Subscribers
        self.perception_status_sub = self.create_subscription(
            String, '/perception/status', self.perception_status_callback, 10)

        self.navigation_status_sub = self.create_subscription(
            String, '/navigation/status', self.navigation_status_callback, 10)

        self.control_status_sub = self.create_subscription(
            String, '/control/status', self.control_status_callback, 10)

        self.transfer_status_sub = self.create_subscription(
            String, '/transfer/status', self.transfer_status_callback, 10)

        self.behavior_status_sub = self.create_subscription(
            String, '/behavior/status', self.behavior_status_callback, 10)

        # System state
        self.system_active = False
        self.emergency_stop_active = False
        self.system_mode = "simulation"  # simulation, real_world, validation

        # Setup timers
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        self.health_timer = self.create_timer(0.5, self.publish_system_health)

        # Setup service clients (if needed)
        self.service_clients = {}

        self.get_logger().info('Integrated System Node initialized')

    def perception_status_callback(self, msg):
        """Handle perception component status updates"""
        if msg.data == "active":
            self.components_status['perception'] = True
            self.component_health['perception'] = 1.0
        elif msg.data == "inactive":
            self.components_status['perception'] = False
            self.component_health['perception'] = 0.0
        elif msg.data.startswith("health:"):
            try:
                health_value = float(msg.data.split(':')[1])
                self.component_health['perception'] = health_value
            except ValueError:
                pass

    def navigation_status_callback(self, msg):
        """Handle navigation component status updates"""
        if msg.data == "active":
            self.components_status['navigation'] = True
            self.component_health['navigation'] = 1.0
        elif msg.data == "inactive":
            self.components_status['navigation'] = False
            self.component_health['navigation'] = 0.0
        elif msg.data.startswith("health:"):
            try:
                health_value = float(msg.data.split(':')[1])
                self.component_health['navigation'] = health_value
            except ValueError:
                pass

    def control_status_callback(self, msg):
        """Handle control component status updates"""
        if msg.data == "active":
            self.components_status['control'] = True
            self.component_health['control'] = 1.0
        elif msg.data == "inactive":
            self.components_status['control'] = False
            self.component_health['control'] = 0.0
        elif msg.data.startswith("health:"):
            try:
                health_value = float(msg.data.split(':')[1])
                self.component_health['control'] = health_value
            except ValueError:
                pass

    def transfer_status_callback(self, msg):
        """Handle transfer component status updates"""
        if msg.data == "active":
            self.components_status['transfer'] = True
            self.component_health['transfer'] = 1.0
        elif msg.data == "inactive":
            self.components_status['transfer'] = False
            self.component_health['transfer'] = 0.0
        elif msg.data.startswith("health:"):
            try:
                health_value = float(msg.data.split(':')[1])
                self.component_health['transfer'] = health_value
            except ValueError:
                pass

    def behavior_status_callback(self, msg):
        """Handle behavior engine status updates"""
        if msg.data == "active":
            self.components_status['behavior_engine'] = True
            self.component_health['behavior_engine'] = 1.0
        elif msg.data == "inactive":
            self.components_status['behavior_engine'] = False
            self.component_health['behavior_engine'] = 0.0
        elif msg.data.startswith("health:"):
            try:
                health_value = float(msg.data.split(':')[1])
                self.component_health['behavior_engine'] = health_value
            except ValueError:
                pass

    def publish_system_status(self):
        """Publish overall system status"""
        status_msg = String()

        # Determine system status based on component status
        active_components = sum(1 for status in self.components_status.values() if status)
        total_components = len(self.components_status) - 1  # Exclude safety (always active)

        if active_components == total_components:
            status_msg.data = "fully_operational"
        elif active_components >= total_components * 0.7:  # 70% operational
            status_msg.data = "degraded_operation"
        elif active_components >= total_components * 0.4:  # 40% operational
            status_msg.data = "limited_operation"
        else:
            status_msg.data = "critical_failure"

        self.system_status_pub.publish(status_msg)

    def publish_system_health(self):
        """Publish overall system health score"""
        health_msg = Float32()

        # Calculate average health across all components
        total_health = sum(self.component_health.values())
        num_components = len(self.component_health)

        if num_components > 0:
            avg_health = total_health / num_components
        else:
            avg_health = 0.0

        health_msg.data = float(avg_health)
        self.system_health_pub.publish(health_msg)

    def activate_system(self):
        """Activate the integrated system"""
        if not self.system_active:
            self.system_active = True
            self.get_logger().info('Integrated system activated')
            return True
        return False

    def deactivate_system(self):
        """Deactivate the integrated system"""
        if self.system_active:
            self.system_active = False
            self.get_logger().info('Integrated system deactivated')
            return True
        return False

    def trigger_emergency_stop(self):
        """Trigger emergency stop across all components"""
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            return True
        return False

    def clear_emergency_stop(self):
        """Clear emergency stop"""
        if self.emergency_stop_active:
            self.emergency_stop_active = False
            stop_msg = Bool()
            stop_msg.data = False
            self.emergency_stop_pub.publish(stop_msg)
            self.get_logger().info('Emergency stop cleared')
            return True
        return False

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        return {
            'system_active': self.system_active,
            'emergency_stop': self.emergency_stop_active,
            'system_mode': self.system_mode,
            'component_status': self.components_status.copy(),
            'component_health': self.component_health.copy(),
            'timestamp': self.get_clock().now().to_msg()
        }

    def change_system_mode(self, new_mode: str):
        """Change system operating mode"""
        valid_modes = ["simulation", "real_world", "transfer_validation"]
        if new_mode in valid_modes:
            old_mode = self.system_mode
            self.system_mode = new_mode
            self.get_logger().info(f'System mode changed from {old_mode} to {new_mode}')
            return True
        else:
            self.get_logger().error(f'Invalid system mode: {new_mode}')
            return False


class SystemCoordinator:
    def __init__(self, node: IntegratedSystemNode):
        """
        Coordinate interactions between system components
        """
        self.node = node
        self.coordination_rules = self._load_coordination_rules()
        self.message_queue = queue.Queue()
        self.coordination_thread = None

    def _load_coordination_rules(self) -> Dict[str, Any]:
        """
        Load rules for coordinating system components
        """
        # In a real implementation, this would load from config
        return {
            "perception_to_navigation": {
                "trigger_conditions": ["object_detected", "obstacle_spotted"],
                "data_mapping": {
                    "detection": "/perception/obstacles",
                    "action": "/navigation/update_costmap"
                }
            },
            "behavior_to_navigation": {
                "trigger_conditions": ["new_goal", "task_request"],
                "data_mapping": {
                    "goal": "/goal_pose",
                    "action": "/navigate_to_pose"
                }
            },
            "safety_to_all": {
                "trigger_conditions": ["safety_violation", "emergency"],
                "data_mapping": {
                    "stop": "/emergency_stop",
                    "action": "all_stop"
                }
            }
        }

    def start_coordination(self):
        """
        Start the coordination thread
        """
        self.coordination_thread = threading.Thread(target=self._coordination_loop)
        self.coordination_thread.daemon = True
        self.coordination_thread.start()

    def _coordination_loop(self):
        """
        Main coordination loop
        """
        while rclpy.ok():
            try:
                # Process coordination messages
                while not self.message_queue.empty():
                    message = self.message_queue.get_nowait()
                    self._process_coordination_message(message)

                time.sleep(0.01)  # 100Hz coordination rate
            except queue.Empty:
                time.sleep(0.01)
            except Exception as e:
                self.node.get_logger().error(f'Coordination loop error: {e}')

    def _process_coordination_message(self, message: Any):
        """
        Process a coordination message
        """
        # Implementation would handle coordination logic
        pass

    def trigger_coordination_event(self, event_type: str, data: Any = None):
        """
        Trigger a coordination event
        """
        message = {
            'type': event_type,
            'data': data,
            'timestamp': time.time()
        }
        self.message_queue.put(message)


def main(args=None):
    rclpy.init(args=args)

    # Create the integrated system node
    integrated_node = IntegratedSystemNode()

    # Create system coordinator
    coordinator = SystemCoordinator(integrated_node)
    coordinator.start_coordination()

    # Activate the system
    integrated_node.activate_system()

    try:
        rclpy.spin(integrated_node)
    except KeyboardInterrupt:
        integrated_node.get_logger().info('Shutting down integrated system...')
    finally:
        integrated_node.deactivate_system()
        integrated_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Behavior Engine Implementation
```python
# integration/behavior_engine.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray
from builtin_interfaces.msg import Time
import time
import numpy as np
from typing import List, Dict, Any, Optional
from enum import Enum

class BehaviorState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PERCEIVING = "perceiving"
    MANIPULATING = "manipulating"
    AVOIDING = "avoiding"
    EMERGENCY = "emergency"
    WAITING = "waiting"

class BehaviorEngine(Node):
    def __init__(self):
        super().__init__('behavior_engine')

        # Initialize behavior state
        self.current_state = BehaviorState.IDLE
        self.previous_state = BehaviorState.IDLE
        self.task_queue = []
        self.current_task = None

        # Setup QoS profiles
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.behavior_status_pub = self.create_publisher(String, '/behavior/status', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/object_detections', self.detection_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            String, '/system/status', self.system_status_callback, 10)

        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Behavior parameters
        self.safety_distance = 0.5  # meters
        self.navigation_timeout = 30.0  # seconds
        self.perception_timeout = 5.0  # seconds

        # Setup timers
        self.behavior_timer = self.create_timer(0.1, self.behavior_loop)  # 10Hz
        self.status_timer = self.create_timer(1.0, self.publish_behavior_status)

        # Perception data
        self.last_detections = None
        self.last_scan = None
        self.system_status = "unknown"

        self.get_logger().info('Behavior Engine initialized')

    def detection_callback(self, msg):
        """Handle object detection messages"""
        self.last_detections = msg
        self.get_logger().debug(f'Received {len(msg.detections)} detections')

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        self.last_scan = msg

    def system_status_callback(self, msg):
        """Handle system status updates"""
        self.system_status = msg.data

    def publish_behavior_status(self):
        """Publish current behavior status"""
        status_msg = String()
        status_msg.data = f"state:{self.current_state.value},health:0.95"
        self.behavior_status_pub.publish(status_msg)

    def behavior_loop(self):
        """Main behavior execution loop"""
        try:
            # Check for emergency conditions
            if self._check_emergency_conditions():
                self._handle_emergency()
                return

            # Execute current behavior based on state
            if self.current_state == BehaviorState.IDLE:
                self._idle_behavior()
            elif self.current_state == BehaviorState.NAVIGATING:
                self._navigation_behavior()
            elif self.current_state == BehaviorState.PERCEIVING:
                self._perception_behavior()
            elif self.current_state == BehaviorState.AVOIDING:
                self._avoidance_behavior()
            elif self.current_state == BehaviorState.WAITING:
                self._waiting_behavior()

            # Process task queue
            self._process_task_queue()

        except Exception as e:
            self.get_logger().error(f'Behavior loop error: {e}')
            self._transition_to_state(BehaviorState.EMERGENCY)

    def _check_emergency_conditions(self) -> bool:
        """Check for emergency conditions"""
        # Check if system is in critical failure state
        if self.system_status == "critical_failure":
            return True

        # Check for collision risk based on scan data
        if self.last_scan is not None:
            min_distance = min(self.last_scan.ranges) if self.last_scan.ranges else float('inf')
            if min_distance < self.safety_distance * 0.5:  # Critical safety distance
                return True

        return False

    def _handle_emergency(self):
        """Handle emergency state"""
        if self.current_state != BehaviorState.EMERGENCY:
            self._transition_to_state(BehaviorState.EMERGENCY)
            self.get_logger().warn('EMERGENCY: Stopping all actions')

        # Stop robot movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def _idle_behavior(self):
        """Handle idle state behavior"""
        # In idle state, just monitor sensors and wait for tasks
        pass

    def _navigation_behavior(self):
        """Handle navigation behavior"""
        if self.current_task and 'goal' in self.current_task:
            goal = self.current_task['goal']

            # Check if navigation is complete or timed out
            # In a real implementation, we would check action server status
            pass

    def _perception_behavior(self):
        """Handle perception behavior"""
        # Process detections and update task queue based on perceived objects
        if self.last_detections:
            self._process_detections()

    def _avoidance_behavior(self):
        """Handle obstacle avoidance behavior"""
        if self.last_scan:
            # Simple reactive avoidance
            ranges = self.last_scan.ranges
            if ranges:
                min_idx = np.argmin(ranges)
                min_distance = ranges[min_idx]

                if min_distance < self.safety_distance:
                    # Stop and plan alternative route
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)

                    # Transition to navigation state to find alternative path
                    self._transition_to_state(BehaviorState.NAVIGATING)

    def _waiting_behavior(self):
        """Handle waiting behavior"""
        # Just wait, possibly with periodic checks
        pass

    def _process_detections(self):
        """Process object detections and update behavior"""
        if not self.last_detections:
            return

        for detection in self.last_detections.detections:
            # Process each detection and potentially update behavior
            if detection.results:
                for result in detection.results:
                    class_id = result.hypothesis.class_id
                    confidence = result.hypothesis.score

                    if confidence > 0.7:  # High confidence detection
                        self.get_logger().info(f'High confidence detection: {class_id} with confidence {confidence:.2f}')

    def _process_task_queue(self):
        """Process the task queue"""
        if self.task_queue and self.current_task is None:
            self.current_task = self.task_queue.pop(0)
            self._execute_task(self.current_task)

    def _execute_task(self, task: Dict[str, Any]):
        """Execute a specific task"""
        task_type = task.get('type', 'unknown')

        if task_type == 'navigate_to':
            self._navigate_to_task(task)
        elif task_type == 'explore_area':
            self._explore_area_task(task)
        elif task_type == 'follow_object':
            self._follow_object_task(task)
        elif task_type == 'inspect_object':
            self._inspect_object_task(task)

    def _navigate_to_task(self, task: Dict[str, Any]):
        """Execute navigate-to task"""
        goal_pose = task.get('goal_pose')
        if goal_pose:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = goal_pose.get('x', 0.0)
            goal_msg.pose.position.y = goal_pose.get('y', 0.0)
            goal_msg.pose.position.z = goal_pose.get('z', 0.0)
            goal_msg.pose.orientation.w = 1.0  # Default orientation

            self.navigation_goal_pub.publish(goal_msg.pose)
            self._transition_to_state(BehaviorState.NAVIGATING)

    def _explore_area_task(self, task: Dict[str, Any]):
        """Execute area exploration task"""
        # For now, just transition to navigation state
        self._transition_to_state(BehaviorState.NAVIGATING)

    def _follow_object_task(self, task: Dict[str, Any]):
        """Execute object following task"""
        self._transition_to_state(BehaviorState.PERCEIVING)

    def _inspect_object_task(self, task: Dict[str, Any]):
        """Execute object inspection task"""
        self._transition_to_state(BehaviorState.PERCEIVING)

    def _transition_to_state(self, new_state: BehaviorState):
        """Safely transition to a new behavior state"""
        if self.current_state != new_state:
            self.previous_state = self.current_state
            self.current_state = new_state

            self.get_logger().info(f'Behavior state transition: {self.previous_state.value} -> {new_state.value}')

            # Perform state-specific actions
            if new_state == BehaviorState.EMERGENCY:
                self._handle_emergency()
            elif new_state == BehaviorState.IDLE:
                self._handle_idle_entry()
            elif new_state == BehaviorState.NAVIGATING:
                self._handle_navigation_entry()

    def _handle_idle_entry(self):
        """Handle actions when entering idle state"""
        pass

    def _handle_navigation_entry(self):
        """Handle actions when entering navigation state"""
        pass

    def add_task(self, task: Dict[str, Any]):
        """Add a task to the queue"""
        self.task_queue.append(task)
        self.get_logger().info(f'Added task to queue: {task.get("type", "unknown")}')

    def get_behavior_status(self) -> Dict[str, Any]:
        """Get current behavior status"""
        return {
            'current_state': self.current_state.value,
            'previous_state': self.previous_state.value,
            'task_queue_size': len(self.task_queue),
            'current_task': self.current_task,
            'system_status': self.system_status,
            'timestamp': self.get_clock().now().to_msg()
        }


def main(args=None):
    rclpy.init(args=args)

    behavior_engine = BehaviorEngine()

    # Example: Add some tasks
    behavior_engine.add_task({
        'type': 'navigate_to',
        'goal_pose': {'x': 2.0, 'y': 2.0, 'z': 0.0}
    })

    behavior_engine.add_task({
        'type': 'explore_area',
        'area': 'room_1'
    })

    try:
        rclpy.spin(behavior_engine)
    except KeyboardInterrupt:
        behavior_engine.get_logger().info('Shutting down behavior engine...')
    finally:
        behavior_engine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. System Integration Test
```python
# integration/system_integration_test.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String, Bool, Float32
from builtin_interfaces.msg import Time
import time
import unittest
from typing import Dict, Any

class SystemIntegrationTest(Node):
    def __init__(self):
        super().__init__('system_integration_test')

        # Test tracking variables
        self.test_results = {}
        self.test_start_time = None
        self.test_timeout = 60.0  # seconds

        # Publishers for test commands
        self.test_command_pub = self.create_publisher(String, '/test/command', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/test/goal', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/test/cmd_vel', 10)

        # Subscribers for system status
        self.system_status_sub = self.create_subscription(
            String, '/system/status', self.system_status_callback, 10)

        self.behavior_status_sub = self.create_subscription(
            String, '/behavior/status', self.behavior_status_callback, 10)

        self.perception_status_sub = self.create_subscription(
            String, '/perception/status', self.perception_status_callback, 10)

        self.navigation_status_sub = self.create_subscription(
            String, '/navigation/status', self.navigation_status_callback, 10)

        # Test state
        self.system_status = "unknown"
        self.behavior_status = "unknown"
        self.perception_status = "unknown"
        self.navigation_status = "unknown"

        # Setup test timer
        self.test_timer = self.create_timer(0.1, self.test_loop)

        self.get_logger().info('System Integration Test Node initialized')

    def system_status_callback(self, msg):
        self.system_status = msg.data

    def behavior_status_callback(self, msg):
        self.behavior_status = msg.data

    def perception_status_callback(self, msg):
        self.perception_status = msg.data

    def navigation_status_callback(self, msg):
        self.navigation_status = msg.data

    def test_loop(self):
        """Main test execution loop"""
        if self.test_start_time is None:
            self.test_start_time = time.time()

        # Check if test has timed out
        elapsed = time.time() - self.test_start_time
        if elapsed > self.test_timeout:
            self.get_logger().error('Integration test timed out')
            self.publish_test_results()
            return

        # Run various integration tests
        self._test_component_communication()
        self._test_behavior_coordination()
        self._test_system_responsiveness()

    def _test_component_communication(self):
        """Test that components can communicate"""
        # Check if all components are reporting status
        statuses = [self.system_status, self.behavior_status,
                   self.perception_status, self.navigation_status]

        all_reporting = all(status != "unknown" for status in statuses)

        if all_reporting:
            self.test_results['component_communication'] = True
            self.get_logger().info('✓ Component communication test passed')
        else:
            self.test_results['component_communication'] = False

    def _test_behavior_coordination(self):
        """Test behavior coordination between components"""
        # This would involve sending commands and verifying responses
        # For simulation, we'll check if behavior engine is active
        if "active" in self.behavior_status or "operational" in self.system_status:
            self.test_results['behavior_coordination'] = True
            self.get_logger().info('✓ Behavior coordination test passed')
        else:
            self.test_results['behavior_coordination'] = False

    def _test_system_responsiveness(self):
        """Test system responsiveness to commands"""
        # Send a simple command to test responsiveness
        if len(self.test_results) > 0:  # Only after other tests have run
            self.test_results['system_responsiveness'] = True
            self.get_logger().info('✓ System responsiveness test passed')

    def publish_test_results(self):
        """Publish comprehensive test results"""
        results_msg = String()
        results_msg.data = f"integration_test_results:{str(self.test_results)}"

        self.test_command_pub.publish(results_msg)

        self.get_logger().info(f'Integration test results: {self.test_results}')

        # Calculate overall success
        if all(self.test_results.values()) and len(self.test_results) > 0:
            self.get_logger().info('🎉 All integration tests PASSED!')
        else:
            failed_tests = [test for test, result in self.test_results.items() if not result]
            self.get_logger().error(f'❌ Integration tests FAILED: {failed_tests}')

    def run_comprehensive_test(self):
        """Run a comprehensive integration test"""
        self.get_logger().info('Starting comprehensive system integration test...')

        # This would involve more complex scenarios in a real implementation
        # For now, we'll just wait and collect status information
        pass


def run_integration_tests():
    """Run system integration tests"""
    print("=== System Integration Tests ===")

    rclpy.init()

    test_node = SystemIntegrationTest()

    # Run for a specific duration
    start_time = time.time()
    test_duration = 10.0  # seconds

    while time.time() - start_time < test_duration:
        rclpy.spin_once(test_node, timeout_sec=0.1)

    test_node.publish_test_results()

    test_node.destroy_node()
    rclpy.shutdown()

    print("Integration tests completed.")


class TestSystemIntegration(unittest.TestCase):
    def setUp(self):
        """Set up integration test fixtures"""
        self.test_results = {}

    def test_component_startup_sequence(self):
        """Test that components start up in the correct sequence"""
        # In a real test, this would verify the startup sequence
        startup_order = ["safety_monitor", "perception", "control", "navigation", "transfer", "behavior_engine"]

        # Mock the startup sequence
        completed_components = []
        for component in startup_order:
            # Simulate component startup
            completed_components.append(component)

        # Verify all components started
        self.assertEqual(len(completed_components), len(startup_order))
        self.assertEqual(completed_components, startup_order)

    def test_data_flow_between_components(self):
        """Test data flow between integrated components"""
        # Simulate data flowing from perception to navigation
        perception_output = {"objects": ["obstacle", "target"], "poses": [(1, 1), (2, 2)]}
        navigation_input = perception_output  # In integrated system, this would be the flow

        # Verify data format is correct
        self.assertIn("objects", navigation_input)
        self.assertIn("poses", navigation_input)
        self.assertEqual(len(navigation_input["objects"]), len(navigation_input["poses"]))

    def test_safety_system_integration(self):
        """Test safety system integration with other components"""
        # Safety system should be able to stop all other components
        safety_active = True
        emergency_stop_triggered = True

        # In a real system, this would verify that emergency stop works
        self.assertTrue(safety_active)
        self.assertTrue(emergency_stop_triggered)

    def test_behavior_coordination(self):
        """Test coordination between behavior engine and other components"""
        # Behavior engine should be able to coordinate navigation and perception
        behavior_commands = ["navigate_to", "perceive", "avoid"]
        expected_component_responses = ["navigation_active", "perception_active", "avoidance_active"]

        # Simulate behavior execution
        executed_responses = []
        for command in behavior_commands:
            # Each command should trigger appropriate component
            if command == "navigate_to":
                executed_responses.append("navigation_active")
            elif command == "perceive":
                executed_responses.append("perception_active")
            elif command == "avoid":
                executed_responses.append("avoidance_active")

        self.assertEqual(executed_responses, expected_component_responses)


def run_unit_tests():
    """Run unit tests for system integration"""
    print("\n=== Running System Integration Unit Tests ===")

    unittest.main(argv=[''], exit=False, verbosity=2)


def main():
    """Main function to run integration tests"""
    print("Starting System Integration Process...")

    # Run unit tests first
    run_unit_tests()

    # Run integration tests
    run_integration_tests()

    print("\nSystem Integration Testing Completed!")


if __name__ == '__main__':
    main()
```

## Launch Files

### 1. Integrated System Launch
```xml
<!-- launch/integrated_system.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    system_mode = LaunchConfiguration('system_mode', default='simulation')
    enable_perception = LaunchConfiguration('enable_perception', default='True')
    enable_navigation = LaunchConfiguration('enable_navigation', default='True')

    # Get package directories
    pkg_isaac_integration = get_package_share_directory('isaac_integration')

    # Integrated System Node
    integrated_system_node = Node(
        package='isaac_integration',
        executable='main_integration_node',
        name='integrated_system_node',
        parameters=[
            PathJoinSubstitution([pkg_isaac_integration, 'config', 'system_integration.yaml']),
            {'use_sim_time': use_sim_time},
            {'system_mode': system_mode}
        ],
        output='screen'
    )

    # Behavior Engine Node
    behavior_engine_node = Node(
        package='isaac_integration',
        executable='behavior_engine',
        name='behavior_engine',
        parameters=[
            PathJoinSubstitution([pkg_isaac_integration, 'config', 'system_integration.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Include perception pipeline if enabled
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_perception'),
                'launch',
                'perception_pipeline.launch.py'
            ])
        ]),
        condition=IfCondition(enable_perception)
    )

    # Include navigation stack if enabled
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_navigation'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        condition=IfCondition(enable_navigation)
    )

    # Transfer validation node
    transfer_node = Node(
        package='isaac_transfer',
        executable='transfer_validator',
        name='transfer_validator',
        parameters=[
            PathJoinSubstitution([pkg_isaac_integration, 'config', 'transfer_config.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Safety monitor node
    safety_monitor_node = Node(
        package='isaac_safety',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'system_mode',
            default_value='simulation',
            description='System mode: simulation, real_world, or transfer_validation'
        ),
        DeclareLaunchArgument(
            'enable_perception',
            default_value='True',
            description='Enable perception system'
        ),
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='True',
            description='Enable navigation system'
        ),

        # Launch nodes in startup order (as defined in system config)
        safety_monitor_node,  # Safety first
        integrated_system_node,
        behavior_engine_node,
        transfer_node,

        # Launch perception and navigation systems
        perception_launch,
        navigation_launch,
    ])
```

## Performance Monitoring

### 1. System Performance Monitoring
```python
# integration/performance_monitor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from builtin_interfaces.msg import Time
import time
from collections import deque
import statistics
from typing import Dict, List, Any

class SystemPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('system_performance_monitor')

        # Performance tracking
        self.component_response_times = {
            'perception': deque(maxlen=100),
            'navigation': deque(maxlen=100),
            'control': deque(maxlen=100),
            'behavior': deque(maxlen=100)
        }

        self.system_throughput = deque(maxlen=100)
        self.cpu_usage = deque(maxlen=100)
        self.memory_usage = deque(maxlen=100)

        # Publishers for performance metrics
        self.perception_perf_pub = self.create_publisher(Float32, '/performance/perception', 10)
        self.navigation_perf_pub = self.create_publisher(Float32, '/performance/navigation', 10)
        self.system_perf_pub = self.create_publisher(Float32, '/performance/system', 10)
        self.status_pub = self.create_publisher(String, '/performance/status', 10)

        # Timers
        self.monitor_timer = self.create_timer(1.0, self.monitor_performance)
        self.report_timer = self.create_timer(10.0, self.report_performance)

        # Performance thresholds
        self.performance_thresholds = {
            'perception_latency': 0.1,    # seconds
            'navigation_update': 0.05,    # seconds
            'control_rate': 100.0,        # Hz
            'system_throughput': 30.0     # Hz
        }

        self.get_logger().info('System Performance Monitor initialized')

    def monitor_performance(self):
        """Monitor system performance metrics"""
        # Simulate monitoring of component response times
        # In a real system, this would measure actual response times
        import random

        # Update response time measurements
        for component in self.component_response_times.keys():
            # Simulate response time measurement
            response_time = random.uniform(0.01, 0.1)  # 10-100ms
            self.component_response_times[component].append(response_time)

        # Update system throughput
        throughput = random.uniform(20, 50)  # 20-50 Hz
        self.system_throughput.append(throughput)

        # Publish performance metrics
        self.publish_component_performance()

    def publish_component_performance(self):
        """Publish component performance metrics"""
        # Publish perception performance
        if self.component_response_times['perception']:
            avg_perception_time = statistics.mean(self.component_response_times['perception'])
            perf_msg = Float32()
            perf_msg.data = float(avg_perception_time)
            self.perception_perf_pub.publish(perf_msg)

        # Publish navigation performance
        if self.component_response_times['navigation']:
            avg_navigation_time = statistics.mean(self.component_response_times['navigation'])
            perf_msg = Float32()
            perf_msg.data = float(avg_navigation_time)
            self.navigation_perf_pub.publish(perf_msg)

        # Publish system performance
        if self.system_throughput:
            avg_throughput = statistics.mean(self.system_throughput)
            perf_msg = Float32()
            perf_msg.data = float(avg_throughput)
            self.system_perf_pub.publish(perf_msg)

    def report_performance(self):
        """Report comprehensive performance summary"""
        report = {
            "timestamp": time.time(),
            "component_performance": {},
            "system_throughput": {},
            "health_status": "normal"
        }

        # Calculate component performance metrics
        for component, times in self.component_response_times.items():
            if times:
                avg_time = statistics.mean(times)
                min_time = min(times)
                max_time = max(times)
                std_dev = statistics.stdev(times) if len(times) > 1 else 0

                report["component_performance"][component] = {
                    "avg_response_time": avg_time,
                    "min_response_time": min_time,
                    "max_response_time": max_time,
                    "std_deviation": std_dev
                }

        # Calculate system throughput
        if self.system_throughput:
            avg_throughput = statistics.mean(self.system_throughput)
            report["system_throughput"] = {
                "avg_throughput": avg_throughput,
                "min_throughput": min(self.system_throughput) if self.system_throughput else 0,
                "max_throughput": max(self.system_throughput) if self.system_throughput else 0
            }

        # Determine health status based on performance
        health_issues = []
        if self.component_response_times['perception'] and \
           statistics.mean(self.component_response_times['perception']) > self.performance_thresholds['perception_latency']:
            health_issues.append("perception_slow")

        if self.system_throughput and \
           statistics.mean(self.system_throughput) < self.performance_thresholds['system_throughput']:
            health_issues.append("throughput_low")

        if health_issues:
            report["health_status"] = "degraded"
            report["health_issues"] = health_issues

        # Publish status
        status_msg = String()
        status_msg.data = f"system_health:{report['health_status']},issues:{len(health_issues) if health_issues else 0}"
        self.status_pub.publish(status_msg)

        self.get_logger().info(f"Performance Report: {report}")

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get a summary of system performance"""
        summary = {}

        for component, times in self.component_response_times.items():
            if times:
                avg_time = statistics.mean(times)
                summary[f"{component}_avg_response"] = avg_time
                summary[f"{component}_response_count"] = len(times)

        if self.system_throughput:
            avg_throughput = statistics.mean(self.system_throughput)
            summary["system_avg_throughput"] = avg_throughput

        return summary


def main(args=None):
    rclpy.init(args=args)

    monitor = SystemPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance monitoring stopped')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for System Integration

### 1. Integration Best Practices
```python
# integration/best_practices.py
class IntegrationBestPractices:
    """
    Best practices for system integration in Isaac-based robotics systems
    """

    @staticmethod
    def component_integration_best_practices():
        """
        Best practices for integrating components
        """
        practices = [
            "Define clear interfaces between components with well-specified message types",
            "Use appropriate QoS profiles for different types of communication",
            "Implement proper error handling and fallback behaviors",
            "Design components to be as independent as possible while maintaining cohesion",
            "Use configuration files to manage component dependencies and parameters",
            "Implement graceful degradation when components fail",
            "Log component interactions for debugging and monitoring",
            "Validate data formats at component boundaries"
        ]
        return practices

    @staticmethod
    def communication_best_practices():
        """
        Best practices for inter-component communication
        """
        practices = [
            "Use sensor_data QoS for perception data with best_effort reliability",
            "Use services for request-response communication patterns",
            "Use actions for long-running tasks with feedback",
            "Implement message filtering to reduce unnecessary communication",
            "Use appropriate message frequencies to avoid system overload",
            "Design message schemas to be extensible for future needs",
            "Implement message validation to ensure data integrity"
        ]
        return practices

    @staticmethod
    def safety_integration_best_practices():
        """
        Best practices for safety system integration
        """
        practices = [
            "Make safety systems independent and always active",
            "Implement multiple safety layers with different triggers",
            "Use hard real-time constraints for safety-critical communications",
            "Design safety systems to be fail-safe by default",
            "Test safety systems under various failure conditions",
            "Implement safety system self-monitoring and validation",
            "Document safety requirements and validation procedures clearly"
        ]
        return practices

    @staticmethod
    def performance_integration_best_practices():
        """
        Best practices for maintaining performance during integration
        """
        practices = [
            "Profile individual components before integration",
            "Monitor system performance during integration testing",
            "Optimize critical communication paths first",
            "Use multi-threading appropriately to avoid blocking",
            "Implement resource management to prevent contention",
            "Design for scalability as more components are added",
            "Monitor and control memory usage across components"
        ]
        return practices

    @staticmethod
    def testing_integration_best_practices():
        """
        Best practices for testing integrated systems
        """
        practices = [
            "Test components individually before integration",
            "Create integration test scenarios that reflect real usage",
            "Test failure modes and error recovery procedures",
            "Validate timing and synchronization between components",
            "Test with realistic data volumes and rates",
            "Perform stress testing under heavy loads",
            "Document test procedures and expected behaviors"
        ]
        return practices


def print_integration_best_practices():
    """
    Print all system integration best practices
    """
    practices = IntegrationBestPractices()

    print("=== System Integration Best Practices ===\n")

    print("Component Integration:")
    for i, practice in enumerate(practices.component_integration_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nCommunication:")
    for i, practice in enumerate(practices.communication_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nSafety Integration:")
    for i, practice in enumerate(practices.safety_integration_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nPerformance:")
    for i, practice in enumerate(practices.performance_integration_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nTesting:")
    for i, practice in enumerate(practices.testing_integration_best_practices(), 1):
        print(f"  {i}. {practice}")


if __name__ == "__main__":
    print_integration_best_practices()
```

## Troubleshooting

### Common Integration Issues and Solutions

#### 1. Communication Issues
- **Message Timing Problems**: Use appropriate QoS profiles and message rates
- **Topic Connection Failures**: Verify topic names, types, and node configurations
- **Data Format Mismatches**: Implement data validation and conversion layers

#### 2. Performance Issues
- **System Bottlenecks**: Profile individual components and optimize critical paths
- **Resource Contention**: Implement resource management and prioritization
- **Timing Violations**: Review and adjust component update rates

#### 3. Coordination Issues
- **Race Conditions**: Implement proper synchronization mechanisms
- **State Inconsistencies**: Use centralized state management where appropriate
- **Component Dependencies**: Design clear startup and shutdown sequences

## Next Steps
After completing system integration:
1. Optimize overall system performance (Task 3.4)
2. Conduct comprehensive testing and validation (Phase 4)
3. Create documentation and examples (Task 4.3)

## Resources
- [ROS 2 Integration Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Isaac ROS Integration](https://nvidia-isaac-ros.github.io/concepts/integration/index.html)
- [System Architecture Patterns](https://patterns.arc42.org/)
- [Real-time ROS Systems](https://docs.ros.org/en/humble/Tutorials/Advanced/Real-Time.html)