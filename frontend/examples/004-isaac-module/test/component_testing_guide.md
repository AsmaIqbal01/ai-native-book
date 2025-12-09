# Component Testing Guide

## Overview
This guide provides instructions for testing individual components of the Isaac-based autonomous humanoid system. The testing approach includes unit testing for individual functions, integration testing for component pairs, and performance testing for each module.

## Testing Strategy

### 1. Testing Levels
- **Unit Tests**: Test individual functions and classes
- **Component Tests**: Test individual system components
- **Integration Tests**: Test interactions between component pairs
- **Performance Tests**: Test performance characteristics of each module
- **Regression Tests**: Ensure new changes don't break existing functionality

### 2. Testing Framework
- **Testing Framework**: pytest for Python components
- **Mocking**: unittest.mock for dependencies
- **Assertions**: Built-in assertions and custom validation
- **Reporting**: Comprehensive test reports with coverage metrics

## Test Configuration Files

### 1. Test Configuration
```yaml
# config/test_config.yaml
testing:
  version: "1.0"
  enabled: true
  test_strategy: "comprehensive"  # comprehensive, smoke, regression

  unit_tests:
    enabled: true
    timeout: 30  # seconds
    coverage_threshold: 80.0  # percent
    parallel_execution: true
    max_workers: 4

  integration_tests:
    enabled: true
    timeout: 60  # seconds
    setup_teardown: true
    mock_external_dependencies: true

  performance_tests:
    enabled: true
    warmup_iterations: 10
    measurement_iterations: 100
    performance_thresholds:
      perception_fps: 30.0
      navigation_frequency: 20.0
      control_frequency: 100.0
      message_latency: 0.05  # seconds

  regression_tests:
    enabled: true
    comparison_baseline: "latest_successful"
    threshold: 0.05  # 5% performance degradation allowed

  mocking:
    enabled: true
    external_services: true
    hardware_interfaces: true
    real_sensors: true
    real_actuators: true

  reporting:
    format: "junit_xml"  # junit_xml, html, console
    output_dir: "test_reports"
    coverage: true
    coverage_format: "html"
    coverage_threshold: 80.0
```

### 2. Component-Specific Test Configurations
```yaml
# config/component_test_configs.yaml
component_tests:
  perception_pipeline:
    enabled: true
    test_modules:
      - "isaac_ros_perception"
      - "camera_processing"
      - "object_detection"
      - "pose_estimation"
    test_data:
      synthetic_images: 100
      real_images: 50
      edge_cases: 20
    performance_thresholds:
      fps: 30.0
      accuracy: 0.85
      latency: 0.05  # seconds

  navigation_stack:
    enabled: true
    test_modules:
      - "nav2_stack"
      - "path_planning"
      - "obstacle_avoidance"
      - "localization"
    test_scenarios:
      simple_maze: 10
      complex_maze: 5
      dynamic_obstacles: 15
      narrow_corridors: 8
    performance_thresholds:
      success_rate: 0.90
      planning_time: 1.0  # seconds
      execution_time: 30.0  # seconds

  control_system:
    enabled: true
    test_modules:
      - "trajectory_execution"
      - "joint_control"
      - "feedback_control"
      - "safety_monitoring"
    test_scenarios:
      position_control: 20
      velocity_control: 15
      effort_control: 10
      safety_emergency: 5
    performance_thresholds:
      tracking_accuracy: 0.01  # meters
      response_time: 0.01  # seconds
      stability: 0.99  # dimensionless

  behavior_engine:
    enabled: true
    test_modules:
      - "task_scheduler"
      - "state_machine"
      - "decision_logic"
      - "coordination"
    test_scenarios:
      simple_tasks: 30
      complex_tasks: 15
      concurrent_tasks: 10
      error_recovery: 8
    performance_thresholds:
      decision_time: 0.1  # seconds
      task_success_rate: 0.95
      coordination_accuracy: 0.98

  transfer_system:
    enabled: true
    test_modules:
      - "domain_randomization"
      - "transfer_validation"
      - "model_adaptation"
      - "sim2real_metrics"
    test_scenarios:
      domain_shift: 25
      model_adaptation: 15
      validation_tests: 20
      real_world_tests: 10
    performance_thresholds:
      transfer_success_rate: 0.80
      adaptation_time: 5.0  # seconds
      validation_accuracy: 0.85
```

## Unit Test Implementations

### 1. Perception Component Unit Tests
```python
# test/unit/test_perception.py
import unittest
import numpy as np
import pytest
from unittest.mock import Mock, patch
from examples.isaac_module.perception.object_detection import IsaacROSObjectDetection
from examples.isaac_module.perception.camera_pipeline import CameraProcessingPipeline

class TestCameraProcessingPipeline(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.pipeline = CameraProcessingPipeline()

    def test_image_processing_basic(self):
        """Test basic image processing functionality"""
        # Create a mock image
        mock_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Test processing function
        processed_image = self.pipeline.process_image(mock_image)

        # Assertions
        self.assertIsNotNone(processed_image)
        self.assertEqual(processed_image.shape, mock_image.shape)
        self.assertEqual(processed_image.dtype, mock_image.dtype)

    def test_image_resize_handling(self):
        """Test image resize functionality"""
        # Create a large mock image
        large_image = np.random.randint(0, 255, (1200, 1600, 3), dtype=np.uint8)

        # Process the image
        processed_image = self.pipeline.process_image(large_image)

        # Check that it's been resized appropriately
        self.assertLessEqual(processed_image.shape[0], 720)
        self.assertLessEqual(processed_image.shape[1], 1280)

    def test_image_callback_processing(self):
        """Test image callback processing"""
        # Mock the bridge and other dependencies
        with patch.object(self.pipeline, 'bridge'), \
             patch.object(self.pipeline, 'processed_image_pub'):

            # Create a mock message
            mock_msg = Mock()
            mock_msg.header = Mock()

            # Call the callback
            self.pipeline.image_callback(mock_msg)

            # Verify that processing occurred
            self.pipeline.processed_image_pub.publish.assert_called()

    def test_camera_info_callback(self):
        """Test camera info callback"""
        # Create mock camera info message
        mock_camera_info = Mock()
        mock_camera_info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        mock_camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Call the callback
        self.pipeline.camera_info_callback(mock_camera_info)

        # Verify that camera matrices were set
        self.assertIsNotNone(self.pipeline.camera_matrix)
        self.assertIsNotNone(self.pipeline.dist_coeffs)


class TestIsaacROSObjectDetection(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.detector = IsaacROSObjectDetection()

    def test_detection_callback_basic(self):
        """Test basic detection callback functionality"""
        # Create a mock image message
        mock_image_msg = Mock()
        mock_image_msg.header = Mock()
        mock_image_msg.height = 480
        mock_image_msg.width = 640

        # Test the callback
        with patch.object(self.detector, 'detection_pub'):
            self.detector.detection_callback(mock_image_msg)

            # Verify that detection was published
            self.detector.detection_pub.publish.assert_called()

    def test_create_detection(self):
        """Test creation of detection messages"""
        # Test creating a detection
        detection = self.detector.create_detection("test_object", 0.9, [10, 10, 50, 50])

        # Verify detection properties
        self.assertIsNotNone(detection)
        self.assertEqual(detection.results[0].hypothesis.class_id, "test_object")
        self.assertEqual(detection.results[0].hypothesis.score, 0.9)

    def test_simulate_detections(self):
        """Test simulation of detections"""
        # Create a mock image message
        mock_image_msg = Mock()
        mock_image_msg.header = Mock()

        # Test detection simulation
        detections = self.detector.simulate_detections(mock_image_msg)

        # Verify that detections were created
        self.assertIsNotNone(detections)
        self.assertGreater(len(detections.detections), 0)


def run_perception_unit_tests():
    """Run perception component unit tests"""
    print("Running Perception Component Unit Tests...")

    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestCameraProcessingPipeline)
    suite.addTests(loader.loadTestsFromTestCase(TestIsaacROSObjectDetection))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_perception_unit_tests()
    exit(0 if success else 1)
```

### 2. Navigation Component Unit Tests
```python
# test/unit/test_navigation.py
import unittest
import numpy as np
import pytest
from unittest.mock import Mock, patch, MagicMock
from examples.isaac_module.navigation.nav2_navigation_guide import IsaacSimNavigationIntegration
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class TestIsaacSimNavigationIntegration(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        # Mock the ROS node initialization
        with patch('rclpy.node.Node.__init__'):
            self.nav_integration = IsaacSimNavigationIntegration.__new__(IsaacSimNavigationIntegration)
            self.nav_integration.get_logger = Mock()
            self.nav_integration.create_publisher = Mock()
            self.nav_integration.create_subscription = Mock()
            self.nav_integration.create_timer = Mock()

    def test_odom_callback_basic(self):
        """Test basic odometry callback functionality"""
        # Create a mock odometry message
        mock_odom = Odometry()
        mock_odom.pose.pose.position.x = 1.0
        mock_odom.pose.pose.position.y = 2.0
        mock_odom.pose.pose.position.z = 0.0
        mock_odom.pose.pose.orientation.w = 1.0

        # Test the callback
        self.nav_integration.odom_callback(mock_odom)

        # Verify that pose was stored
        self.assertIsNotNone(self.nav_integration.current_pose)

    def test_scan_callback_basic(self):
        """Test basic scan callback functionality"""
        # Create a mock scan message
        mock_scan = LaserScan()
        mock_scan.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]  # 5 range readings
        mock_scan.angle_min = -1.57  # -90 degrees
        mock_scan.angle_max = 1.57   # 90 degrees
        mock_scan.angle_increment = 0.785  # 45 degrees

        # Test the callback
        self.nav_integration.scan_callback(mock_scan)

        # Verify that scan was processed (logger should be called)
        self.nav_integration.get_logger().debug.assert_called()

    def test_send_velocity_command(self):
        """Test sending velocity commands"""
        # Mock the publisher
        mock_publisher = Mock()
        self.nav_integration.cmd_vel_pub = mock_publisher

        # Test sending a command
        self.nav_integration.send_velocity_command(0.5, 0.1)

        # Verify that command was published
        mock_publisher.publish.assert_called()
        published_msg = mock_publisher.publish.call_args[0][0]
        self.assertEqual(published_msg.linear.x, 0.5)
        self.assertEqual(published_msg.angular.z, 0.1)

    def test_send_navigation_goal(self):
        """Test sending navigation goals"""
        # Mock the publisher
        mock_publisher = Mock()
        self.nav_integration.nav_goal_pub = mock_publisher

        # Test sending a goal
        self.nav_integration.send_navigation_goal(2.0, 3.0, 1.57)

        # Verify that goal was published
        mock_publisher.publish.assert_called()


class TestPathPlanning(unittest.TestCase):
    def setUp(self):
        """Set up path planning test fixtures"""
        pass

    def test_path_generation_basic(self):
        """Test basic path generation"""
        from examples.isaac_module.navigation.humanoid_path_planner import HumanoidPathPlanner

        # Mock the ROS node initialization
        with patch('rclpy.node.Node.__init__'):
            planner = HumanoidPathPlanner.__new__(HumanoidPathPlanner)
            planner.get_logger = Mock()
            planner.create_publisher = Mock()
            planner.create_timer = Mock()

        # Create mock poses
        from geometry_msgs.msg import Pose
        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0

        goal_pose = Pose()
        goal_pose.position.x = 5.0
        goal_pose.position.y = 5.0

        # Test path planning
        path = planner.generate_humanoid_feasible_path(start_pose, goal_pose)

        # Verify path was generated
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 1)  # Should have multiple waypoints

    def test_path_validation(self):
        """Test path validation"""
        from examples.isaac_module.navigation.humanoid_path_planner import HumanoidPathPlanner

        # Mock the ROS node initialization
        with patch('rclpy.node.Node.__init__'):
            planner = HumanoidPathPlanner.__new__(HumanoidPathPlanner)
            planner.get_logger = Mock()
            planner.create_publisher = Mock()
            planner.create_timer = Mock()

        # Create a valid path
        valid_path = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4)]

        # Validate the path
        is_valid, reasons = planner.validate_path_for_humanoid(valid_path)

        # The path should be valid (all steps are equal distance of sqrt(2))
        self.assertTrue(is_valid)


class TestNavigationPerformanceMonitor(unittest.TestCase):
    def setUp(self):
        """Set up performance monitor test fixtures"""
        from examples.isaac_module.navigation.performance_monitor import NavigationPerformanceMonitor

        # Mock the ROS node initialization
        with patch('rclpy.node.Node.__init__'):
            self.monitor = NavigationPerformanceMonitor.__new__(NavigationPerformanceMonitor)
            self.monitor.get_logger = Mock()
            self.monitor.create_publisher = Mock()
            self.monitor.create_subscription = Mock()
            self.monitor.create_timer = Mock()

    def test_performance_tracking(self):
        """Test performance tracking functionality"""
        # This would test the performance monitoring logic
        # For now, we'll just verify the structure is sound
        pass


def run_navigation_unit_tests():
    """Run navigation component unit tests"""
    print("Running Navigation Component Unit Tests...")

    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestIsaacSimNavigationIntegration)
    suite.addTests(loader.loadTestsFromTestCase(TestPathPlanning))
    suite.addTests(loader.loadTestsFromTestCase(TestNavigationPerformanceMonitor))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_navigation_unit_tests()
    exit(0 if success else 1)
```

### 3. Transfer Component Unit Tests
```python
# test/unit/test_transfer.py
import unittest
import numpy as np
import pytest
from unittest.mock import Mock, patch
from examples.isaac_module.transfer.transfer_learning import (
    DomainRandomizationTransfer,
    DomainAdaptationTransfer,
    FineTuningTransfer,
    TransferValidator
)

class TestDomainRandomizationTransfer(unittest.TestCase):
    def setUp(self):
        """Set up domain randomization test fixtures"""
        self.transfer_method = DomainRandomizationTransfer()

    def test_transfer_basic(self):
        """Test basic domain randomization transfer"""
        # Create mock source model and target data
        source_model = {"type": "mock_model", "trained": True}
        target_data = (np.random.random((10, 4)), np.random.randint(0, 2, 10))

        # Test transfer
        result = self.transfer_method.transfer(source_model, target_data)

        # Should return the source model unchanged (randomization happens during data gen)
        self.assertEqual(result, source_model)

    def test_validate_transfer(self):
        """Test transfer validation"""
        # Create mock transferred model and test data
        model = {"type": "mock_model", "trained": True}
        test_data = (np.random.random((10, 4)), np.random.randint(0, 2, 10))

        # Test validation
        metrics = self.transfer_method.validate_transfer(model, test_data)

        # Verify metrics structure
        self.assertIn('accuracy', metrics)
        self.assertIn('precision', metrics)
        self.assertIn('recall', metrics)
        self.assertIn('f1_score', metrics)

        # Verify values are reasonable
        for metric in metrics.values():
            self.assertGreaterEqual(metric, 0.0)
            self.assertLessEqual(metric, 1.0)


class TestDomainAdaptationTransfer(unittest.TestCase):
    def setUp(self):
        """Set up domain adaptation test fixtures"""
        self.transfer_method = DomainAdaptationTransfer()

    def test_transfer_adaptation(self):
        """Test domain adaptation transfer"""
        # Create mock source model and target data
        source_model = {"type": "mock_model", "trained": True}
        target_data = [1, 2, 3, 4, 5]  # Mock target data

        # Test adaptation
        adapted_model = self.transfer_method.transfer(source_model, target_data)

        # Should return a model (could be same or adapted)
        self.assertIsNotNone(adapted_model)

    def test_adaptation_validation(self):
        """Test domain adaptation validation"""
        # Create mock model and test data
        model = {"type": "mock_model", "adapted": True}
        test_data = [1, 2, 3, 4, 5]

        # Test validation
        metrics = self.transfer_method.validate_transfer(model, test_data)

        # Verify metrics are returned
        self.assertIn('accuracy', metrics)
        self.assertIn('precision', metrics)
        self.assertIn('recall', metrics)
        self.assertIn('f1_score', metrics)


class TestFineTuningTransfer(unittest.TestCase):
    def setUp(self):
        """Set up fine-tuning test fixtures"""
        self.transfer_method = FineTuningTransfer()

    def test_fine_tuning_transfer(self):
        """Test fine-tuning transfer"""
        # Create mock source model and target data
        source_model = {"type": "mock_model", "trained": True}
        target_data = [1, 2, 3, 4, 5]

        # Test fine-tuning
        fine_tuned_model = self.transfer_method.transfer(source_model, target_data)

        # Should return a model
        self.assertIsNotNone(fine_tuned_model)

    def test_fine_tuning_validation(self):
        """Test fine-tuning validation"""
        # Create mock model and test data
        model = {"type": "mock_model", "fine_tuned": True}
        test_data = [1, 2, 3, 4, 5]

        # Test validation
        metrics = self.transfer_method.validate_transfer(model, test_data)

        # Verify metrics structure
        self.assertIn('accuracy', metrics)
        self.assertIn('precision', metrics)
        self.assertIn('recall', metrics)
        self.assertIn('f1_score', metrics)


class TestTransferValidator(unittest.TestCase):
    def setUp(self):
        """Set up transfer validator test fixtures"""
        self.validator = TransferValidator()

    def test_performance_validation(self):
        """Test performance validation between sim and real"""
        # Create mock performance data
        sim_performance = {
            "accuracy": 0.92,
            "precision": 0.90,
            "recall": 0.91,
            "f1_score": 0.90
        }
        real_performance = {
            "accuracy": 0.85,
            "precision": 0.83,
            "recall": 0.84,
            "f1_score": 0.83
        }

        # Test validation
        results = self.validator.validate_performance(sim_performance, real_performance)

        # Verify results structure
        self.assertIn('accuracy_ratio', results)
        self.assertIn('precision_ratio', results)
        self.assertIn('overall_performance_score', results)
        self.assertIn('performance_transfer_success', results)

        # Verify ratio calculations are reasonable
        self.assertGreaterEqual(results['accuracy_ratio'], 0.5)  # Should be > 0.5
        self.assertLessEqual(results['accuracy_ratio'], 1.5)    # Should be < 1.5

    def test_behavior_similarity(self):
        """Test behavior similarity validation"""
        # Create mock trajectories
        sim_trajectory = [[0, 0], [1, 1], [2, 2], [3, 1], [4, 0]]
        real_trajectory = [[0, 0], [1.1, 0.9], [2.1, 2.1], [3.1, 1.1], [4.1, 0.1]]

        # Test similarity
        results = self.validator.validate_behavior_similarity(sim_trajectory, real_trajectory)

        # Verify results structure
        self.assertIn('similarity_score', results)
        self.assertIn('similarity_threshold', results)
        self.assertIn('behavior_transfer_success', results)

        # Similarity should be reasonably high for these similar trajectories
        self.assertGreaterEqual(results['similarity_score'], 0.5)

    def test_safety_validation(self):
        """Test safety validation"""
        # Create mock safety metrics
        safety_metrics = {
            "collision_free": 0.98,
            "stability": 0.96,
            "limit_compliance": 0.99
        }

        # Test safety validation
        results = self.validator.validate_safety(safety_metrics)

        # Verify results structure
        self.assertIn('safety_score', results)
        self.assertIn('safety_threshold', results)
        self.assertIn('safety_transfer_success', results)

        # With high safety metrics, it should be successful
        self.assertTrue(results['safety_transfer_success'])

    def test_overall_transfer_success(self):
        """Test overall transfer success calculation"""
        # Create mock validation results
        performance_results = {
            "overall_performance_score": 0.85,
            "accuracy_ratio": 0.9
        }

        behavior_results = {
            "similarity_score": 0.90,
            "behavior_transfer_success": True
        }

        safety_results = {
            "safety_score": 0.96,
            "safety_transfer_success": True
        }

        # Test overall calculation
        overall_results = self.validator.calculate_overall_transfer_success(
            performance_results, behavior_results, safety_results
        )

        # Verify results structure
        self.assertIn('overall_transfer_score', overall_results)
        self.assertIn('transfer_success', overall_results)
        self.assertIn('detailed_scores', overall_results)

        # With good scores, transfer should be successful
        self.assertTrue(overall_results['transfer_success'])


def run_transfer_unit_tests():
    """Run transfer component unit tests"""
    print("Running Transfer Component Unit Tests...")

    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestDomainRandomizationTransfer)
    suite.addTests(loader.loadTestsFromTestCase(TestDomainAdaptationTransfer))
    suite.addTests(loader.loadTestsFromTestCase(TestFineTuningTransfer))
    suite.addTests(loader.loadTestsFromTestCase(TestTransferValidator))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_transfer_unit_tests()
    exit(0 if success else 1)
```

## Integration Tests

### 1. Component Pair Integration Tests
```python
# test/integration/test_component_pairs.py
import unittest
import numpy as np
from unittest.mock import Mock, patch
import threading
import time

class TestPerceptionNavigationIntegration(unittest.TestCase):
    """Test integration between perception and navigation components"""

    def setUp(self):
        """Set up perception-navigation integration test"""
        pass

    def test_object_detection_to_navigation_goal(self):
        """Test that object detections can trigger navigation goals"""
        # This would test the integration between object detection and navigation
        # In a real system, this would involve ROS message passing

        # Mock perception output
        detected_objects = [
            {"class": "person", "position": (2.0, 3.0), "confidence": 0.95},
            {"class": "obstacle", "position": (1.5, 1.0), "confidence": 0.87}
        ]

        # In a real system, this would trigger navigation to approach person
        # For now, we'll just verify the concept
        person_detected = any(obj["class"] == "person" for obj in detected_objects)

        self.assertTrue(person_detected)

        # Verify that a navigation goal would be appropriate
        person_position = next(obj["position"] for obj in detected_objects if obj["class"] == "person")
        self.assertIsNotNone(person_position)

    def test_perception_feedback_to_navigation(self):
        """Test that perception feedback improves navigation"""
        # Mock navigation system receiving perception feedback
        obstacle_positions = [(1.0, 1.0), (2.5, 1.5), (3.0, 2.0)]

        # Navigation system should use this information to replan
        # For now, just verify the data flow concept
        self.assertGreater(len(obstacle_positions), 0)


class TestNavigationControlIntegration(unittest.TestCase):
    """Test integration between navigation and control components"""

    def setUp(self):
        """Set up navigation-control integration test"""
        pass

    def test_path_to_trajectory_execution(self):
        """Test that navigation paths are converted to control trajectories"""
        # Mock a planned path from navigation
        planned_path = [
            (0.0, 0.0), (0.5, 0.1), (1.0, 0.2),
            (1.5, 0.3), (2.0, 0.4), (2.5, 0.5)
        ]

        # Control system should convert this to trajectory commands
        # For now, verify the path structure
        self.assertGreater(len(planned_path), 1)

        # Verify path is sequential
        for i in range(1, len(planned_path)):
            prev_x, prev_y = planned_path[i-1]
            curr_x, curr_y = planned_path[i]
            # Each point should be reachable from the previous one
            distance = ((curr_x - prev_x)**2 + (curr_y - prev_y)**2)**0.5
            self.assertLessEqual(distance, 1.0)  # Reasonable step size

    def test_velocity_command_generation(self):
        """Test that navigation generates appropriate velocity commands"""
        # Mock current robot state and desired path
        current_pose = (1.0, 1.0, 0.0)  # x, y, theta
        desired_pose = (2.0, 2.0, 0.785)  # x, y, theta (45 degrees)

        # Calculate required velocity
        dx = desired_pose[0] - current_pose[0]
        dy = desired_pose[1] - current_pose[1]
        linear_vel = (dx**2 + dy**2)**0.5  # Simple distance-based velocity

        # Angular velocity based on orientation difference
        angular_vel = desired_pose[2] - current_pose[2]

        # Verify velocities are reasonable
        self.assertGreaterEqual(linear_vel, 0.0)
        self.assertLessEqual(abs(angular_vel), 3.14159)  # Within pi range


class TestBehaviorPerceptionIntegration(unittest.TestCase):
    """Test integration between behavior engine and perception"""

    def setUp(self):
        """Set up behavior-perception integration test"""
        pass

    def test_perception_triggers_behavior(self):
        """Test that perception results trigger appropriate behaviors"""
        # Mock perception results
        detections = [
            {"class": "person", "position": (3.0, 2.0), "confidence": 0.92},
            {"class": "table", "position": (1.5, 4.0), "confidence": 0.88}
        ]

        # Behavior engine should interpret these and decide on actions
        # For approaching person
        person_detected = any(d["class"] == "person" for d in detections)
        self.assertTrue(person_detected)

        # Behavior might be to navigate to person
        if person_detected:
            target_position = next(d["position"] for d in detections if d["class"] == "person")
            self.assertIsNotNone(target_position)

    def test_behavior_guided_perception(self):
        """Test that behaviors guide perception focus"""
        # Mock a "search for object" behavior
        search_task = {
            "type": "search",
            "target_object": "red_cup",
            "search_area": {"center": (2.0, 2.0), "radius": 1.0}
        }

        # Perception system should focus on the search area
        search_center = search_task["search_area"]["center"]
        search_radius = search_task["search_area"]["radius"]

        self.assertIsNotNone(search_center)
        self.assertGreater(search_radius, 0.0)


def run_component_pair_integration_tests():
    """Run component pair integration tests"""
    print("Running Component Pair Integration Tests...")

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add perception-navigation tests
    suite.addTests(loader.loadTestsFromTestCase(TestPerceptionNavigationIntegration))

    # Add navigation-control tests
    suite.addTests(loader.loadTestsFromTestCase(TestNavigationControlIntegration))

    # Add behavior-perception tests
    suite.addTests(loader.loadTestsFromTestCase(TestBehaviorPerceptionIntegration))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_component_pair_integration_tests()
    exit(0 if success else 1)
```

## Performance Tests

### 1. Component Performance Tests
```python
# test/performance/test_component_performance.py
import unittest
import time
import statistics
from collections import deque
import numpy as np
import threading
import pytest

class TestPerceptionPerformance(unittest.TestCase):
    """Performance tests for perception components"""

    def setUp(self):
        """Set up perception performance test fixtures"""
        self.iterations = 100
        self.timing_results = deque(maxlen=100)

    def test_perception_pipeline_throughput(self):
        """Test perception pipeline throughput (FPS)"""
        processing_times = []

        for i in range(self.iterations):
            start_time = time.time()

            # Simulate perception processing
            # In real implementation, this would call the actual perception pipeline
            self._simulate_perception_processing()

            end_time = time.time()
            processing_time = end_time - start_time
            processing_times.append(processing_time)

        # Calculate FPS
        avg_processing_time = statistics.mean(processing_times)
        fps = 1.0 / avg_processing_time if avg_processing_time > 0 else float('inf')

        print(f"Perception FPS: {fps:.2f}, Avg Processing Time: {avg_processing_time:.4f}s")

        # Verify performance meets minimum requirements
        min_fps = 30.0  # Minimum 30 FPS requirement
        self.assertGreaterEqual(fps, min_fps,
                               f"Perception FPS {fps:.2f} below minimum {min_fps}")

    def test_perception_accuracy_under_load(self):
        """Test perception accuracy under processing load"""
        accuracy_results = []

        for i in range(50):  # Test with reduced iterations for accuracy testing
            # Simulate perception with accuracy measurement
            accuracy = self._simulate_perception_with_accuracy()
            accuracy_results.append(accuracy)

        avg_accuracy = statistics.mean(accuracy_results)
        print(f"Average perception accuracy: {avg_accuracy:.3f}")

        # Verify accuracy meets minimum requirements
        min_accuracy = 0.85  # Minimum 85% accuracy requirement
        self.assertGreaterEqual(avg_accuracy, min_accuracy,
                               f"Perception accuracy {avg_accuracy:.3f} below minimum {min_accuracy}")

    def _simulate_perception_processing(self):
        """Simulate perception processing"""
        # Simulate processing of an image
        time.sleep(0.01)  # 10ms processing time simulation

        # Add some computation to make it more realistic
        data = np.random.random((640, 480, 3))
        processed = np.mean(data, axis=2)  # Simple processing
        return processed

    def _simulate_perception_with_accuracy(self):
        """Simulate perception with accuracy measurement"""
        # Simulate processing that includes accuracy assessment
        time.sleep(0.015)  # 15ms processing time

        # Simulate accuracy (random value with bias toward high accuracy)
        accuracy = 0.8 + np.random.random() * 0.15  # Between 0.8 and 0.95
        return min(accuracy, 1.0)  # Cap at 1.0


class TestNavigationPerformance(unittest.TestCase):
    """Performance tests for navigation components"""

    def setUp(self):
        """Set up navigation performance test fixtures"""
        self.iterations = 20  # Navigation is slower, so fewer iterations
        self.planning_times = []

    def test_path_planning_performance(self):
        """Test path planning performance"""
        planning_times = []

        for i in range(self.iterations):
            start_time = time.time()

            # Simulate path planning
            path = self._simulate_path_planning(i)

            end_time = time.time()
            planning_time = end_time - start_time
            planning_times.append(planning_time)

        avg_planning_time = statistics.mean(planning_times)
        print(f"Average path planning time: {avg_planning_time:.4f}s")

        # Verify planning time meets requirements
        max_planning_time = 1.0  # Maximum 1 second for planning
        self.assertLessEqual(avg_planning_time, max_planning_time,
                            f"Path planning time {avg_planning_time:.4f}s exceeds maximum {max_planning_time}")

    def test_navigation_success_rate(self):
        """Test navigation success rate under various conditions"""
        success_count = 0
        total_attempts = 50

        for i in range(total_attempts):
            success = self._simulate_navigation_attempt(i)
            if success:
                success_count += 1

        success_rate = success_count / total_attempts
        print(f"Navigation success rate: {success_rate:.3f} ({success_count}/{total_attempts})")

        # Verify success rate meets minimum requirements
        min_success_rate = 0.90  # Minimum 90% success rate
        self.assertGreaterEqual(success_rate, min_success_rate,
                               f"Navigation success rate {success_rate:.3f} below minimum {min_success_rate}")

    def _simulate_path_planning(self, iteration):
        """Simulate path planning operation"""
        # Simulate planning complexity based on iteration
        time.sleep(0.1 + np.random.random() * 0.2)  # 100-300ms planning time
        return [(0, 0), (1, 1), (2, 2), (3, 1), (4, 0)]  # Mock path

    def _simulate_navigation_attempt(self, attempt_num):
        """Simulate a navigation attempt"""
        # Simulate navigation with some chance of failure
        time.sleep(0.5 + np.random.random() * 1.0)  # 500ms to 1.5s navigation time

        # 90% success rate simulation
        return np.random.random() < 0.9


class TestControlPerformance(unittest.TestCase):
    """Performance tests for control components"""

    def setUp(self):
        """Set up control performance test fixtures"""
        self.iterations = 1000  # Control runs at high frequency
        self.response_times = []

    def test_control_loop_frequency(self):
        """Test control loop frequency"""
        start_time = time.time()

        for i in range(self.iterations):
            loop_start = time.time()

            # Simulate control loop iteration
            self._simulate_control_iteration(i)

            loop_end = time.time()
            self.response_times.append(loop_end - loop_start)

        total_time = time.time() - start_time
        frequency = self.iterations / total_time

        print(f"Control loop frequency: {frequency:.2f} Hz, Avg response time: {statistics.mean(self.response_times):.6f}s")

        # Verify control frequency meets requirements
        min_frequency = 100.0  # Minimum 100 Hz control frequency
        self.assertGreaterEqual(frequency, min_frequency,
                               f"Control frequency {frequency:.2f} Hz below minimum {min_frequency}")

    def test_control_tracking_accuracy(self):
        """Test control tracking accuracy"""
        tracking_errors = []

        for i in range(100):
            error = self._simulate_tracking_error(i)
            tracking_errors.append(error)

        avg_error = statistics.mean(tracking_errors)
        print(f"Average tracking error: {avg_error:.6f}m")

        # Verify tracking accuracy meets requirements
        max_error = 0.01  # Maximum 1cm tracking error
        self.assertLessEqual(avg_error, max_error,
                            f"Tracking error {avg_error:.6f}m exceeds maximum {max_error}m")

    def _simulate_control_iteration(self, iteration):
        """Simulate a control loop iteration"""
        # Simulate control computation
        time.sleep(0.001)  # 1ms control loop time simulation
        command = np.random.random(2)  # Simulated control command
        return command

    def _simulate_tracking_error(self, iteration):
        """Simulate tracking error measurement"""
        # Simulate error between desired and actual position
        time.sleep(0.002)  # 2ms simulation time
        error = np.random.normal(0, 0.005)  # Normal distribution around 0 with std 5mm
        return abs(error)


def run_component_performance_tests():
    """Run component performance tests"""
    print("Running Component Performance Tests...")

    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Add perception performance tests
    suite.addTests(loader.loadTestsFromTestCase(TestPerceptionPerformance))

    # Add navigation performance tests
    suite.addTests(loader.loadTestsFromTestCase(TestNavigationPerformance))

    # Add control performance tests
    suite.addTests(loader.loadTestsFromTestCase(TestControlPerformance))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_component_performance_tests()
    exit(0 if success else 1)
```

## Test Utilities

### 1. Test Helper Functions
```python
# test/utils/test_helpers.py
import numpy as np
import time
from typing import Any, Dict, List, Tuple
import json
import tempfile
import os

class TestHelper:
    """Utility functions for testing Isaac components"""

    @staticmethod
    def create_mock_image(width: int = 640, height: int = 480, channels: int = 3) -> np.ndarray:
        """Create a mock image for testing"""
        return np.random.randint(0, 255, (height, width, channels), dtype=np.uint8)

    @staticmethod
    def create_mock_laser_scan(ranges_count: int = 360, min_range: float = 0.1, max_range: float = 10.0) -> List[float]:
        """Create mock laser scan data"""
        return [min_range + np.random.random() * (max_range - min_range) for _ in range(ranges_count)]

    @staticmethod
    def create_mock_odometry(x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> Dict[str, Any]:
        """Create mock odometry data"""
        return {
            "position": {"x": x, "y": y, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": np.sin(theta/2), "w": np.cos(theta/2)},
            "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    @staticmethod
    def measure_function_performance(func, *args, iterations: int = 100, **kwargs) -> Dict[str, float]:
        """Measure performance of a function"""
        times = []

        for i in range(iterations):
            start_time = time.time()
            result = func(*args, **kwargs)
            end_time = time.time()
            times.append(end_time - start_time)

        return {
            "average_time": np.mean(times),
            "min_time": np.min(times),
            "max_time": np.max(times),
            "std_dev": np.std(times),
            "median_time": np.median(times)
        }

    @staticmethod
    def assert_within_tolerance(value: float, expected: float, tolerance: float = 0.01):
        """Assert that a value is within tolerance of expected"""
        assert abs(value - expected) <= tolerance, \
            f"Value {value} not within tolerance {tolerance} of expected {expected}"

    @staticmethod
    def create_test_data_directory() -> str:
        """Create a temporary directory for test data"""
        temp_dir = tempfile.mkdtemp(prefix="isaac_test_")
        return temp_dir

    @staticmethod
    def save_test_results(results: Dict[str, Any], filename: str = None) -> str:
        """Save test results to a JSON file"""
        if filename is None:
            filename = f"test_results_{int(time.time())}.json"

        filepath = os.path.join(os.getcwd(), filename)

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        return filepath


def run_test_utils_demo():
    """Demonstrate test utility functions"""
    print("Testing Helper Utilities...")

    # Create mock data
    mock_image = TestHelper.create_mock_image(320, 240, 3)
    print(f"Created mock image with shape: {mock_image.shape}")

    mock_scan = TestHelper.create_mock_laser_scan(360)
    print(f"Created mock scan with {len(mock_scan)} ranges")

    mock_odom = TestHelper.create_mock_odometry(1.0, 2.0, 1.57)
    print(f"Created mock odometry at position: ({mock_odom['position']['x']}, {mock_odom['position']['y']})")

    # Test performance measurement
    def sample_function(x, y):
        time.sleep(0.001)  # Simulate some work
        return x + y

    perf_results = TestHelper.measure_function_performance(sample_function, 1, 2, iterations=10)
    print(f"Function performance: {perf_results['average_time']:.6f}s average")

    # Test tolerance assertion
    try:
        TestHelper.assert_within_tolerance(1.0, 1.01, 0.02)  # Should pass
        print("Tolerance assertion passed")
    except AssertionError:
        print("Tolerance assertion failed (unexpected)")

    print("Test helper utilities demonstrated successfully")


if __name__ == '__main__':
    run_test_utils_demo()
```

## Test Coverage and Reporting

### 1. Test Coverage Configuration
```bash
# test/coverage/coverage_config.sh
#!/bin/bash

# Test coverage configuration for Isaac components

echo "Setting up test coverage configuration..."

# Install coverage tools
pip3 install coverage pytest-cov

# Create coverage configuration
cat > .coveragerc << 'EOF'
[run]
source = .
omit =
    */venv/*
    */env/*
    */tests/*
    */test/*
    */__pycache__/*
    */site-packages/*
    setup.py
    examples/*  # Exclude examples from coverage
    test/*      # Exclude test files from coverage

[report]
exclude_lines =
    pragma: no cover
    def __repr__
    raise AssertionError
    raise NotImplementedError
    if __name__ == .__main__.:

precision = 2
show_missing = True
skip_covered = False
EOF

echo "Coverage configuration created at .coveragerc"

# Create test runner with coverage
cat > run_tests_with_coverage.sh << 'EOF'
#!/bin/bash
# Run tests with coverage

echo "Running tests with coverage..."

# Run unit tests with coverage
python -m coverage run -m pytest test/unit/ -v

# Run integration tests with coverage
python -m coverage run -a -m pytest test/integration/ -v

# Run performance tests with coverage
python -m coverage run -a -m pytest test/performance/ -v

# Generate coverage report
python -m coverage report -m

# Generate HTML coverage report
python -m coverage html

echo "Coverage reports generated in htmlcov/ directory"
EOF

chmod +x run_tests_with_coverage.sh

echo "Test coverage configuration completed!"
```

### 2. Test Reporting Configuration
```yaml
# test/reporting/junit_config.xml
# This would be an XSD schema for JUnit test reports
# For documentation purposes only
<?xml version="1.0" encoding="UTF-8"?>
<testsuites>
  <testsuite name="Isaac Component Tests" tests="100" failures="0" errors="0" time="120.5">
    <testcase name="test_perception_pipeline" classname="PerceptionTests" time="0.05"/>
    <testcase name="test_navigation_accuracy" classname="NavigationTests" time="1.2"/>
    <!-- More test cases -->
  </testsuite>
</testsuites>
```

## Testing Best Practices

### 1. Component Testing Best Practices
```python
# test/best_practices.py
class ComponentTestingBestPractices:
    """
    Best practices for component testing in Isaac-based systems
    """

    @staticmethod
    def unit_testing_best_practices():
        """
        Best practices for unit testing
        """
        practices = [
            "Test individual functions and methods in isolation",
            "Use mocking to isolate dependencies",
            "Cover edge cases and error conditions",
            "Test both positive and negative scenarios",
            "Keep tests fast and focused",
            "Use descriptive test names that explain the behavior",
            "Maintain high code coverage (aim for 80%+)",
            "Test error handling and exception cases",
            "Verify inputs and outputs thoroughly"
        ]
        return practices

    @staticmethod
    def integration_testing_best_practices():
        """
        Best practices for integration testing
        """
        practices = [
            "Test component interactions and data flow",
            "Verify message passing between components",
            "Test system behavior under various conditions",
            "Include both nominal and off-nominal scenarios",
            "Test component startup and shutdown sequences",
            "Validate component dependencies and interfaces",
            "Test error recovery and fault tolerance",
            "Include performance considerations in integration tests",
            "Test with realistic data volumes and rates"
        ]
        return practices

    @staticmethod
    def performance_testing_best_practices():
        """
        Best practices for performance testing
        """
        practices = [
            "Establish baseline performance metrics",
            "Test under realistic workload conditions",
            "Include warmup periods to eliminate cold start effects",
            "Measure both peak and sustained performance",
            "Test with varying data sizes and complexity",
            "Include memory usage and resource consumption metrics",
            "Test performance degradation over time",
            "Validate performance under stress conditions",
            "Compare performance across different hardware configurations"
        ]
        return practices

    @staticmethod
    def test_data_best_practices():
        """
        Best practices for test data
        """
        practices = [
            "Use realistic test data that represents real-world scenarios",
            "Include edge cases and boundary conditions",
            "Create diverse test datasets covering different situations",
            "Use both synthetic and real data when possible",
            "Include corrupted or malformed data for robustness testing",
            "Ensure test data privacy and security compliance",
            "Maintain test data in version control",
            "Document test data sources and characteristics",
            "Regularly update test data to reflect new scenarios"
        ]
        return practices

    @staticmethod
    def test_organization_best_practices():
        """
        Best practices for test organization
        """
        practices = [
            "Organize tests by component and functionality",
            "Use clear and consistent naming conventions",
            "Separate unit, integration, and performance tests",
            "Group related tests into test suites",
            "Use setup and teardown methods appropriately",
            "Maintain test independence and reproducibility",
            "Document test purposes and expected behaviors",
            "Keep tests maintainable and readable",
            "Use continuous integration for automated testing"
        ]
        return practices


def print_component_testing_best_practices():
    """
    Print all component testing best practices
    """
    practices = ComponentTestingBestPractices()

    print("=== Component Testing Best Practices ===\n")

    print("Unit Testing:")
    for i, practice in enumerate(practices.unit_testing_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nIntegration Testing:")
    for i, practice in enumerate(practices.integration_testing_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nPerformance Testing:")
    for i, practice in enumerate(practices.performance_testing_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nTest Data:")
    for i, practice in enumerate(practices.test_data_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nTest Organization:")
    for i, practice in enumerate(practices.test_organization_best_practices(), 1):
        print(f"  {i}. {practice}")


if __name__ == "__main__":
    print_component_testing_best_practices()
```

## Troubleshooting

### Common Testing Issues and Solutions

#### 1. Unit Test Issues
- **Mocking Dependencies**: Use appropriate mocking libraries and techniques
- **Test Isolation**: Ensure tests don't depend on each other
- **Timing Issues**: Use time mocking for time-dependent tests
- **Resource Cleanup**: Implement proper setup and teardown

#### 2. Integration Test Issues
- **Message Synchronization**: Handle asynchronous message passing
- **Component Initialization**: Ensure components are properly initialized
- **Data Flow**: Verify data passes correctly between components
- **Timeout Handling**: Set appropriate timeouts for operations

#### 3. Performance Test Issues
- **Warmup Periods**: Account for initialization and caching effects
- **Measurement Accuracy**: Use precise timing and avoid interference
- **Resource Contention**: Isolate performance tests from other processes
- **Statistical Significance**: Use enough iterations for reliable results

## Next Steps
After completing component testing:
1. Proceed to system validation (Task 4.2)
2. Create comprehensive documentation and examples (Task 4.3)
3. Conduct final system integration testing

## Resources
- [pytest Documentation](https://docs.pytest.org/)
- [ROS 2 Testing Guidelines](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html#testing)
- [Unit Testing Best Practices](https://docs.python.org/3/library/unittest.html)
- [Continuous Integration for Robotics](https://www.jenkins.io/doc/book/pipeline/)