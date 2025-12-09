# Isaac ROS Perception Pipeline Guide

## Overview
This guide provides instructions for implementing hardware-accelerated perception using Isaac ROS. The perception pipeline will include camera processing, object detection, pose estimation, and depth estimation capabilities for the humanoid robot.

## Isaac ROS Perception Components

### 1. Key Isaac ROS Packages
- **isaac_ros_apriltag**: Marker detection and pose estimation
- **isaac_ros_centerpose**: Multi-object detection and pose estimation
- **isaac_ros_dnn_inference**: Deep neural network inference acceleration
- **isaac_ros_image_pipeline**: Image processing and calibration
- **isaac_ros_visual_slam**: Visual SLAM capabilities
- **isaac_ros_pointcloud_utils**: Point cloud processing utilities

### 2. Performance Benefits
- **GPU Acceleration**: 10-100x speedup for vision algorithms
- **Tensor Cores**: AI inference acceleration on modern GPUs
- **CUDA Optimization**: Direct GPU memory access and processing
- **Real-time Processing**: 30+ FPS for optimized pipelines

## Perception Pipeline Architecture

### 1. Basic Perception Stack
```yaml
# config/perception_stack.yaml
perception_stack:
  version: "3.2.0"
  nodes:
    image_processing:
      package: "isaac_ros_image_pipeline"
      node_name: "image_proc"
      input_topic: "/camera/color/image_raw"
      output_topic: "/camera/color/image_rect_color"
      parameters:
        use_sensor_data_qos: true
        queue_size: 1
        debayer_edge_algorithm: "EdgeAware"
        debayer_blue_red_bilateral_threshold: 50
        interpolation: 1
    apriltag_detector:
      package: "isaac_ros_apriltag"
      node_name: "apriltag"
      input_topic: "/camera/color/image_rect_color"
      output_topic: "/detections"
      parameters:
        max_tags: 64
        tag_family: "tag36h11"
        tag_size: 0.166
        max_hamming_dist: 0
        quad_decimate: 2.0
        quad_sigma: 0.0
        refine_edges: 1
        decode_sharpening: 0.25
        debug: false
    centerpose:
      package: "isaac_ros_centerpose"
      node_name: "centerpose"
      input_topic: "/camera/color/image_rect_color"
      output_topic: "/centerpose_detections"
      parameters:
        model_path: "/usr/src/app/models/centerpose_resnet50.plan"
        input_layer_width: 512
        input_layer_height: 512
        input_layer_name: "input"
        output_layer_name: "output"
        confidence_threshold: 0.5
        mask_threshold: 0.5
        max_objects: 10
        enable_bbox: true
        enable_mask: true
        enable_pose: true
    visual_slam:
      package: "isaac_ros_visual_slam"
      node_name: "visual_slam"
      input_topics:
        - "/camera/color/image_rect_color"
        - "/camera/depth/image_rect_raw"
        - "/imu/data"
      output_topics:
        - "/visual_slam/trajectory"
        - "/visual_slam/map"
      parameters:
        enable_occupancy_map: true
        enable_localization_n_mapping: true
        enable_slam_visualization: true
        enable_rectification: true
        rectified_frame_id: "camera_color_optical_frame"
        base_frame_id: "base_link"
        odom_frame_id: "odom"
        map_frame_id: "map"
```

### 2. Perception Pipeline Launch Configuration
```xml
<!-- launch/perception_pipeline.launch.xml -->
<launch>
  <!-- Arguments -->
  <arg name="camera_namespace" default="camera"/>
  <arg name="apriltag" default="true"/>
  <arg name="centerpose" default="true"/>
  <arg name="visual_slam" default="true"/>
  <arg name="image_proc" default="true"/>

  <!-- Image Processing Node -->
  <node pkg="isaac_ros_image_proc" exec="isaac_ros_image_proc" name="image_proc" if="$(var image_proc)">
    <param name="use_sensor_data_qos" value="true"/>
    <param name="queue_size" value="1"/>
    <remap from="image_raw" to="$(var camera_namespace)/color/image_raw"/>
    <remap from="camera_info" to="$(var camera_namespace)/color/camera_info"/>
    <remap from="image_rect" to="$(var camera_namespace)/color/image_rect_color"/>
  </node>

  <!-- AprilTag Detector -->
  <node pkg="isaac_ros_apriltag" exec="apriltag_node" name="apriltag" if="$(var apriltag)">
    <param name="max_tags" value="64"/>
    <param name="tag_family" value="tag36h11"/>
    <param name="tag_size" value="0.166"/>
    <remap from="image" to="$(var camera_namespace)/color/image_rect_color"/>
    <remap from="detections" to="apriltag_detections"/>
  </node>

  <!-- CenterPose Detector -->
  <node pkg="isaac_ros_centerpose" exec="centerpose_node" name="centerpose" if="$(var centerpose)">
    <param name="model_path" value="$(find-pkg-share isaac_ros_centerpose)/models/centerpose_resnet50.plan"/>
    <param name="input_layer_width" value="512"/>
    <param name="input_layer_height" value="512"/>
    <param name="confidence_threshold" value="0.5"/>
    <remap from="image" to="$(var camera_namespace)/color/image_rect_color"/>
    <remap from="detections" to="centerpose_detections"/>
  </node>

  <!-- Visual SLAM -->
  <node pkg="isaac_ros_visual_slam" exec="visual_slam_node" name="visual_slam" if="$(var visual_slam)">
    <param name="enable_occupancy_map" value="true"/>
    <param name="enable_localization_n_mapping" value="true"/>
    <remap from="camera0/image" to="$(var camera_namespace)/color/image_rect_color"/>
    <remap from="camera0/camera_info" to="$(var camera_namespace)/color/camera_info"/>
    <remap from="depth/image" to="$(var camera_namespace)/depth/image_rect_raw"/>
    <remap from="depth/camera_info" to="$(var camera_namespace)/depth/camera_info"/>
    <remap from="imu" to="/imu/data"/>
    <remap from="visual_slam/trajectory" to="trajectory"/>
    <remap from="visual_slam/map" to="map"/>
  </node>
</launch>
```

## Implementation Examples

### 1. Camera Processing Pipeline
```python
# perception/camera_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessingPipeline(Node):
    def __init__(self):
        super().__init__('camera_processing_pipeline')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.processed_image_pub = self.create_publisher(
            Image,
            '/camera/color/image_processed',
            10
        )

        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info('Camera Processing Pipeline initialized')

    def camera_info_callback(self, msg):
        """Callback for camera info to get calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming camera image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply camera calibration if available
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                cv_image = cv2.undistort(
                    cv_image,
                    self.camera_matrix,
                    self.dist_coeffs
                )

            # Apply additional processing (resize, normalize, etc.)
            processed_image = self.process_image(cv_image)

            # Convert back to ROS image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header

            # Publish processed image
            self.processed_image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, image):
        """Apply image processing operations"""
        # Resize if needed
        height, width = image.shape[:2]
        if height > 720 or width > 1280:
            scale = min(720/height, 1280/width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            image = cv2.resize(image, (new_width, new_height))

        # Apply any additional processing
        # (denoising, enhancement, etc.)

        return image

def main(args=None):
    rclpy.init(args=args)
    camera_pipeline = CameraProcessingPipeline()
    rclpy.spin(camera_pipeline)
    camera_pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Object Detection Node
```python
# perception/object_detection.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

class IsaacROSObjectDetection(Node):
    def __init__(self):
        super().__init__('isaac_ros_object_detection')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscriber and publisher
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_color',
            self.detection_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/object_detections',
            10
        )

        # This would typically interface with Isaac ROS nodes
        # For this example, we'll simulate the detection process
        self.get_logger().info('Isaac ROS Object Detection initialized')

    def detection_callback(self, msg):
        """Process image and publish detections"""
        try:
            # In a real implementation, this would connect to Isaac ROS
            # detection nodes and process the results
            detections = self.simulate_detections(msg)

            # Publish detections
            self.detection_pub.publish(detections)

        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')

    def simulate_detections(self, image_msg):
        """Simulate object detections (in real implementation, this connects to Isaac ROS)"""
        # Create detection array message
        detection_array = Detection2DArray()
        detection_array.header = image_msg.header

        # Simulate some detections (in real implementation, these come from Isaac ROS nodes)
        # For example, detections from isaac_ros_centerpose or other perception nodes
        simulated_detections = [
            self.create_detection("humanoid_robot", 0.95, [100, 100, 200, 200]),
            self.create_detection("obstacle", 0.87, [300, 200, 150, 150]),
            self.create_detection("furniture", 0.78, [50, 300, 100, 120])
        ]

        detection_array.detections = simulated_detections

        return detection_array

    def create_detection(self, class_name, confidence, bbox):
        """Create a Detection2D message"""
        detection = Detection2D()

        # Set bounding box
        detection.bbox.center.x = bbox[0] + bbox[2] / 2  # center x
        detection.bbox.center.y = bbox[1] + bbox[3] / 2  # center y
        detection.bbox.size_x = bbox[2]  # width
        detection.bbox.size_y = bbox[3]  # height

        # Set hypothesis
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_name
        hypothesis.hypothesis.score = confidence

        detection.results.append(hypothesis)

        return detection

def main(args=None):
    rclpy.init(args=args)
    detector = IsaacROSObjectDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Pose Estimation Pipeline
```python
# perception/pose_estimation.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PoseArray
from vision_msgs.msg import Detection2DArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from builtin_interfaces.msg import Time
import numpy as np

class IsaacROSPoseEstimation(Node):
    def __init__(self):
        super().__init__('isaac_ros_pose_estimation')

        # Create subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/apriltag_detections',
            self.detection_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseArray,
            '/estimated_poses',
            10
        )

        # TF broadcaster for pose transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot pose publisher
        self.robot_pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            10
        )

        self.get_logger().info('Isaac ROS Pose Estimation initialized')

    def detection_callback(self, msg):
        """Process detection messages and estimate poses"""
        try:
            # Process detections to extract poses
            poses = self.process_detections_for_poses(msg)

            # Publish pose array
            pose_array_msg = PoseArray()
            pose_array_msg.header = msg.header
            pose_array_msg.poses = poses

            self.pose_pub.publish(pose_array_msg)

            # Broadcast transforms
            self.broadcast_poses_as_transforms(msg.header, poses)

        except Exception as e:
            self.get_logger().error(f'Error processing detections: {e}')

    def process_detections_for_poses(self, detection_array):
        """Process detections to estimate 3D poses"""
        poses = []

        for detection in detection_array.detections:
            # In a real implementation, this would use Isaac ROS
            # pose estimation nodes to compute 3D poses from 2D detections
            pose = self.estimate_pose_from_detection(detection)
            if pose is not None:
                poses.append(pose)

        return poses

    def estimate_pose_from_detection(self, detection):
        """Estimate 3D pose from 2D detection (simplified example)"""
        # This is a simplified example
        # In real implementation, this would use Isaac ROS pose estimation nodes
        # like apriltag for fiducial markers or centerpose for general objects

        pose = PoseStamped()
        pose.header.frame_id = "camera_color_optical_frame"

        # Estimate position based on bounding box center and depth
        # (In real implementation, this would come from depth data)
        pose.pose.position.x = detection.bbox.center.x
        pose.pose.position.y = detection.bbox.center.y
        pose.pose.position.z = 1.0  # Default depth estimate

        # Default orientation (facing forward)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        return pose.pose

    def broadcast_poses_as_transforms(self, header, poses):
        """Broadcast estimated poses as TF transforms"""
        for i, pose in enumerate(poses):
            t = TransformStamped()

            t.header.stamp = header.stamp
            t.header.frame_id = "camera_color_optical_frame"
            t.child_frame_id = f"object_{i}_frame"

            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z

            t.transform.rotation = pose.orientation

            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    pose_estimator = IsaacROSPoseEstimation()
    rclpy.spin(pose_estimator)
    pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### 1. TensorRT Configuration for Perception
```yaml
# config/tensorrt_config.yaml
tensorrt:
  enabled: true
  precision: "fp16"  # fp32, fp16, int8
  max_batch_size: 1
  dynamic_shapes:
    enabled: true
    min_shape: [1, 3, 224, 224]
    opt_shape: [1, 3, 512, 512]
    max_shape: [1, 3, 1024, 1024]
  workspace_size: 1073741824  # 1GB in bytes
  timing_cache: true
  persistent_cache: true
  cache_file: "/tmp/tensorrt_cache"
  plugins:
    enabled: true
    paths:
      - "/usr/lib/x86_64-linux-gnu/libnvinfer_plugin.so"
```

### 2. Isaac ROS Performance Monitoring
```python
# perception/performance_monitor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
import time
from collections import deque
import statistics

class PerceptionPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('perception_performance_monitor')

        # Performance tracking
        self.processing_times = deque(maxlen=100)
        self.fps_history = deque(maxlen=100)

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.performance_callback,
            1
        )

        self.fps_pub = self.create_publisher(Float32, '/perception_fps', 10)
        self.latency_pub = self.create_publisher(Float32, '/perception_latency', 10)

        # Timer for FPS calculation
        self.frame_count = 0
        self.last_time = time.time()

        self.get_logger().info('Perception Performance Monitor initialized')

    def performance_callback(self, msg):
        """Monitor perception pipeline performance"""
        start_time = time.time()

        # Simulate processing time (in real implementation, this would process the image)
        time.sleep(0.03)  # Simulate 30ms processing time

        end_time = time.time()
        processing_time = end_time - start_time

        # Track processing time
        self.processing_times.append(processing_time)

        # Calculate FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_time >= 1.0:  # Every second
            fps = self.frame_count / (current_time - self.last_time)
            self.fps_history.append(fps)

            # Publish FPS
            fps_msg = Float32()
            fps_msg.data = fps
            self.fps_pub.publish(fps_msg)

            # Publish latency
            latency_msg = Float32()
            latency_msg.data = processing_time * 1000  # Convert to milliseconds
            self.latency_pub.publish(latency_msg)

            self.get_logger().info(f'FPS: {fps:.2f}, Latency: {processing_time*1000:.2f}ms')

            self.frame_count = 0
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    monitor = PerceptionPerformanceMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Isaac Sim

### 1. Isaac Sim Perception Configuration
```python
# perception/isaac_sim_integration.py
import omni
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import carb
import numpy as np

def setup_isaac_sim_perception_sensors():
    """
    Setup perception sensors in Isaac Sim
    """
    stage = omni.usd.get_context().get_stage()

    # Create RGB camera
    camera_path = "/World/Camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.GetFocalLengthAttr().Set(24.0)
    camera.GetHorizontalApertureAttr().Set(20.955)
    camera.GetVerticalApertureAttr().Set(15.2908)
    camera.GetClippingRangeAttr().Set((0.1, 1000.0))

    # Create depth camera
    depth_camera_path = "/World/DepthCamera"
    depth_camera = UsdGeom.Camera.Define(stage, depth_camera_path)
    # Configure for depth data

    # Create LIDAR sensor (if needed)
    lidar_path = "/World/Lidar"
    # Configure LIDAR sensor

    print("Isaac Sim perception sensors configured")

def connect_perception_to_ros_bridge():
    """
    Connect Isaac Sim perception outputs to ROS bridge
    """
    # Configure ROS bridge topics for perception data
    ros_topics = [
        {
            "name": "/camera/color/image_raw",
            "type": "sensor_msgs/Image",
            "path": "/World/Camera",
            "frequency": 30.0
        },
        {
            "name": "/camera/depth/image_raw",
            "type": "sensor_msgs/Image",
            "path": "/World/DepthCamera",
            "frequency": 30.0
        },
        {
            "name": "/camera/color/camera_info",
            "type": "sensor_msgs/CameraInfo",
            "path": "/World/Camera",
            "frequency": 1.0
        }
    ]

    # This would typically be configured in the Isaac Sim ROS bridge extension
    print("ROS bridge configured for perception topics")

def generate_ground_truth_data():
    """
    Generate ground truth annotations for synthetic data
    """
    # This function would generate accurate ground truth
    # for training data from the simulation
    ground_truth = {
        "objects": [],
        "poses": [],
        "segmentation": [],
        "depth": []
    }

    # In Isaac Sim, we can access exact object poses and generate
    # perfect annotations for training
    print("Ground truth data generated")

    return ground_truth
```

## Testing and Validation

### 1. Perception Pipeline Test
```python
# test/perception_pipeline_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import time

class PerceptionPipelineTest(Node):
    def __init__(self):
        super().__init__('perception_pipeline_test')

        # Publishers
        self.test_image_pub = self.create_publisher(Image, '/test_image', 10)

        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/object_detections',
            self.detection_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/perception_test_status', 10)

        # Test timer
        self.test_timer = self.create_timer(5.0, self.run_test)
        self.test_counter = 0

        self.get_logger().info('Perception Pipeline Test Node initialized')

    def run_test(self):
        """Run periodic perception tests"""
        self.test_counter += 1

        if self.test_counter <= 5:
            # Publish test image
            test_msg = String()
            test_msg.data = f'Perception test run {self.test_counter}'
            self.status_pub.publish(test_msg)

            self.get_logger().info(f'Running perception test {self.test_counter}')
        else:
            # Stop testing
            self.test_timer.cancel()
            self.get_logger().info('Perception pipeline testing completed')

    def detection_callback(self, msg):
        """Handle detection results"""
        detection_count = len(msg.detections)
        self.get_logger().info(f'Received {detection_count} detections')

        # Validate detections
        for i, detection in enumerate(msg.detections):
            if detection.bbox.size_x > 0 and detection.bbox.size_y > 0:
                self.get_logger().info(f'Detection {i}: Valid bounding box')
            else:
                self.get_logger().warn(f'Detection {i}: Invalid bounding box')

def main(args=None):
    rclpy.init(args=args)
    test_node = PerceptionPipelineTest()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Performance Benchmarking
```bash
# benchmark/perception_benchmark.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import time
from collections import deque
import statistics

class PerceptionBenchmark(Node):
    def __init__(self):
        super().__init__('perception_benchmark')

        # Performance tracking
        self.latencies = deque(maxlen=1000)
        self.frame_count = 0
        self.start_time = time.time()

        # Subscribe to processed images
        self.subscriber = self.create_subscription(
            Image,
            '/camera/color/image_rect_color',
            self.benchmark_callback,
            10
        )

        # Publisher for results
        self.results_pub = self.create_publisher(Float32, '/benchmark_results', 10)

        # Timer to report results periodically
        self.report_timer = self.create_timer(10.0, self.report_results)

        self.get_logger().info('Perception Benchmark initialized')

    def benchmark_callback(self, msg):
        """Benchmark perception pipeline performance"""
        # Calculate processing latency
        current_time = self.get_clock().now().nanoseconds
        msg_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        latency = (current_time - msg_time) / 1e6  # Convert to ms

        self.latencies.append(latency)
        self.frame_count += 1

    def report_results(self):
        """Report benchmark results"""
        if len(self.latencies) > 0:
            avg_latency = statistics.mean(self.latencies)
            min_latency = min(self.latencies)
            max_latency = max(self.latencies)

            elapsed_time = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0

            self.get_logger().info(
                f'Benchmark Results:\n'
                f'  Average Latency: {avg_latency:.2f}ms\n'
                f'  Min Latency: {min_latency:.2f}ms\n'
                f'  Max Latency: {max_latency:.2f}ms\n'
                f'  Average FPS: {avg_fps:.2f}\n'
                f'  Total Frames: {self.frame_count}'
            )

def main(args=None):
    rclpy.init(args=args)
    benchmark = PerceptionBenchmark()
    rclpy.spin(benchmark)
    benchmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### Common Issues

#### 1. Performance Issues
- **Low FPS**: Check GPU utilization, reduce input resolution, optimize model
- **High Latency**: Verify GPU memory, check for bottlenecks in pipeline
- **Memory Errors**: Monitor GPU memory usage, adjust batch sizes

#### 2. Detection Issues
- **Poor Accuracy**: Verify lighting conditions, adjust confidence thresholds
- **False Positives**: Tune detection parameters, improve training data
- **Missing Detections**: Check input image quality, adjust model settings

#### 3. Integration Issues
- **ROS Bridge Problems**: Verify topic names, check QoS settings
- **TF Issues**: Ensure proper frame relationships, check transform timing
- **Synchronization**: Verify timestamp alignment between sensors

## Next Steps
After implementing the perception pipeline:
1. Configure navigation system with Nav2 (Task 2.4)
2. Test perception in various simulated environments
3. Validate performance benchmarks
4. Implement sim-to-real transfer techniques (Phase 3)

## Resources
- [Isaac ROS Perception Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_perception/index.html)
- [Isaac ROS AprilTag Detection](https://nvidia-isaac-ros.github.io/packages/isaac_ros_apriltag/index.html)
- [Isaac ROS CenterPose](https://nvidia-isaac-ros.github.io/packages/isaac_ros_centerpose/index.html)
- [ROS 2 Image Transport](https://docs.ros.org/en/humble/p/image_transport/)