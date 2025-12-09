# Isaac ROS Package Installation Guide

## Overview
This guide provides instructions for installing and configuring Isaac ROS packages for the AI-Robot Brain module. Isaac ROS provides hardware-accelerated perception and navigation capabilities using NVIDIA GPUs.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 20.04 LTS or 22.04 LTS
- **ROS Distribution**: ROS 2 Humble Hawksbill (recommended)
- **Hardware**: NVIDIA GPU with compute capability 6.0 or higher
- **Container Runtime**: Docker with nvidia-docker2
- **Memory**: 16GB+ RAM recommended

### Software Dependencies
- NVIDIA GPU drivers (latest recommended)
- CUDA 11.8 or later
- Docker and nvidia-docker2
- ROS 2 Humble installation

## Installation Methods

### Method 1: Docker Container (Recommended)
```bash
# Pull the Isaac ROS Docker image
docker pull nvcr.io/nvidia/isaac-ros:latest

# Run the container with GPU access
docker run --gpus all -it --rm \
  --network host \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --name="isaac-ros-container" \
  nvcr.io/nvidia/isaac-ros:latest

# Inside the container, verify Isaac ROS packages
ros2 pkg list | grep isaac_ros
```

### Method 2: Native Installation (Ubuntu 20.04/22.04)
```bash
# Add NVIDIA ROS2 repository
curl -sSL https://repos.ros.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64] http://repo.ron.cal.rwth-aachen.de/repos/nvidia-jetson focal main" > /etc/apt/sources.list.d/nvidia-jetson.list'

# Update package lists
sudo apt update

# Install Isaac ROS common packages
sudo apt install -y ros-humble-isaac-ros-common

# Install Isaac ROS perception packages
sudo apt install -y ros-humble-isaac-ros-perception

# Install Isaac ROS navigation packages
sudo apt install -y ros-humble-isaac-ros-navigation

# Install Isaac ROS manipulation packages (if needed)
sudo apt install -y ros-humble-isaac-ros-manipulation
```

### Method 3: Build from Source
```bash
# Create ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Clone Isaac ROS repositories
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git src/isaac_ros_visual_slam
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git src/isaac_ros_apriltag
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_centerpose.git src/isaac_ros_centerpose
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git src/isaac_ros_dnn_inference
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git src/isaac_ros_image_pipeline
git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pointcloud_utils.git src/isaac_ros_pointcloud_utils

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_visual_slam \
  isaac_ros_apriltag \
  isaac_ros_centerpose \
  isaac_ros_dnn_inference \
  isaac_ros_image_pipeline \
  isaac_ros_pointcloud_utils
```

## Configuration Files

### 1. Isaac ROS Common Configuration
```yaml
# config/isaac_ros_common.yaml
isaac_ros_common:
  version: "3.2.0"
  gpu_id: 0  # GPU to use for acceleration
  enable_performance_metrics: true
  performance_report_interval: 1.0  # seconds
  memory_pool_size: 1073741824  # 1GB in bytes
  cuda_device: 0
  cuda_context_mode: "exclusive_thread"
  logging:
    level: "INFO"
    format: "default"
    enable_console: true
    enable_file: false
```

### 2. Isaac ROS Perception Pipeline Configuration
```yaml
# config/isaac_ros_perception.yaml
isaac_ros_perception:
  pipelines:
    object_detection:
      enabled: true
      model: "yolov5n"
      input_topic: "/camera/color/image_raw"
      output_topic: "/detections"
      confidence_threshold: 0.5
      max_objects: 10
      image_width: 640
      image_height: 480
    semantic_segmentation:
      enabled: true
      model: "unet"
      input_topic: "/camera/color/image_raw"
      output_topic: "/segmentation"
      confidence_threshold: 0.7
    depth_estimation:
      enabled: true
      input_topic: "/camera/depth/image_rect_raw"
      output_topic: "/depth_processed"
      min_depth: 0.1
      max_depth: 10.0
    pose_estimation:
      enabled: true
      detector: "apriltag"
      input_topic: "/camera/color/image_raw"
      output_topic: "/pose_estimation"
      tag_family: "tag36h11"
      tag_size: 0.166
```

### 3. Isaac ROS Navigation Configuration
```yaml
# config/isaac_ros_navigation.yaml
isaac_ros_navigation:
  global_planner:
    planner_plugin: "nav2_navfn_planner/NavfnPlanner"
    interpolation_resolution: 0.1
    use_astar: false
    allow_unknown: true
  local_planner:
    planner_plugin: "nav2_teb_local_planner/TebLocalPlanner"
    max_vel_x: 0.5
    max_vel_theta: 1.0
    min_vel_x: 0.05
    min_vel_theta: 0.1
    acc_lim_x: 2.5
    acc_lim_theta: 3.2
  controller:
    type: "teb"
    frequency: 10.0
    min_samples: 3
    max_samples: 500
  costmap:
    global:
      resolution: 0.05
      robot_radius: 0.3
      inflation_radius: 0.55
      observation_sources: "scan"
      scan: {sensor_frame: "base_scan", topic: "/scan", data_type: "LaserScan", marking: true, clearing: true}
    local:
      resolution: 0.025
      robot_radius: 0.3
      inflation_radius: 0.15
      observation_sources: "scan"
      scan: {sensor_frame: "base_scan", topic: "/scan", data_type: "LaserScan", marking: true, clearing: true}
```

## Verification Steps

### 1. Verify Package Installation
```bash
# List Isaac ROS packages
ros2 pkg list | grep isaac_ros

# Check specific packages
ros2 pkg list | grep isaac_ros_common
ros2 pkg list | grep isaac_ros_perception
ros2 pkg list | grep isaac_ros_navigation
```

### 2. Test Isaac ROS Nodes
```bash
# Terminal 1: Launch a simple Isaac ROS node
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py

# Terminal 2: Publish test image
ros2 topic pub /image raw sensor_msgs/msg/Image "{}"
```

### 3. Performance Verification
```bash
# Check GPU utilization during Isaac ROS operations
nvidia-smi

# Monitor Isaac ROS node performance
ros2 run isaac_ros_apriltag isaac_ros_apriltag --ros-args --log-level info
```

## Hardware Acceleration Setup

### 1. CUDA Configuration
```bash
# Verify CUDA installation
nvcc --version
nvidia-smi

# Test CUDA functionality
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

### 2. Isaac ROS Performance Settings
```yaml
# config/isaac_ros_performance.yaml
performance:
  gpu_settings:
    cuda_device: 0
    memory_pool_size: 2147483648  # 2GB
    tensorrt_cache: "/tmp/tensorrt_cache"
    enable_tensorrt: true
    tensorrt_precision: "fp16"  # fp32, fp16, int8
  pipeline_settings:
    max_batch_size: 1
    input_queue_size: 5
    output_queue_size: 5
    enable_async: true
  optimization:
    enable_caching: true
    enable_fusion: true
    enable_layer_clustering: true
```

## Troubleshooting

### Common Issues

#### 1. CUDA/GPU Issues
```bash
# Check if GPU is accessible
nvidia-smi

# Verify CUDA installation
nvidia-ml-py3 --version

# Check if Isaac ROS can access GPU
# Look for CUDA errors in logs
```

#### 2. Docker Runtime Issues
```bash
# Verify nvidia-docker2 installation
docker run --rm --gpus all nvidia/cuda:11.8-base-ubuntu20.04 nvidia-smi

# Check if nvidia-container-runtime is properly configured
sudo nvidia-ctk runtime configure --runtime=docker
```

#### 3. Package Dependency Issues
```bash
# Reinstall dependencies
sudo apt update
sudo apt install -f
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. Permission Issues
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in, or run:
newgrp docker
```

## Testing Isaac ROS Functionality

### 1. Basic Node Test
```python
# test/isaac_ros_basic_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class IsaacROSBasicTest(Node):
    def __init__(self):
        super().__init__('isaac_ros_basic_test')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.subscription = self.create_subscription(
            Image,
            'input_image',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Isaac ROS: {self.i}'
        self.publisher_.publish(msg)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image with dimensions: {msg.height}x{msg.width}')

def main(args=None):
    rclpy.init(args=args)
    isaac_ros_basic_test = IsaacROSBasicTest()
    rclpy.spin(isaac_ros_basic_test)
    isaac_ros_basic_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Isaac ROS Perception Test
```python
# test/isaac_ros_perception_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class IsaacROSPerceptionTest(Node):
    def __init__(self):
        super().__init__('isaac_ros_perception_test')

        # Subscribe to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)

        # Subscribe to detections
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10)

        self.get_logger().info('Isaac ROS Perception Test Node Started')

    def image_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.height}x{msg.width}')

    def detection_callback(self, msg):
        self.get_logger().info(f'Received {len(msg.detections)} detections')

def main(args=None):
    rclpy.init(args=args)
    perception_test = IsaacROSPerceptionTest()
    rclpy.spin(perception_test)
    perception_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Isaac Sim

### 1. ROS Bridge Configuration
```yaml
# config/ros_bridge_config.yaml
ros_bridge:
  enabled: true
  bridge_type: "python"
  topics:
    - name: "camera/color/image_raw"
      type: "sensor_msgs/Image"
      direction: "pub"
      frequency: 30.0
    - name: "camera/depth/image_rect_raw"
      type: "sensor_msgs/Image"
      direction: "pub"
      frequency: 30.0
    - name: "imu/data"
      type: "sensor_msgs/Imu"
      direction: "pub"
      frequency: 100.0
    - name: "joint_states"
      type: "sensor_msgs/JointState"
      direction: "pub"
      frequency: 50.0
    - name: "cmd_vel"
      type: "geometry_msgs/Twist"
      direction: "sub"
      frequency: 50.0
    - name: "joint_group_position_controller/commands"
      type: "std_msgs/Float64MultiArray"
      direction: "sub"
      frequency: 100.0
```

### 2. Isaac Sim to Isaac ROS Connection
```python
# config/isaac_sim_ros_bridge.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.ros_bridge.scripts import example_stand_alone
import rclpy

def setup_isaac_sim_ros_bridge():
    """
    Setup the bridge between Isaac Sim and ROS 2
    """
    # Initialize ROS context
    rclpy.init()

    # Create Isaac Sim world
    world = World(stage_units_in_meters=1.0)

    # Add robot to the world
    # Configure sensors
    # Setup ROS bridge

    print("Isaac Sim to ROS bridge configured successfully")

def run_isaac_ros_integration():
    """
    Run integration test between Isaac Sim and Isaac ROS
    """
    # Launch Isaac Sim with robot
    # Start Isaac ROS perception nodes
    # Verify data flow between systems
    pass
```

## Performance Benchmarks

### Expected Performance
- **Object Detection**: <10ms inference time on RTX 3080
- **Pose Estimation**: Real-time 6DOF pose estimation
- **Semantic Segmentation**: <20ms processing time
- **Sensor Processing**: Real-time processing for multiple sensors

### Benchmarking Script
```bash
# benchmark/isaac_ros_benchmark.py
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

class IsaacROSPerformanceBenchmark(Node):
    def __init__(self):
        super().__init__('isaac_ros_performance_benchmark')
        self.subscription = self.create_subscription(
            Image,
            '/input_image',
            self.benchmark_callback,
            10)
        self.latencies = []
        self.frame_count = 0

    def benchmark_callback(self, msg):
        # Calculate processing latency
        current_time = self.get_clock().now().nanoseconds
        msg_time = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        latency = (current_time - msg_time) / 1e6  # Convert to ms

        self.latencies.append(latency)
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            avg_latency = sum(self.latencies) / len(self.latencies)
            self.get_logger().info(f'Average latency: {avg_latency:.2f}ms over {self.frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    benchmark = IsaacROSPerformanceBenchmark()
    rclpy.spin(benchmark)
    benchmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps
After successful Isaac ROS installation:
1. Test perception pipeline integration (Task 2.3)
2. Configure navigation system (Task 2.4)
3. Verify hardware acceleration is working properly
4. Proceed to advanced features (Phase 3)

## Resources
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html)
- [Isaac ROS GitHub Repository](https://github.com/NVIDIA-ISAAC-ROS)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)