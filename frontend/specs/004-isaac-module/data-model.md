# Module 3: AI-Robot Brain (NVIDIA Isaac™) - Data Model

## Overview
This document defines the data models and structures used in the NVIDIA Isaac module, including simulation data, perception outputs, navigation information, and transfer learning datasets.

## Core Data Models

### 1. Simulation Data Model

#### Environment Configuration
```yaml
Environment:
  name: string
  description: string
  scene_file: string  # USD file path
  lighting:
    type: "directional" | "point" | "spot"
    position: [x: float, y: float, z: float]
    intensity: float
    color: [r: float, g: float, b: float]
  physics:
    gravity: [x: float, y: float, z: float]
    friction: float
    restitution: float
  objects:
    - name: string
      type: "static" | "dynamic" | "kinematic"
      pose: [x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float]
      mass: float
      mesh_file: string
```

#### Robot Configuration
```yaml
Robot:
  name: string
  urdf_file: string
  joint_limits:
    position: [min: float, max: float]
    velocity: [min: float, max: float]
    effort: [min: float, max: float]
  sensors:
    - type: "camera" | "lidar" | "imu" | "joint_state"
      name: string
      frame_id: string
      parameters: object
  actuators:
    - joint_name: string
      control_type: "position" | "velocity" | "effort"
      parameters: object
```

### 2. Perception Data Model

#### Isaac ROS Perception Pipeline
```yaml
PerceptionPipeline:
  name: string
  nodes:
    - node_name: string
      node_type: string  # e.g., "isaac_ros_apriltag", "isaac_ros_centerpose"
      input_topics:
        - topic_name: string
          message_type: string
      output_topics:
        - topic_name: string
          message_type: string
      parameters: object
  performance_metrics:
    fps: float
    latency: float
    accuracy: float
```

#### Object Detection Result
```yaml
DetectionResult:
  timestamp: float
  frame_id: string
  detections:
    - class_name: string
      confidence: float
      bounding_box:
        x: float
        y: float
        width: float
        height: float
      pose:
        position: [x: float, y: float, z: float]
        orientation: [qx: float, qy: float, qz: float, qw: float]
```

#### Semantic Segmentation
```yaml
SegmentationResult:
  timestamp: float
  frame_id: string
  segmented_image: string  # image topic reference
  class_mapping:
    class_id: string
    class_name: string
    color: [r: float, g: float, b: float]
  statistics:
    class_counts: object
    coverage_percentage: float
```

### 3. Navigation Data Model

#### Navigation Goal
```yaml
NavigationGoal:
  goal_pose:
    position: [x: float, y: float, z: float]
    orientation: [qx: float, qy: float, qz: float, qw: float]
  frame_id: string
  behavior_tree: string  # path to BT file
  controller_id: string
  planner_id: string
```

#### Path Plan
```yaml
PathPlan:
  header:
    stamp: float
    frame_id: string
  poses:
    - position: [x: float, y: float, z: float]
      orientation: [qx: float, qy: float, qz: float, qw: float]
  planner_info:
    planner_name: string
    computation_time: float
    path_length: float
```

#### Costmap
```yaml
Costmap:
  metadata:
    resolution: float
    width: int
    height: int
    origin: [x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float]
  data: [int]  # flattened array of cost values
  update_frequency: float
  transform_tolerance: float
```

### 4. Sim-to-Real Transfer Data Model

#### Synthetic Dataset
```yaml
SyntheticDataset:
  name: string
  description: string
  generation_parameters:
    domain_randomization:
      lighting_variations: int
      texture_variations: int
      color_variations: int
      physics_variations: int
  statistics:
    total_samples: int
    classes: [string]
    annotations: int
    validation_split: float
  format: "image_classification" | "object_detection" | "segmentation" | "pose_estimation"
  storage_path: string
```

#### Domain Randomization Configuration
```yaml
DomainRandomization:
  textures:
    material_types: [string]
    color_ranges: [min: float, max: float]
    roughness_range: [min: float, max: float]
    metallic_range: [min: float, max: float]
  lighting:
    intensity_range: [min: float, max: float]
    color_temperature_range: [min: float, max: float]
    light_positions: [x_range: [min: float, max: float], y_range: [min: float, max: float], z_range: [min: float, max: float]]
  physics:
    friction_range: [min: float, max: float]
    restitution_range: [min: float, max: float]
    mass_range: [min: float, max: float]
```

### 5. Performance Monitoring Model

#### System Performance Metrics
```yaml
PerformanceMetrics:
  timestamp: float
  simulation:
    fps: float
    physics_update_rate: float
    rendering_time: float
  perception:
    pipeline_fps: float
    inference_time: float
    accuracy: float
  navigation:
    planning_time: float
    execution_time: float
    success_rate: float
  hardware:
    gpu_utilization: float
    gpu_memory: float
    cpu_utilization: float
    ram_usage: float
```

#### Training Progress
```yaml
TrainingProgress:
  epoch: int
  loss: float
  accuracy: float
  validation_loss: float
  validation_accuracy: float
  learning_rate: float
  batch_size: int
  samples_processed: int
  time_elapsed: float
  metrics:
    - name: string
      value: float
```

## Data Flow Architecture

### Simulation to Perception Pipeline
```
Environment Configuration → Robot State → Sensor Data → Isaac ROS Nodes → Perception Results → Navigation Input
```

### Training Data Pipeline
```
Synthetic Dataset Generation → Domain Randomization → Model Training → Transfer Validation → Real Robot Deployment
```

### Performance Monitoring Pipeline
```
System Metrics Collection → Performance Analysis → Optimization Recommendations → Parameter Adjustment → Performance Validation
```

## Serialization Formats

### Configuration Files
- **Format**: YAML
- **Purpose**: Environment and robot configuration
- **Schema Validation**: JSON Schema
- **Location**: `configs/` directory

### Dataset Storage
- **Format**: ROS 2 bag files or HDF5
- **Purpose**: Training and validation data
- **Compression**: LZ4 for real-time recording
- **Location**: `datasets/` directory

### Model Storage
- **Format**: ONNX or TensorRT
- **Purpose**: Trained perception models
- **Optimization**: TensorRT for inference acceleration
- **Location**: `models/` directory

## API Endpoints (for documentation)

### Isaac Sim API
```
POST /api/simulations - Create new simulation
GET /api/simulations/{id} - Get simulation status
POST /api/simulations/{id}/start - Start simulation
POST /api/simulations/{id}/stop - Stop simulation
GET /api/simulations/{id}/metrics - Get performance metrics
```

### Perception Service API
```
POST /api/perception/detect - Object detection
POST /api/perception/segment - Semantic segmentation
POST /api/perception/pose - Pose estimation
GET /api/perception/models - List available models
POST /api/perception/models - Upload new model
```

### Navigation Service API
```
POST /api/navigation/goal - Set navigation goal
GET /api/navigation/status - Get navigation status
POST /api/navigation/cancel - Cancel navigation
GET /api/navigation/path - Get current path
```

## Data Validation Rules

### Schema Validation
- All configuration files must pass JSON Schema validation
- Required fields must be present in all data models
- Data types must match defined schemas
- Enum values must be from defined sets

### Performance Validation
- FPS must be above minimum threshold (10 FPS for simulation)
- Inference time must be below maximum threshold
- Memory usage must be within acceptable limits
- GPU utilization should be monitored for optimization

### Safety Validation
- Joint limits must be respected in all robot commands
- Collision detection must be active during simulation
- Navigation goals must be within environment boundaries
- Emergency stop conditions must be defined and monitored