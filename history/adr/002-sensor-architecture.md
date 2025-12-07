# ADR-002: Sensor Architecture for Digital Twin Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 003-digital-twin-module
- **Context:** Module 2 needs to teach sensor simulation for humanoid robotics perception. Students must learn realistic sensor data generation, ROS 2 integration, and sensor-based control. The sensor suite must cover essential perception modalities (distance, depth, orientation) without overwhelming beginners, work within performance constraints (8GB RAM, ≥0.5x real-time), and integrate cleanly with ROS 2 topics for closed-loop demonstrations.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines data model and perception capabilities for all examples
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - RGB camera, stereo camera, GPS, tactile sensors all considered
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects URDF structure, ROS nodes, launch files, RViz configs, performance requirements
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

We will use the following integrated sensor architecture:

- **Sensor Suite:**
  - **LiDAR:** Torso-mounted, 360° horizontal scan, 0.1-10m range, 1Hz update, publishes `sensor_msgs/LaserScan` to `/scan`
  - **Depth Camera:** Head-mounted, 640x480 resolution, 90° FOV, 10Hz update, publishes `sensor_msgs/PointCloud2` to `/camera/depth/points`
  - **IMU:** Torso-mounted, 100Hz update, Gaussian noise (0.01 rad/s angular, 0.1 m/s² linear), publishes `sensor_msgs/Imu` to `/imu`

- **URDF Configuration Strategy:**
  - Three progressive URDF variants: `simple_humanoid_lidar.urdf`, `simple_humanoid_depth.urdf`, `simple_humanoid_full.urdf` (all sensors)
  - Each extends base `simple_humanoid.urdf` from Module 1 with Gazebo sensor plugins
  - XML-based Gazebo plugin configuration inline in URDF `<sensor>` tags

- **ROS 2 Integration:**
  - All sensors use standard `sensor_msgs` message types (no custom messages)
  - Topic names follow ROS 2 conventions (`/scan`, `/camera/depth/points`, `/imu`)
  - Python subscriber examples demonstrate reading each sensor type
  - Closed-loop demo integrates IMU with joint controller for sensor-based behavior

## Consequences

### Positive

- **Comprehensive Coverage:** LiDAR (distance), Depth (3D geometry), IMU (orientation/acceleration) cover essential perception modalities for robotics
- **Beginner-Friendly Scope:** Three sensors are sufficient for learning without cognitive overload (spec confirmed no RGB camera needed)
- **Standard ROS 2 Messages:** Using `sensor_msgs` ensures compatibility with ROS 2 ecosystem tools (RViz, rqt, rosbag)
- **Progressive Complexity:** Three URDF variants let students add sensors incrementally, understanding each plugin individually
- **Performance Optimized:** 1Hz LiDAR and 10Hz Depth Camera keep simulations within ≥0.5x real-time on 8GB RAM
- **Realistic Noise Models:** Gaussian noise in IMU teaches students about sensor uncertainty (critical for real-world robotics)
- **Reusable Patterns:** Sensor plugin patterns (update rate, noise config, topic names) are transferable to other sensors in future work

### Negative

- **No Visual Perception:** Lack of RGB camera excludes computer vision tutorials (out of scope per spec, but limits perception coverage)
- **Limited Sensor Diversity:** Students don't learn tactile, force/torque, or radar sensors (deferred to advanced modules)
- **XML Configuration Complexity:** Gazebo plugin syntax in URDF is verbose and error-prone for beginners (mitigated with clear examples)
- **Update Rate Trade-offs:** 1Hz LiDAR is slow for real-time obstacle avoidance demos (chosen for performance over realism)
- **Noise Simplification:** Only Gaussian noise (no Perlin, systematic errors) may not reflect real sensor characteristics

## Alternatives Considered

### Alternative Sensor Suite A: LiDAR + RGB Camera + IMU
- **Components:** Replace Depth Camera with RGB camera (`sensor_msgs/Image`)
- **Why Rejected:** RGB camera doesn't provide depth data needed for 3D perception demos; computer vision (image processing) is out of scope per spec; Depth Camera is more directly useful for robotics navigation

### Alternative Sensor Suite B: Stereo Camera (replacing LiDAR and Depth)
- **Components:** Single stereo camera plugin generates both RGB and depth from disparity
- **Why Rejected:** More complex to configure (baseline, calibration params), higher computational cost, less educational clarity (students must understand stereo vision math vs direct depth sensing)

### Alternative Sensor Suite C: LiDAR + Depth + IMU + GPS
- **Components:** Add GPS sensor for outdoor navigation scenarios
- **Why Rejected:** Humanoid robotics focus is indoor (no outdoor world files planned); GPS adds complexity without clear educational value for this module; GPS simulation in Gazebo is less realistic than LiDAR/Depth

### Alternative Configuration Strategy: Single URDF with All Sensors
- **Components:** One `simple_humanoid_sensors.urdf` instead of three progressive variants
- **Why Rejected:** Less pedagogical - students benefit from seeing incremental sensor addition; harder to debug (all plugins active at once); doesn't teach URDF extensibility pattern

## References

- Feature Spec: `specs/003-digital-twin-module/spec.md` (FR-009 to FR-015, sensor specifications)
- Implementation Plan: `specs/003-digital-twin-module/plan.md`
- Data Model: `specs/003-digital-twin-module/data-model.md` (ROS 2 message types)
- Related ADRs: ADR-001 (Simulation Platform Stack - sensor plugins require Gazebo Classic)
- Evaluator Evidence: None (initial design decision)
- [Gazebo Sensor Plugins](http://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Sensor)
- [ROS 2 sensor_msgs Package](https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs)
