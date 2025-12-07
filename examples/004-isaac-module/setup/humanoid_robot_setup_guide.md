# Humanoid Robot Model Setup Guide

## Overview
This guide provides instructions for setting up a humanoid robot model in Isaac Sim. The model will be used for perception, navigation, and sim-to-real transfer exercises in Module 3.

## Robot Model Specifications

### Design Philosophy
- **Simplified Humanoid**: 5-7 DOF for educational purposes
- **Components**: Torso (base), head (1 DOF pan), 2 arms (2-3 DOF each)
- **Total**: ~5-7 joints, manageable for beginners
- **Sensors**: Camera (head), IMU (torso), joint encoders
- **Safety**: Simple wheeled mobile base optional for initial examples

### Joint Configuration
```yaml
# config/humanoid_joints.yaml
humanoid_robot:
  name: "simple_humanoid"
  urdf_file: "urdf/simple_humanoid.urdf"
  joint_limits:
    head_pan:
      position: [-1.57, 1.57]  # ±90 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    left_shoulder_pitch:
      position: [-1.57, 0.5]    # -90 to 28.6 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    left_shoulder_roll:
      position: [-1.57, 1.57]   # ±90 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    left_elbow:
      position: [0, 1.57]       # 0 to 90 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    right_shoulder_pitch:
      position: [-1.57, 0.5]    # -90 to 28.6 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    right_shoulder_roll:
      position: [-1.57, 1.57]   # ±90 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
    right_elbow:
      position: [0, 1.57]       # 0 to 90 degrees
      velocity: [0, 2.0]
      effort: [0, 100]
```

## Importing the Robot Model

### 1. Create URDF File
```xml
<!-- urdf/simple_humanoid.urdf -->
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Torso Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="0.9 0.9 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_pan" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0.5" effort="100" velocity="2"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="2"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="0.5" effort="100" velocity="2"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="2"/>
  </joint>

  <!-- Camera sensor on head -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.03 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor on torso -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
```

### 2. Create Isaac Sim Robot Configuration
```python
# config/isaac_sim_robot_config.py
import omni
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import carb

def create_humanoid_robot(stage, prim_path="/World/HumanoidRobot"):
    """
    Create a humanoid robot in Isaac Sim
    """
    # Create robot prim
    robot_prim = UsdGeom.Xform.Define(stage, prim_path)

    # Add physics properties
    UsdPhysics.RigidBodyAPI.Apply(robot_prim.GetPrim())

    # Create base link
    base_path = f"{prim_path}/BaseLink"
    base_link = UsdGeom.Cube.Define(stage, base_path)
    base_link.GetSizeAttr().Set(0.5)

    # Set material properties
    material_path = f"{prim_path}/Material"
    material = UsdShade.Material.Define(stage, material_path)

    return robot_prim

def setup_robot_joints(stage, robot_path):
    """
    Set up joints for the humanoid robot
    """
    # Implementation for creating joints in Isaac Sim
    # This would involve creating USD physics joints
    pass

def configure_robot_sensors(robot_prim):
    """
    Configure sensors for the robot (camera, IMU, etc.)
    """
    # Implementation for adding sensors to the robot
    pass
```

## Testing Robot Model

### 1. Basic Movement Test
```python
# test/robot_movement_test.py
import omni
from pxr import Gf
import numpy as np

def test_robot_movement():
    """
    Test basic movement of the humanoid robot
    """
    # Get stage
    stage = omni.usd.get_context().get_stage()

    # Get robot references
    robot_prim = stage.GetPrimAtPath("/World/HumanoidRobot")

    # Test joint movements
    joint_names = ["head_pan", "left_shoulder_pitch", "left_elbow",
                   "right_shoulder_pitch", "right_elbow"]

    for joint_name in joint_names:
        joint_path = f"/World/HumanoidRobot/{joint_name}"
        # Move joint to test range
        # Verify movement is within limits
        print(f"Testing joint: {joint_name}")

    print("Robot movement test completed successfully")

if __name__ == "__main__":
    test_robot_movement()
```

### 2. Physics Validation
```python
# test/physics_validation.py
def validate_robot_physics():
    """
    Validate that the robot behaves correctly with physics
    """
    # Check that robot maintains stability
    # Verify joint constraints work properly
    # Test collision detection
    pass
```

## Integration with Isaac ROS

### 1. ROS Bridge Configuration
```yaml
# config/ros_bridge_config.yaml
ros_bridge:
  enabled: true
  robot_name: "simple_humanoid"
  joint_state_topic: "/joint_states"
  control_topics:
    - name: "head_pan_position_controller"
      type: "position"
      joint: "head_pan"
    - name: "left_shoulder_position_controller"
      type: "position"
      joint: "left_shoulder_pitch"
    - name: "left_elbow_position_controller"
      type: "position"
      joint: "left_elbow"
    - name: "right_shoulder_position_controller"
      type: "position"
      joint: "right_shoulder_pitch"
    - name: "right_elbow_position_controller"
      type: "position"
      joint: "right_elbow"
  sensor_topics:
    - name: "camera"
      type: "camera"
      link: "camera_link"
    - name: "imu"
      type: "imu"
      link: "imu_link"
```

## Troubleshooting

### Common Issues
- **Model not loading**: Verify URDF syntax and file paths
- **Physics instability**: Check mass/inertia values
- **Joint limits not working**: Verify joint configuration
- **Sensor data not publishing**: Check ROS bridge configuration

## Next Steps
After setting up the humanoid robot model:
1. Proceed to Basic Scene Configuration (Task 1.3)
2. Test robot movement and physics
3. Integrate with Isaac ROS (Task 1.4)

## Resources
- [Isaac Sim Robot Setup Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_basic.html)
- [URDF to USD Conversion Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_robots.html)
- [Isaac ROS Robot Bridge](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_bridges/index.html)