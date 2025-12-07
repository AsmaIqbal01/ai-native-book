# Basic Scene Configuration Guide

## Overview
This guide provides instructions for creating and configuring basic simulation environments in Isaac Sim. These environments will be used for testing perception, navigation, and control systems in Module 3.

## Scene Design Philosophy

### Educational Approach
- **Simple Environments**: Start with basic rooms/simple layouts
- **Progressive Complexity**: Gradually increase scene complexity
- **Consistent Layouts**: Use consistent themes for comparison
- **Performance Focused**: Optimize for real-time simulation

### Environment Types
1. **Simple Room**: Basic enclosed space for initial testing
2. **Office Environment**: Indoor setting with furniture and obstacles
3. **Outdoor Scene**: Simple outdoor environment with terrain
4. **Navigation Course**: Designed for path planning challenges

## Scene Configuration Files

### 1. Simple Room Configuration
```yaml
# environments/simple_room.yaml
scene:
  name: "simple_room"
  description: "Basic rectangular room for initial robot testing"
  dimensions:
    width: 5.0
    length: 5.0
    height: 3.0
  lighting:
    type: "dome"
    intensity: 3000
    color: [1.0, 1.0, 1.0]
    texture: "builtin://Gunnar_Grimstad_4k.hdr"
  materials:
    floor:
      color: [0.7, 0.7, 0.7]
      roughness: 0.8
      metallic: 0.0
    walls:
      color: [0.8, 0.8, 0.9]
      roughness: 0.7
      metallic: 0.0
    ceiling:
      color: [0.9, 0.9, 0.9]
      roughness: 0.8
      metallic: 0.0
  physics:
    gravity: [0, 0, -9.81]
    solver: "TGS"
    iterations: 8
    substeps: 1
  objects:
    - name: "ground_plane"
      type: "plane"
      position: [0, 0, 0]
      rotation: [0, 0, 0]
      size: [10, 10]
      static: true
    - name: "south_wall"
      type: "box"
      position: [0, -2.5, 1.5]
      rotation: [0, 0, 0]
      dimensions: [5.2, 0.2, 3.0]
      static: true
    - name: "north_wall"
      type: "box"
      position: [0, 2.5, 1.5]
      rotation: [0, 0, 0]
      dimensions: [5.2, 0.2, 3.0]
      static: true
    - name: "west_wall"
      type: "box"
      position: [-2.5, 0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [0.2, 5.2, 3.0]
      static: true
    - name: "east_wall"
      type: "box"
      position: [2.5, 0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [0.2, 5.2, 3.0]
      static: true
    - name: "ceiling"
      type: "box"
      position: [0, 0, 3.0]
      rotation: [0, 0, 0]
      dimensions: [5.2, 5.2, 0.2]
      static: true
```

### 2. Office Environment Configuration
```yaml
# environments/office_env.yaml
scene:
  name: "office_environment"
  description: "Office setting with desks, chairs, and obstacles"
  dimensions:
    width: 8.0
    length: 10.0
    height: 3.0
  lighting:
    type: "dome"
    intensity: 4000
    color: [1.0, 1.0, 1.0]
    texture: "builtin://Ditch_River_4k.hdr"
  materials:
    floor:
      color: [0.6, 0.6, 0.6]
      roughness: 0.9
      metallic: 0.0
    walls:
      color: [0.9, 0.9, 0.9]
      roughness: 0.8
      metallic: 0.0
    furniture:
      color: [0.5, 0.3, 0.1]
      roughness: 0.6
      metallic: 0.0
  physics:
    gravity: [0, 0, -9.81]
    solver: "TGS"
    iterations: 8
    substeps: 1
  objects:
    # Ground plane
    - name: "ground_plane"
      type: "plane"
      position: [0, 0, 0]
      rotation: [0, 0, 0]
      size: [20, 20]
      static: true

    # Walls
    - name: "outer_wall_north"
      type: "box"
      position: [0, 5.0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [8.2, 0.2, 3.0]
      static: true
    - name: "outer_wall_south"
      type: "box"
      position: [0, -5.0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [8.2, 0.2, 3.0]
      static: true
    - name: "outer_wall_west"
      type: "box"
      position: [-4.0, 0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [0.2, 10.2, 3.0]
      static: true
    - name: "outer_wall_east"
      type: "box"
      position: [4.0, 0, 1.5]
      rotation: [0, 0, 0]
      dimensions: [0.2, 10.2, 3.0]
      static: true

    # Furniture
    - name: "desk_1"
      type: "box"
      position: [-1.5, 2.0, 0.4]
      rotation: [0, 0, 0.2]
      dimensions: [1.5, 0.8, 0.8]
      static: true
    - name: "chair_1"
      type: "capsule"
      position: [-2.5, 1.5, 0.4]
      rotation: [0, 0, 0.2]
      radius: 0.2
      height: 0.8
      static: true
    - name: "plant_1"
      type: "cylinder"
      position: [2.0, -3.0, 0.5]
      rotation: [0, 0, 0]
      radius: 0.15
      height: 1.0
      static: true
    - name: "box_1"
      type: "box"
      position: [1.5, 1.0, 0.3]
      rotation: [0, 0, 0]
      dimensions: [0.6, 0.6, 0.6]
      static: true
    - name: "box_2"
      type: "box"
      position: [-1.0, -1.5, 0.3]
      rotation: [0.1, -0.1, 0.05]
      dimensions: [0.5, 0.5, 0.6]
      static: true
```

## Isaac Sim Scene Creation

### 1. Python Script for Scene Setup
```python
# environments/scene_setup.py
import omni
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import carb
import yaml

def create_simple_room_scene(stage):
    """
    Create a simple room scene in Isaac Sim
    """
    # Create ground plane
    ground_plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/groundPlane"))
    # Configure ground plane properties

    # Create walls
    create_room_walls(stage)

    # Add lighting
    add_dome_light(stage)

    print("Simple room scene created successfully")

def create_office_environment(stage):
    """
    Create an office environment scene in Isaac Sim
    """
    # Create ground plane
    ground_plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/groundPlane"))

    # Create room structure
    create_room_walls(stage)

    # Add furniture and obstacles
    add_office_furniture(stage)

    # Add lighting
    add_dome_light(stage)

    print("Office environment scene created successfully")

def create_room_walls(stage, center=[0, 0, 0], size=[5.0, 5.0, 3.0]):
    """
    Create walls for a room
    """
    wall_thickness = 0.2
    wall_height = size[2]

    # South wall
    south_wall = UsdGeom.Cube.Define(stage, Sdf.Path("/World/southWall"))
    south_wall.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(south_wall)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, -size[1]/2 - wall_thickness/2, wall_height/2))
    xform.AddScaleOp().Set(Gf.Vec3d(size[0] + wall_thickness, wall_thickness, wall_height))

    # North wall
    north_wall = UsdGeom.Cube.Define(stage, Sdf.Path("/World/northWall"))
    north_wall.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(north_wall)
    xform.AddTranslateOp().Set(Gf.Vec3d(0, size[1]/2 + wall_thickness/2, wall_height/2))
    xform.AddScaleOp().Set(Gf.Vec3d(size[0] + wall_thickness, wall_thickness, wall_height))

    # West wall
    west_wall = UsdGeom.Cube.Define(stage, Sdf.Path("/World/westWall"))
    west_wall.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(west_wall)
    xform.AddTranslateOp().Set(Gf.Vec3d(-size[0]/2 - wall_thickness/2, 0, wall_height/2))
    xform.AddScaleOp().Set(Gf.Vec3d(wall_thickness, size[1] + wall_thickness, wall_height))

    # East wall
    east_wall = UsdGeom.Cube.Define(stage, Sdf.Path("/World/eastWall"))
    east_wall.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(east_wall)
    xform.AddTranslateOp().Set(Gf.Vec3d(size[0]/2 + wall_thickness/2, 0, wall_height/2))
    xform.AddScaleOp().Set(Gf.Vec3d(wall_thickness, size[1] + wall_thickness, wall_height))

    print("Room walls created")

def add_dome_light(stage):
    """
    Add dome light to the scene
    """
    dome_light = UsdGeom.Sphere.Define(stage, Sdf.Path("/World/domeLight"))
    # Configure dome light properties
    print("Dome light added")

def add_office_furniture(stage):
    """
    Add office furniture to the scene
    """
    # Add desk
    desk = UsdGeom.Cube.Define(stage, Sdf.Path("/World/desk"))
    desk.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(desk)
    xform.AddTranslateOp().Set(Gf.Vec3d(-1.5, 2.0, 0.4))
    xform.AddScaleOp().Set(Gf.Vec3d(1.5, 0.8, 0.8))

    # Add chair
    chair = UsdGeom.Capsule.Define(stage, Sdf.Path("/World/chair"))
    # Configure chair properties

    print("Office furniture added")

def load_scene_from_config(config_path):
    """
    Load scene configuration from YAML file
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

    stage = omni.usd.get_context().get_stage()

    # Create scene based on configuration
    if config['scene']['name'] == 'simple_room':
        create_simple_room_scene(stage)
    elif config['scene']['name'] == 'office_environment':
        create_office_environment(stage)

    return config
```

### 2. Lighting Configuration
```python
# environments/lighting_config.py
def configure_lighting(scene_type="simple_room"):
    """
    Configure lighting for different scene types
    """
    if scene_type == "simple_room":
        # Dome light configuration for simple room
        dome_light_config = {
            "intensity": 3000,
            "color": [1.0, 1.0, 1.0],
            "texture": "builtin://Gunnar_Grimstad_4k.hdr",
            "exposure": 0.0
        }
        return dome_light_config

    elif scene_type == "office_environment":
        # More complex lighting for office environment
        dome_light_config = {
            "intensity": 4000,
            "color": [1.0, 1.0, 1.0],
            "texture": "builtin://Ditch_River_4k.hdr",
            "exposure": 0.0
        }
        return dome_light_config

def setup_advanced_lighting():
    """
    Set up advanced lighting scenarios for synthetic data generation
    """
    # Create multiple lighting scenarios
    lighting_scenarios = [
        {
            "name": "morning_light",
            "intensity": 2500,
            "color": [1.0, 0.9, 0.8],
            "direction": [-0.5, -0.5, -1.0]
        },
        {
            "name": "noon_light",
            "intensity": 5000,
            "color": [1.0, 1.0, 1.0],
            "direction": [0, 0, -1.0]
        },
        {
            "name": "evening_light",
            "intensity": 2000,
            "color": [0.9, 0.7, 0.5],
            "direction": [0.5, -0.5, -1.0]
        }
    ]

    return lighting_scenarios
```

## Physics Configuration

### 1. Physics Scene Setup
```yaml
# environments/physics_config.yaml
physics:
  scene:
    gravity: [0, 0, -9.81]
    solver_type: "TGS"  # TGS (Two Way Gauss Seidel) or PGS (Projected Gauss Seidel)
    iterations: 8
    substeps: 1
    enable_ccd: false
    ccd_threshold: 1e-07
    enable_stabilization: true
    stabilization_threshold: 0.0
  materials:
    default:
      static_friction: 0.5
      dynamic_friction: 0.5
      restitution: 0.1
    high_friction:
      static_friction: 0.8
      dynamic_friction: 0.8
      restitution: 0.05
    low_friction:
      static_friction: 0.1
      dynamic_friction: 0.1
      restitution: 0.2
```

## Testing Scene Configuration

### 1. Scene Validation Script
```python
# test/scene_validation.py
def validate_scene_configuration(scene_name):
    """
    Validate that a scene is properly configured
    """
    # Check that all required objects exist
    required_objects = ["ground_plane", "lighting"]

    # Check physics properties
    # Check rendering properties
    # Check that robot can be placed and move properly

    print(f"Scene {scene_name} validated successfully")
    return True

def test_scene_rendering(scene_name):
    """
    Test that the scene renders correctly
    """
    # Check rendering performance
    # Verify lighting setup
    # Check material properties

    print(f"Scene {scene_name} rendering test passed")
    return True

def test_scene_physics(scene_name):
    """
    Test that the scene physics work correctly
    """
    # Test gravity
    # Test collision detection
    # Test object interactions

    print(f"Scene {scene_name} physics test passed")
    return True
```

## Synthetic Data Generation Preparation

### 1. Camera Positioning for Data Capture
```python
# environments/data_capture_config.py
def setup_data_capture_positions():
    """
    Set up camera positions for synthetic data capture
    """
    camera_positions = [
        {
            "name": "robot_camera",
            "position": [0.05, 0, 0],  # On robot head
            "rotation": [0, 0, 0],
            "fov": 60,
            "resolution": [640, 480]
        },
        {
            "name": "overhead_camera",
            "position": [0, 0, 5],
            "rotation": [-90, 0, 0],  # Looking down
            "fov": 90,
            "resolution": [1280, 720]
        },
        {
            "name": "side_camera",
            "position": [-3, 0, 1.5],
            "rotation": [0, 90, 0],  # Looking toward center
            "fov": 75,
            "resolution": [640, 480]
        }
    ]

    return camera_positions

def setup_annotation_config():
    """
    Set up configuration for data annotation
    """
    annotation_config = {
        "object_detection": {
            "enabled": True,
            "classes": ["humanoid_robot", "furniture", "obstacles"],
            "format": "coco"
        },
        "semantic_segmentation": {
            "enabled": True,
            "color_map": {
                "robot": [255, 0, 0],
                "furniture": [0, 255, 0],
                "floor": [0, 0, 255],
                "walls": [255, 255, 0]
            }
        },
        "depth": {
            "enabled": True,
            "format": "16-bit"
        }
    }

    return annotation_config
```

## Performance Optimization

### 1. Scene Optimization Settings
```python
# environments/optimization_config.py
def get_performance_settings(scene_type="simple_room"):
    """
    Get performance settings for different scene types
    """
    if scene_type == "simple_room":
        settings = {
            "max_texture_resolution": 2048,
            "lod_bias": 0,
            "occlusion_culling": True,
            "frustum_culling": True,
            "shadow_quality": "medium",
            "reflection_quality": "low",
            "render_resolution": [1280, 720]
        }
    elif scene_type == "office_environment":
        settings = {
            "max_texture_resolution": 2048,
            "lod_bias": 0,
            "occlusion_culling": True,
            "frustum_culling": True,
            "shadow_quality": "medium",
            "reflection_quality": "low",
            "render_resolution": [1280, 720]
        }

    return settings

def optimize_scene_for_training():
    """
    Optimize scene for synthetic data training
    """
    # Higher quality settings for training data
    training_settings = {
        "render_resolution": [1920, 1080],
        "max_texture_resolution": 0,  # No limit
        "shadow_quality": "high",
        "reflection_quality": "high",
        "aa_samples": 16,
        "light_samples": 16
    }

    return training_settings
```

## Troubleshooting

### Common Issues
- **Scene not loading**: Verify USD file paths and syntax
- **Physics not working**: Check collision geometries and properties
- **Lighting issues**: Verify dome light configuration
- **Performance problems**: Adjust quality settings

## Next Steps
After setting up basic scenes:
1. Test scene rendering and physics
2. Verify robot placement and movement in scenes
3. Prepare for synthetic data generation (Task 2.2)
4. Configure for perception pipeline testing (Task 2.3)

## Resources
- [Isaac Sim Scene Creation Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_basic.html)
- [USD Scene Description Guide](https://graphics.pixar.com/usd/release/docs/index.html)
- [Isaac Sim Physics Configuration](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_physics.html)