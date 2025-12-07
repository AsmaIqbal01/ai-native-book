# Photorealistic Simulation Environment Guide

## Overview
This guide provides instructions for developing realistic environments with advanced rendering in Isaac Sim. These environments will be used for synthetic data generation and perception system training in Module 3.

## Advanced Rendering Features

### 1. Physically-Based Rendering (PBR)
- **Materials**: Use physically accurate materials with proper roughness and metallic properties
- **Lighting**: Implement HDR lighting with realistic intensity values
- **Shadows**: Enable realistic shadow casting and receiving
- **Reflections**: Configure mirror-like and glossy reflections

### 2. Rendering Techniques
- **Global Illumination**: Enable light bouncing for realistic indirect lighting
- **Anti-Aliasing**: Use temporal anti-aliasing (TAA) for smooth edges
- **Motion Blur**: Add realistic motion blur for moving objects
- **Depth of Field**: Configure camera focus effects

## Environment Creation

### 1. Indoor Environments

#### Modern Office Space
```yaml
# environments/advanced/modern_office.yaml
scene:
  name: "modern_office"
  description: "Photorealistic modern office environment"
  lighting:
    type: "dome"
    intensity: 3500
    color: [1.0, 1.0, 1.0]
    texture: "builtin://Ditch_River_4k.hdr"
    exposure: -1.0
  materials:
    floor:
      type: "pbr"
      base_color: [0.8, 0.8, 0.8]
      roughness: 0.9
      metallic: 0.0
      normal_map: "textures/polished_floor_normal.png"
      occlusion_map: "textures/polished_floor_ao.png"
    walls:
      type: "pbr"
      base_color: [0.95, 0.95, 0.95]
      roughness: 0.8
      metallic: 0.0
      normal_map: "textures/wall_normal.png"
    furniture:
      type: "pbr"
      base_color: [0.6, 0.4, 0.2]
      roughness: 0.7
      metallic: 0.0
      normal_map: "textures/wood_grain_normal.png"
  objects:
    - name: "floor"
      type: "plane"
      position: [0, 0, 0]
      size: [20, 20]
      material: "floor"
      static: true
    - name: "glass_partition"
      type: "box"
      position: [0, 3, 1.5]
      dimensions: [4, 0.05, 3]
      material:
        type: "glass"
        base_color: [0.9, 0.95, 1.0]
        roughness: 0.05
        metallic: 0.0
        ior: 1.52
        transmission: 0.95
      static: true
    - name: "desk"
      type: "box"
      position: [-2, 1, 0.75]
      dimensions: [1.8, 0.8, 0.75]
      material: "furniture"
      static: true
    - name: "office_chair"
      type: "capsule"
      position: [-2.5, 0.5, 0.75]
      radius: 0.3
      height: 0.8
      material: "furniture"
      static: true
    - name: "monitor"
      type: "box"
      position: [-2, 1.2, 1.2]
      dimensions: [0.5, 0.3, 0.01]
      material:
        type: "pbr"
        base_color: [0.1, 0.1, 0.1]
        roughness: 0.1
        metallic: 0.9
      static: true
```

#### Industrial Warehouse
```yaml
# environments/advanced/industrial_warehouse.yaml
scene:
  name: "industrial_warehouse"
  description: "Large industrial warehouse with heavy machinery"
  lighting:
    type: "dome"
    intensity: 4000
    color: [1.0, 1.0, 1.0]
    texture: "builtin://Pleasant_Point_4k.hdr"
    exposure: -0.5
  materials:
    concrete_floor:
      type: "pbr"
      base_color: [0.6, 0.6, 0.6]
      roughness: 0.9
      metallic: 0.0
      normal_map: "textures/concrete_normal.png"
      height_map: "textures/concrete_height.png"
    metal:
      type: "pbr"
      base_color: [0.7, 0.7, 0.7]
      roughness: 0.3
      metallic: 0.9
      normal_map: "textures/metal_scratches_normal.png"
    wood_pallet:
      type: "pbr"
      base_color: [0.5, 0.3, 0.1]
      roughness: 0.8
      metallic: 0.0
      normal_map: "textures/wood_grain_normal.png"
  objects:
    - name: "warehouse_floor"
      type: "plane"
      position: [0, 0, 0]
      size: [50, 50]
      material: "concrete_floor"
      static: true
    - name: "warehouse_walls"
      type: "group"
      objects:
        - type: "box"
          name: "north_wall"
          position: [0, 25, 5]
          dimensions: [50.2, 0.2, 10]
          material: "concrete_floor"
        - type: "box"
          name: "south_wall"
          position: [0, -25, 5]
          dimensions: [50.2, 0.2, 10]
          material: "concrete_floor"
        - type: "box"
          name: "east_wall"
          position: [25, 0, 5]
          dimensions: [0.2, 50.2, 10]
          material: "concrete_floor"
        - type: "box"
          name: "west_wall"
          position: [-25, 0, 5]
          dimensions: [0.2, 50.2, 10]
          material: "concrete_floor"
    - name: "forklift"
      type: "group"
      objects:
        - type: "box"
          name: "forklift_body"
          position: [5, 0, 0.8]
          dimensions: [2.5, 1.2, 1.5]
          material: "metal"
        - type: "cylinder"
          name: "forklift_wheel_1"
          position: [4, 0.7, 0.3]
          radius: 0.3
          height: 0.2
          material: "metal"
        - type: "cylinder"
          name: "forklift_wheel_2"
          position: [4, -0.7, 0.3]
          radius: 0.3
          height: 0.2
          material: "metal"
    - name: "cargo_pallets"
      type: "array"
      count: 8
      spacing: [2, 2, 0]
      objects:
        - type: "box"
          name: "pallet"
          position: [0, 0, 0.3]
          dimensions: [1.2, 1.0, 0.6]
          material: "wood_pallet"
```

### 2. Outdoor Environments

#### Urban Street Scene
```yaml
# environments/advanced/urban_street.yaml
scene:
  name: "urban_street"
  description: "Photorealistic urban street environment"
  lighting:
    type: "dome"
    intensity: 5000
    color: [1.0, 1.0, 1.0]
    texture: "builtin://Ditch_River_4k.hdr"
    exposure: 0.0
  materials:
    asphalt:
      type: "pbr"
      base_color: [0.2, 0.2, 0.2]
      roughness: 0.9
      metallic: 0.0
      normal_map: "textures/asphalt_normal.png"
    building_wall:
      type: "pbr"
      base_color: [0.7, 0.7, 0.7]
      roughness: 0.7
      metallic: 0.0
      normal_map: "textures/brick_normal.png"
    building_glass:
      type: "pbr"
      base_color: [0.8, 0.9, 1.0]
      roughness: 0.05
      metallic: 0.0
      ior: 1.52
      transmission: 0.9
  objects:
    - name: "street"
      type: "plane"
      position: [0, 0, 0]
      size: [100, 20]
      material: "asphalt"
      static: true
    - name: "building_row_1"
      type: "array"
      count: 5
      spacing: [20, 0, 0]
      objects:
        - type: "box"
          name: "building"
          position: [0, -15, 10]
          dimensions: [15, 10, 20]
          material: "building_wall"
        - type: "box"
          name: "building_windows"
          position: [0, -15, 10]
          dimensions: [14.8, 9.8, 19.8]
          material: "building_glass"
    - name: "building_row_2"
      type: "array"
      count: 5
      spacing: [20, 0, 0]
      objects:
        - type: "box"
          name: "building"
          position: [0, 15, 10]
          dimensions: [15, 10, 20]
          material: "building_wall"
        - type: "box"
          name: "building_windows"
          position: [0, 15, 10]
          dimensions: [14.8, 9.8, 19.8]
          material: "building_glass"
    - name: "street_lamp"
      type: "array"
      count: 6
      spacing: [20, 0, 0]
      objects:
        - type: "cylinder"
          name: "lamp_pole"
          position: [0, 0, 4]
          radius: 0.1
          height: 8
          material: "building_glass"
        - type: "sphere"
          name: "lamp_head"
          position: [0, 0, 8.5]
          radius: 0.2
          emissive_color: [1.0, 0.9, 0.7]
          emissive_intensity: 100
```

## Advanced Lighting Scenarios

### 1. Time-of-Day Variations
```python
# environments/lighting_scenarios.py
import math

def get_lighting_scenario(scenario_name):
    """
    Get different lighting scenarios for time-of-day variations
    """
    scenarios = {
        "dawn": {
            "intensity": 2000,
            "color": [1.0, 0.7, 0.4],  # Warm orange
            "direction": [-0.5, -0.5, -0.8],
            "exposure": -1.0,
            "shadow_softness": 0.8
        },
        "noon": {
            "intensity": 8000,
            "color": [1.0, 1.0, 1.0],  # White
            "direction": [0.2, -0.8, -0.9],
            "exposure": 0.0,
            "shadow_softness": 0.2
        },
        "sunset": {
            "intensity": 2500,
            "color": [1.0, 0.5, 0.2],  # Warm red-orange
            "direction": [0.5, -0.5, -0.8],
            "exposure": -0.5,
            "shadow_softness": 0.9
        },
        "indoor_daylight": {
            "intensity": 3000,
            "color": [0.95, 0.98, 1.0],  # Cool white
            "direction": [0, 0, -1],
            "exposure": -0.5,
            "shadow_softness": 0.3
        },
        "artificial_lighting": {
            "intensity": 4000,
            "color": [1.0, 0.95, 0.9],  # Warm white
            "direction": [0, 0, -1],
            "exposure": 0.0,
            "shadow_softness": 0.6
        }
    }

    return scenarios.get(scenario_name, scenarios["noon"])

def apply_lighting_scenario(stage, scenario_name):
    """
    Apply a specific lighting scenario to the scene
    """
    scenario = get_lighting_scenario(scenario_name)

    # Apply dome light with scenario parameters
    # Configure additional lights if needed
    # Adjust camera exposure

    print(f"Applied lighting scenario: {scenario_name}")
    return scenario
```

### 2. Weather Variations
```python
# environments/weather_scenarios.py
def get_weather_scenario(weather_type):
    """
    Get different weather scenarios with atmospheric effects
    """
    scenarios = {
        "clear_sky": {
            "fog_density": 0.0,
            "fog_color": [0.8, 0.9, 1.0],
            "atmospheric_haze": 0.0,
            "cloud_coverage": 0.0,
            "precipitation": 0.0,
            "wind_speed": 0.0
        },
        "overcast": {
            "fog_density": 0.02,
            "fog_color": [0.7, 0.75, 0.8],
            "atmospheric_haze": 0.1,
            "cloud_coverage": 0.8,
            "precipitation": 0.0,
            "wind_speed": 1.0
        },
        "foggy": {
            "fog_density": 0.08,
            "fog_color": [0.8, 0.85, 0.9],
            "atmospheric_haze": 0.2,
            "cloud_coverage": 0.6,
            "precipitation": 0.0,
            "wind_speed": 0.5
        },
        "rainy": {
            "fog_density": 0.03,
            "fog_color": [0.6, 0.65, 0.7],
            "atmospheric_haze": 0.15,
            "cloud_coverage": 0.95,
            "precipitation": 0.7,
            "wind_speed": 3.0
        }
    }

    return scenarios.get(weather_type, scenarios["clear_sky"])

def apply_weather_scenario(stage, weather_type):
    """
    Apply a specific weather scenario to the scene
    """
    scenario = get_weather_scenario(weather_type)

    # Configure atmospheric effects
    # Adjust lighting for weather conditions
    # Apply precipitation effects if needed

    print(f"Applied weather scenario: {weather_type}")
    return scenario
```

## Performance Optimization

### 1. Quality Settings Configuration
```python
# environments/performance_config.py
def get_rendering_quality_settings(quality_level="balanced"):
    """
    Get rendering quality settings for different performance levels
    """
    quality_settings = {
        "ultra": {
            "renderer": "Ray Traced Light Map",
            "renderMode": "material",
            "maxTextureResolution": 0,  # Unlimited
            "groundplane": True,
            "domeLight": True,
            "complexion": True,
            "aa_samples": 16,
            "light_samples": 16,
            "max_bounces": 8,
            "frame_rate": 30
        },
        "high": {
            "renderer": "HydraEngine",
            "renderMode": "material",
            "maxTextureResolution": 4096,
            "groundplane": True,
            "domeLight": True,
            "complexion": True,
            "aa_samples": 8,
            "light_samples": 8,
            "max_bounces": 4,
            "frame_rate": 30
        },
        "balanced": {
            "renderer": "HydraEngine",
            "renderMode": "material",
            "maxTextureResolution": 2048,
            "groundplane": True,
            "domeLight": True,
            "complexion": False,
            "aa_samples": 4,
            "light_samples": 4,
            "max_bounces": 2,
            "frame_rate": 30
        },
        "performance": {
            "renderer": "HydraEngine",
            "renderMode": "material",
            "maxTextureResolution": 1024,
            "groundplane": False,
            "domeLight": False,
            "complexion": False,
            "aa_samples": 2,
            "light_samples": 2,
            "max_bounces": 1,
            "frame_rate": 60
        }
    }

    return quality_settings.get(quality_level, quality_settings["balanced"])

def optimize_for_synthetic_data():
    """
    Optimize rendering settings for synthetic data generation
    """
    data_optimized_settings = {
        "renderer": "Ray Traced Light Map",  # Highest quality for training
        "renderMode": "material",
        "maxTextureResolution": 0,  # No limit for training data
        "groundplane": True,
        "domeLight": True,
        "complexion": True,
        "aa_samples": 16,
        "light_samples": 16,
        "max_bounces": 8,
        "frame_rate": 1  # Slow for high quality, not real-time
    }

    return data_optimized_settings
```

### 2. LOD (Level of Detail) Configuration
```yaml
# environments/lod_config.yaml
lod_system:
  enabled: true
  max_lods: 4
  screen_size_thresholds:
    - distance: 10.0  # Very detailed model
      screen_size: 0.5
    - distance: 20.0  # Detailed model
      screen_size: 0.2
    - distance: 50.0  # Simplified model
      screen_size: 0.05
    - distance: 100.0  # Proxy/placeholder
      screen_size: 0.01
  detail_reduction:
    - "high_to_medium": 0.5  # Reduce to 50% detail
    - "medium_to_low": 0.3   # Reduce to 30% detail
    - "low_to_proxy": 0.1    # Reduce to 10% detail
```

## Synthetic Data Generation

### 1. Camera Configuration for Data Capture
```python
# environments/data_capture_config.py
def get_camera_configurations():
    """
    Get different camera configurations for synthetic data capture
    """
    camera_configs = [
        {
            "name": "front_facing_camera",
            "position": [0.1, 0, 1.2],  # Robot head level
            "rotation": [0, 0, 0],
            "fov": 60,
            "resolution": [1920, 1080],
            "format": "RGB8",
            "frame_rate": 30,
            "clip_near": 0.1,
            "clip_far": 100.0
        },
        {
            "name": "wide_angle_camera",
            "position": [0.1, 0, 1.2],
            "rotation": [0, 0, 0],
            "fov": 120,
            "resolution": [1280, 720],
            "format": "RGB8",
            "frame_rate": 30,
            "clip_near": 0.1,
            "clip_far": 50.0
        },
        {
            "name": "overhead_camera",
            "position": [0, 0, 10],
            "rotation": [-90, 0, 0],
            "fov": 90,
            "resolution": [1920, 1080],
            "format": "RGB8",
            "frame_rate": 10,
            "clip_near": 1.0,
            "clip_far": 20.0
        },
        {
            "name": "side_camera",
            "position": [-5, 0, 2],
            "rotation": [0, 90, 0],
            "fov": 75,
            "resolution": [1280, 720],
            "format": "RGB8",
            "frame_rate": 30,
            "clip_near": 0.5,
            "clip_far": 15.0
        }
    ]

    return camera_configs

def setup_annotation_pipeline():
    """
    Set up annotation pipeline for synthetic data
    """
    annotation_config = {
        "object_detection": {
            "enabled": True,
            "classes": ["humanoid_robot", "furniture", "obstacles", "humans", "vehicles"],
            "format": "coco",
            "bounding_box_format": "xywh",
            "confidence_threshold": 0.5
        },
        "semantic_segmentation": {
            "enabled": True,
            "color_map": {
                "robot": [255, 0, 0],
                "furniture": [0, 255, 0],
                "floor": [0, 0, 255],
                "walls": [255, 255, 0],
                "humans": [255, 0, 255],
                "obstacles": [0, 255, 255],
                "sky": [135, 206, 235]
            },
            "format": "png"
        },
        "instance_segmentation": {
            "enabled": True,
            "format": "coco"
        },
        "depth": {
            "enabled": True,
            "format": "16-bit",
            "min_depth": 0.1,
            "max_depth": 100.0,
            "unit": "meters"
        },
        "pose_estimation": {
            "enabled": True,
            "format": "quaternion",
            "coordinate_system": "world_frame",
            "accuracy": "subpixel"
        }
    }

    return annotation_config
```

### 2. Domain Randomization for Synthetic Data
```python
# environments/domain_randomization.py
import random
import numpy as np

def get_domain_randomization_config():
    """
    Get domain randomization configuration for synthetic-to-real transfer
    """
    dr_config = {
        "textures": {
            "material_types": ["wood", "metal", "plastic", "concrete", "fabric"],
            "color_ranges": {
                "hue": [0, 360],
                "saturation": [0.2, 1.0],
                "value": [0.3, 1.0]
            },
            "roughness_range": [0.1, 0.9],
            "metallic_range": [0.0, 1.0],
            "normal_map_intensity": [0.5, 2.0]
        },
        "lighting": {
            "intensity_range": [1000, 8000],
            "color_temperature_range": [3000, 8000],  # Kelvin
            "light_positions": {
                "x_range": [-10, 10],
                "y_range": [-10, 10],
                "z_range": [2, 15]
            },
            "shadow_softness_range": [0.1, 1.0]
        },
        "physics": {
            "friction_range": [0.1, 0.9],
            "restitution_range": [0.0, 0.5],
            "mass_range": [0.5, 10.0]
        },
        "camera": {
            "fov_range": [45, 120],
            "position_variance": [0.05, 0.05, 0.05],
            "rotation_variance": [2, 2, 2],
            "noise_level": [0.001, 0.01]
        },
        "environment": {
            "object_placement_variance": 0.1,
            "background_complexity": [0.1, 0.9],
            "clutter_density": [0.0, 0.8]
        }
    }

    return dr_config

def apply_domain_randomization(scene_config, dr_config, randomization_factor=1.0):
    """
    Apply domain randomization to a scene configuration
    """
    # Randomize textures
    for obj in scene_config.get("objects", []):
        if random.random() < randomization_factor:
            # Apply texture randomization
            obj["material"]["base_color"] = [
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0)
            ]
            obj["material"]["roughness"] = random.uniform(
                dr_config["textures"]["roughness_range"][0],
                dr_config["textures"]["roughness_range"][1]
            )

    # Randomize lighting
    if random.random() < randomization_factor:
        scene_config["lighting"]["intensity"] = random.uniform(
            dr_config["lighting"]["intensity_range"][0],
            dr_config["lighting"]["intensity_range"][1]
        )
        scene_config["lighting"]["color"] = [
            random.uniform(0.7, 1.0),
            random.uniform(0.7, 1.0),
            random.uniform(0.7, 1.0)
        ]

    return scene_config
```

## Scene Testing and Validation

### 1. Performance Testing
```python
# test/performance_test.py
import time
import statistics

def test_rendering_performance(scene_name, quality_settings):
    """
    Test rendering performance for different quality settings
    """
    frame_times = []
    target_fps = quality_settings.get("frame_rate", 30)

    # Simulate rendering frames
    for i in range(100):  # Test with 100 frames
        start_time = time.time()

        # Simulate frame rendering
        # In real implementation, this would render a frame
        time.sleep(1.0 / (target_fps * 1.5))  # Simulate rendering time

        frame_time = time.time() - start_time
        frame_times.append(frame_time)

    avg_frame_time = statistics.mean(frame_times)
    avg_fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

    results = {
        "scene": scene_name,
        "quality_setting": quality_settings.get("renderer", "default"),
        "avg_frame_time_ms": avg_frame_time * 1000,
        "avg_fps": avg_fps,
        "target_fps": target_fps,
        "performance_ratio": avg_fps / target_fps if target_fps > 0 else 0
    }

    return results

def validate_scene_realism(scene_config):
    """
    Validate that the scene has realistic properties
    """
    issues = []

    # Check lighting realism
    lighting = scene_config.get("lighting", {})
    intensity = lighting.get("intensity", 0)
    if intensity < 500 or intensity > 20000:
        issues.append(f"Lighting intensity {intensity} may be unrealistic (recommended: 1000-8000)")

    # Check material properties
    materials = scene_config.get("materials", {})
    for mat_name, mat_props in materials.items():
        roughness = mat_props.get("roughness", 0)
        if roughness < 0 or roughness > 1:
            issues.append(f"Material {mat_name} roughness {roughness} out of range [0,1]")

        metallic = mat_props.get("metallic", 0)
        if metallic < 0 or metallic > 1:
            issues.append(f"Material {mat_name} metallic {metallic} out of range [0,1]")

    # Check object scales
    objects = scene_config.get("objects", [])
    for obj in objects:
        if obj.get("type") == "box":
            dims = obj.get("dimensions", [1, 1, 1])
            if any(dim > 100 for dim in dims):
                issues.append(f"Object {obj.get('name', 'unnamed')} has unusually large dimensions {dims}")

    return {
        "is_valid": len(issues) == 0,
        "issues": issues,
        "scene": scene_config.get("name", "unnamed")
    }
```

### 2. Synthetic Data Quality Validation
```python
# test/synthetic_data_validation.py
def validate_synthetic_data_quality(images, annotations):
    """
    Validate the quality of synthetic data
    """
    validation_results = {
        "image_quality": {},
        "annotation_accuracy": {},
        "realism_metrics": {}
    }

    # Image quality checks
    for img in images:
        # Check image sharpness, contrast, color balance
        # Verify no rendering artifacts
        # Check for proper exposure
        pass

    # Annotation accuracy
    for ann in annotations:
        # Verify bounding boxes are within image bounds
        # Check segmentation masks are properly formatted
        # Validate pose estimates are reasonable
        pass

    # Realism metrics
    # Compare synthetic data statistics to real-world data when available

    return validation_results
```

## Troubleshooting

### Common Issues

#### 1. Performance Problems
- **Low FPS**: Reduce texture resolution, disable complexion, lower AA samples
- **High Memory Usage**: Reduce texture sizes, limit scene complexity, use LOD
- **Rendering Artifacts**: Check material properties, lighting setup, camera settings

#### 2. Realism Issues
- **Unrealistic Lighting**: Verify HDR textures, proper intensity values, shadow settings
- **Material Issues**: Check PBR properties, normal maps, roughness/metallic values
- **Physics Problems**: Verify collision geometries, mass properties, friction values

## Next Steps
After setting up photorealistic environments:
1. Implement synthetic data generation pipeline (Task 2.2)
2. Test perception pipeline integration (Task 2.3)
3. Configure navigation system (Task 2.4)
4. Implement domain randomization (Task 3.1)

## Resources
- [Isaac Sim Advanced Rendering](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_advanced_rendering.html)
- [USD Scene Description](https://graphics.pixar.com/usd/release/docs/index.html)
- [PBR Materials Guide](https://docs.omniverse.nvidia.com/kit/docs/materials/latest/index.html)
- [Synthetic Data Generation Best Practices](https://developer.nvidia.com/blog/generating-synthetic-data-with-isaac-sim/)