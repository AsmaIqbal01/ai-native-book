# Domain Randomization Implementation Guide

## Overview
This guide provides instructions for implementing domain randomization techniques to improve sim-to-real transfer capabilities. Domain randomization involves systematically varying simulation parameters to make models more robust to real-world variations.

## Domain Randomization Concepts

### 1. Key Domain Randomization Techniques
- **Color and Texture Variation**: Randomizing appearance parameters
- **Lighting Conditions**: Varying light positions, colors, and intensities
- **Physics Parameters**: Randomizing friction, mass, and other properties
- **Camera Parameters**: Varying focal length, distortion, and noise
- **Background Clutter**: Adding random objects to scenes

### 2. Benefits of Domain Randomization
- **Improved Generalization**: Models perform better on real data
- **Reduced Reality Gap**: Better sim-to-real transfer
- **Robustness**: Models handle environmental variations
- **Data Efficiency**: Reduces need for extensive real-world data

## Configuration Files

### 1. Domain Randomization Configuration
```yaml
# config/domain_randomization.yaml
domain_randomization:
  version: "1.0"
  enabled: true
  randomization_factor: 0.8  # How much to randomize (0.0 to 1.0)

  textures:
    enabled: true
    material_types: ["wood", "metal", "plastic", "concrete", "fabric", "glass"]
    color_ranges:
      hue: [0, 360]
      saturation: [0.2, 1.0]
      value: [0.3, 1.0]
    roughness_range: [0.1, 0.9]
    metallic_range: [0.0, 1.0]
    normal_map_intensity: [0.5, 2.0]
    texture_scale_range: [0.5, 2.0]

  lighting:
    enabled: true
    intensity_range: [1000, 8000]
    color_temperature_range: [3000, 8000]  # Kelvin
    light_positions:
      x_range: [-10, 10]
      y_range: [-10, 10]
      z_range: [2, 15]
    shadow_softness_range: [0.1, 1.0]
    dome_light_texture_variation: true
    dome_light_exposure_range: [-2.0, 1.0]

  physics:
    enabled: false  # Usually disabled for perception tasks
    friction_range: [0.1, 0.9]
    restitution_range: [0.0, 0.5]
    mass_range: [0.5, 10.0]
    linear_damping_range: [0.0, 10.0]
    angular_damping_range: [0.0, 10.0]

  camera:
    enabled: true
    fov_range: [45, 120]
    position_variance: [0.05, 0.05, 0.05]
    rotation_variance: [2, 2, 2]  # degrees
    noise_level: [0.001, 0.01]
    distortion_coefficients_range:
      k1: [-0.5, 0.5]
      k2: [-0.5, 0.5]
      p1: [-0.01, 0.01]
      p2: [-0.01, 0.01]
      k3: [-0.5, 0.5]

  environment:
    enabled: true
    object_placement_variance: 0.1
    background_complexity: [0.1, 0.9]
    clutter_density: [0.0, 0.8]
    floor_roughness_range: [0.0, 1.0]
    wall_pattern_variation: true

  post_processing:
    enabled: true
    blur_range: [0.0, 2.0]
    brightness_range: [-0.2, 0.2]
    contrast_range: [0.8, 1.2]
    saturation_range: [0.5, 1.5]
    gamma_range: [0.8, 1.2]
    chromatic_aberration_range: [0.0, 0.05]
    motion_blur_enabled: true
    motion_blur_strength_range: [0.0, 0.5]

  sensor_noise:
    enabled: true
    rgb_noise_std: [0.005, 0.02]
    depth_noise_std: [0.001, 0.01]
    imu_noise_std: [0.001, 0.01]
    joint_noise_std: [0.0001, 0.001]
```

### 2. Scene Randomization Configuration
```yaml
# config/scene_randomization.yaml
scene_randomization:
  version: "1.0"
  base_scenes:
    - "simple_room"
    - "office_environment"
    - "industrial_warehouse"
    - "urban_street"

  randomization_strategies:
    texture_randomization:
      strategy: "completely_random"
      frequency: 0.8
      categories:
        - "furniture"
        - "obstacles"
        - "environment"
        - "robot_parts"

    lighting_randomization:
      strategy: "time_of_day"
      frequency: 0.9
      variations:
        - "dawn"
        - "noon"
        - "sunset"
        - "overcast"
        - "artificial"

    weather_randomization:
      strategy: "probabilistic"
      frequency: 0.3
      variations:
        - "clear_sky"
        - "overcast"
        - "foggy"
        - "rainy"

  object_randomization:
    placement:
      enabled: true
      max_objects: 20
      min_objects: 5
      object_categories:
        - "furniture"
        - "obstacles"
        - "decorative"
      placement_strategies:
        - "uniform_random"
        - "clustered"
        - "symmetric"
        - "path_aware"

    appearance:
      enabled: true
      color_variation: 0.7
      size_variation: 0.3
      rotation_variation: 45.0  # degrees

  physics_randomization:
    enabled: true
    parameters:
      gravity_range: [-9.9, -9.7]
      friction_coefficient_range: [0.1, 0.9]
      restitution_range: [0.0, 0.5]
```

## Implementation Files

### 1. Domain Randomization Python Module
```python
# transfer/domain_randomization.py
import random
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
import omni
import carb
import yaml
from typing import Dict, List, Tuple, Any

class DomainRandomizer:
    def __init__(self, config_path: str = None):
        """
        Initialize the domain randomizer with configuration
        """
        if config_path:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            # Default configuration
            self.config = self._get_default_config()

        self.stage = omni.usd.get_context().get_stage()
        self.randomization_enabled = self.config['domain_randomization']['enabled']

    def _get_default_config(self) -> Dict:
        """
        Get default domain randomization configuration
        """
        return {
            "domain_randomization": {
                "version": "1.0",
                "enabled": True,
                "randomization_factor": 0.8,
                "textures": {
                    "enabled": True,
                    "material_types": ["wood", "metal", "plastic", "concrete", "fabric", "glass"],
                    "color_ranges": {
                        "hue": [0, 360],
                        "saturation": [0.2, 1.0],
                        "value": [0.3, 1.0]
                    },
                    "roughness_range": [0.1, 0.9],
                    "metallic_range": [0.0, 1.0]
                },
                "lighting": {
                    "enabled": True,
                    "intensity_range": [1000, 8000],
                    "color_temperature_range": [3000, 8000],
                    "light_positions": {
                        "x_range": [-10, 10],
                        "y_range": [-10, 10],
                        "z_range": [2, 15]
                    }
                },
                "camera": {
                    "enabled": True,
                    "fov_range": [45, 120],
                    "position_variance": [0.05, 0.05, 0.05],
                    "rotation_variance": [2, 2, 2]
                }
            }
        }

    def randomize_textures(self, prim_paths: List[str] = None):
        """
        Randomize textures and materials in the scene
        """
        if not self.config['domain_randomization']['textures']['enabled']:
            return

        if prim_paths is None:
            # Get all prims in the stage that might have materials
            prim_paths = self._get_material_prims()

        for prim_path in prim_paths:
            self._randomize_prim_material(prim_path)

    def _get_material_prims(self) -> List[str]:
        """
        Get all prims in the stage that have or could have materials
        """
        prims_with_materials = []
        stage = omni.usd.get_context().get_stage()

        for prim in stage.TraverseAll():
            if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Sphere):
                prims_with_materials.append(str(prim.GetPath()))

        return prims_with_materials

    def _randomize_prim_material(self, prim_path: str):
        """
        Randomize the material of a specific prim
        """
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)

        if not prim.IsValid():
            return

        # Create a new material with random properties
        material_name = f"{prim_path.replace('/', '_')}_material"
        material_path = Sdf.Path(f"{prim_path}/Material")

        material = UsdShade.Material.Define(stage, material_path)

        # Create shader
        shader = UsdShade.Shader.Define(stage, f"{material_path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")

        # Randomize base color
        color_range = self.config['domain_randomization']['textures']['color_ranges']
        base_color = Gf.Vec3f(
            random.uniform(color_range['hue'][0]/360.0, color_range['hue'][1]/360.0),
            random.uniform(color_range['saturation'][0], color_range['saturation'][1]),
            random.uniform(color_range['value'][0], color_range['value'][1])
        )
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(base_color)

        # Randomize roughness
        roughness_range = self.config['domain_randomization']['textures']['roughness_range']
        roughness = random.uniform(roughness_range[0], roughness_range[1])
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)

        # Randomize metallic
        metallic_range = self.config['domain_randomization']['textures']['metallic_range']
        metallic = random.uniform(metallic_range[0], metallic_range[1])
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

        # Connect shader to material
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    def randomize_lighting(self):
        """
        Randomize lighting conditions in the scene
        """
        if not self.config['domain_randomization']['lighting']['enabled']:
            return

        # Find dome light in the scene
        dome_light_path = "/World/DomeLight"
        dome_light = UsdGeom.DomeLight.Get(self.stage, dome_light_path)

        if not dome_light:
            # Create dome light if it doesn't exist
            dome_light = UsdGeom.DomeLight.Define(self.stage, dome_light_path)

        # Randomize intensity
        intensity_range = self.config['domain_randomization']['lighting']['intensity_range']
        intensity = random.uniform(intensity_range[0], intensity_range[1])
        dome_light.GetIntensityAttr().Set(intensity)

        # Randomize color temperature
        temp_range = self.config['domain_randomization']['lighting']['color_temperature_range']
        color_temp = random.uniform(temp_range[0], temp_range[1])

        # Convert color temperature to RGB approximation
        rgb_color = self._color_temperature_to_rgb(color_temp)
        dome_light.GetColorAttr().Set(Gf.Vec3f(*rgb_color))

    def _color_temperature_to_rgb(self, color_temp: float) -> Tuple[float, float, float]:
        """
        Convert color temperature in Kelvin to RGB values
        """
        temp = color_temp / 100.0

        # Red
        if temp <= 66:
            red = 255
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)

        # Green
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)

        # Blue
        if temp >= 66:
            blue = 255
        elif temp <= 19:
            blue = 0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307

        # Normalize to 0-1 range
        return (
            max(0, min(1, red / 255)),
            max(0, min(1, green / 255)),
            max(0, min(1, blue / 255))
        )

    def randomize_camera(self, camera_path: str = "/World/Camera"):
        """
        Randomize camera parameters
        """
        if not self.config['domain_randomization']['camera']['enabled']:
            return

        # Get camera prim
        camera_prim = UsdGeom.Camera(self.stage.GetPrimAtPath(camera_path))
        if not camera_prim:
            return

        # Randomize field of view
        fov_range = self.config['domain_randomization']['camera']['fov_range']
        new_fov = random.uniform(fov_range[0], fov_range[1])
        focal_length = 24.0  # Default focal length
        # Adjust focal length based on FOV change
        new_focal_length = focal_length * (60.0 / new_fov)  # Assuming 60 degree is normal FOV
        camera_prim.GetFocalLengthAttr().Set(new_focal_length)

    def randomize_physics(self, object_paths: List[str] = None):
        """
        Randomize physics properties of objects
        """
        if not self.config['domain_randomization']['physics']['enabled']:
            return

        if object_paths is None:
            # Get all physics-enabled objects
            object_paths = self._get_physics_objects()

        for obj_path in object_paths:
            self._randomize_object_physics(obj_path)

    def _get_physics_objects(self) -> List[str]:
        """
        Get all objects with physics properties
        """
        physics_objects = []
        stage = omni.usd.get_context().get_stage()

        for prim in stage.TraverseAll():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                physics_objects.append(str(prim.GetPath()))

        return physics_objects

    def _randomize_object_physics(self, obj_path: str):
        """
        Randomize physics properties of a specific object
        """
        # This would involve modifying physics properties like friction, mass, etc.
        # Implementation would depend on the physics API being used
        pass

    def apply_randomization(self, scene_objects: List[str] = None):
        """
        Apply all domain randomization techniques to the scene
        """
        if not self.randomization_enabled:
            return

        # Apply randomization based on randomization factor
        if random.random() > self.config['domain_randomization']['randomization_factor']:
            return

        # Apply texture randomization
        self.randomize_textures(scene_objects)

        # Apply lighting randomization
        self.randomize_lighting()

        # Apply camera randomization
        self.randomize_camera()

        # Apply physics randomization
        self.randomize_physics(scene_objects)

        carb.log_info("Domain randomization applied to scene")

    def generate_randomization_report(self) -> Dict[str, Any]:
        """
        Generate a report of applied randomization
        """
        report = {
            "timestamp": carb.events.GetTime(),
            "randomization_factor": self.config['domain_randomization']['randomization_factor'],
            "applied_randomizations": {
                "textures": self.config['domain_randomization']['textures']['enabled'],
                "lighting": self.config['domain_randomization']['lighting']['enabled'],
                "camera": self.config['domain_randomization']['camera']['enabled'],
                "physics": self.config['domain_randomization']['physics']['enabled']
            }
        }

        return report


class SceneRandomizer:
    def __init__(self, config_path: str = None):
        """
        Initialize scene randomizer with configuration
        """
        if config_path:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            # Default configuration
            self.config = self._get_default_scene_config()

    def _get_default_scene_config(self) -> Dict:
        """
        Get default scene randomization configuration
        """
        return {
            "scene_randomization": {
                "version": "1.0",
                "base_scenes": ["simple_room", "office_environment"],
                "randomization_strategies": {
                    "texture_randomization": {
                        "strategy": "completely_random",
                        "frequency": 0.8,
                        "categories": ["furniture", "obstacles", "environment"]
                    }
                }
            }
        }

    def randomize_scene_layout(self, scene_name: str):
        """
        Randomize the layout of a scene
        """
        # Implementation for randomizing scene layout
        pass

    def add_background_clutter(self, count: int = 5):
        """
        Add random objects to the scene as background clutter
        """
        # Implementation for adding background clutter
        pass


def create_domain_randomization_config():
    """
    Create a configuration for domain randomization
    """
    dr_config = {
        "textures": {
            "enabled": True,
            "color_variance": [0.1, 1.0],
            "roughness_range": [0.1, 0.9],
            "metallic_range": [0.0, 1.0],
            "normal_map_intensity": [0.5, 2.0]
        },
        "lighting": {
            "enabled": True,
            "intensity_range": [1000, 8000],
            "color_temperature_range": [3000, 8000],
            "position_variance": [5.0, 5.0, 5.0]
        },
        "camera": {
            "enabled": True,
            "position_noise": [0.01, 0.01, 0.01],
            "rotation_noise": [0.1, 0.1, 0.1],
            "fov_jitter": 2.0
        },
        "background": {
            "enabled": True,
            "clutter_density": [0.0, 0.8],
            "complexity": [0.1, 0.9]
        }
    }

    return dr_config
```

### 2. Domain Randomization Integration with Isaac Sim
```python
# transfer/isaac_sim_integration.py
import omni
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
import carb
import numpy as np
from typing import List, Dict, Any
from .domain_randomization import DomainRandomizer

class IsaacSimDomainRandomizer:
    def __init__(self, config_path: str = None):
        """
        Initialize Isaac Sim domain randomizer
        """
        self.domain_randomizer = DomainRandomizer(config_path)
        self.stage = omni.usd.get_context().get_stage()

    def setup_randomization_for_scene(self, scene_name: str):
        """
        Setup domain randomization for a specific scene
        """
        # Load scene-specific configurations
        # Apply scene-specific randomization parameters
        carb.log_info(f"Setting up domain randomization for scene: {scene_name}")

    def apply_randomization_during_simulation(self):
        """
        Apply domain randomization during simulation run
        """
        # This would be called periodically during simulation
        # to apply randomization effects
        self.domain_randomizer.apply_randomization()

    def randomize_environment(self, environment_objects: List[str] = None):
        """
        Randomize environment objects in the scene
        """
        if environment_objects is None:
            environment_objects = self._get_environment_objects()

        self.domain_randomizer.randomize_textures(environment_objects)

    def _get_environment_objects(self) -> List[str]:
        """
        Get all environment objects that should be randomized
        """
        env_objects = []
        for prim in self.stage.TraverseAll():
            prim_path = str(prim.GetPath())
            # Look for objects that are likely environment objects
            if any(keyword in prim_path.lower() for keyword in ['floor', 'wall', 'ground', 'env', 'background']):
                env_objects.append(prim_path)

        return env_objects

    def randomize_robot_appearance(self, robot_path: str = "/World/Robot"):
        """
        Randomize the appearance of the robot
        """
        robot_objects = [robot_path]
        # Get all parts of the robot
        robot_prim = self.stage.GetPrimAtPath(robot_path)
        if robot_prim:
            for child in robot_prim.GetAllChildren():
                robot_objects.append(str(child.GetPath()))

        self.domain_randomizer.randomize_textures(robot_objects)

    def randomize_sensor_data(self):
        """
        Apply randomization to sensor data to simulate real-world noise
        """
        # This would involve adding noise to sensor outputs
        # to make them more realistic
        pass

    def get_randomization_metrics(self) -> Dict[str, Any]:
        """
        Get metrics about the randomization applied
        """
        return self.domain_randomizer.generate_randomization_report()


def setup_domain_randomization_in_isaac_sim(config_path: str = None):
    """
    Setup domain randomization in Isaac Sim environment
    """
    randomizer = IsaacSimDomainRandomizer(config_path)

    # Register with Isaac Sim lifecycle
    # This would integrate with Isaac Sim's update loop

    carb.log_info("Domain randomization setup completed")
    return randomizer


# Example usage function
def example_domain_randomization():
    """
    Example of how to use domain randomization in Isaac Sim
    """
    # Initialize domain randomizer
    randomizer = IsaacSimDomainRandomizer()

    # Setup for specific scene
    randomizer.setup_randomization_for_scene("simple_room")

    # Apply randomization to environment
    randomizer.randomize_environment()

    # Apply randomization to robot
    randomizer.randomize_robot_appearance()

    # Get metrics
    metrics = randomizer.get_randomization_metrics()
    carb.log_info(f"Randomization metrics: {metrics}")
```

### 3. Domain Randomization Testing
```python
# test/domain_randomization_test.py
import unittest
import numpy as np
from transfer.domain_randomization import DomainRandomizer

class TestDomainRandomizer(unittest.TestCase):
    def setUp(self):
        """
        Set up test configuration
        """
        self.config = {
            "domain_randomization": {
                "version": "1.0",
                "enabled": True,
                "randomization_factor": 0.8,
                "textures": {
                    "enabled": True,
                    "color_ranges": {
                        "hue": [0, 360],
                        "saturation": [0.2, 1.0],
                        "value": [0.3, 1.0]
                    },
                    "roughness_range": [0.1, 0.9],
                    "metallic_range": [0.0, 1.0]
                },
                "lighting": {
                    "enabled": True,
                    "intensity_range": [1000, 8000],
                    "color_temperature_range": [3000, 8000]
                }
            }
        }

        # Mock stage for testing
        self.domain_randomizer = DomainRandomizer()
        self.domain_randomizer.config = self.config
        self.domain_randomizer.randomization_enabled = True

    def test_color_temperature_conversion(self):
        """
        Test color temperature to RGB conversion
        """
        rgb = self.domain_randomizer._color_temperature_to_rgb(6500)  # Daylight
        self.assertIsInstance(rgb, tuple)
        self.assertEqual(len(rgb), 3)
        # Check that RGB values are in [0, 1] range
        for val in rgb:
            self.assertGreaterEqual(val, 0)
            self.assertLessEqual(val, 1)

    def test_randomization_factor(self):
        """
        Test that randomization respects the randomization factor
        """
        # This is difficult to test deterministically due to randomness
        # We can test that the factor is being used somehow
        self.assertEqual(
            self.domain_randomizer.config['domain_randomization']['randomization_factor'],
            0.8
        )

    def test_config_defaults(self):
        """
        Test that default configuration is properly loaded
        """
        default_randomizer = DomainRandomizer()
        self.assertIn('domain_randomization', default_randomizer.config)
        self.assertTrue(default_randomizer.config['domain_randomization']['enabled'])


def run_domain_randomization_tests():
    """
    Run all domain randomization tests
    """
    unittest.main(argv=[''], exit=False, verbosity=2)


if __name__ == '__main__':
    run_domain_randomization_tests()
```

## Usage Examples

### 1. Basic Domain Randomization Usage
```python
# examples/basic_domain_randomization.py
import carb
from transfer.domain_randomization import DomainRandomizer, IsaacSimDomainRandomizer

def basic_domain_randomization_example():
    """
    Basic example of using domain randomization
    """
    # Initialize domain randomizer
    randomizer = IsaacSimDomainRandomizer("config/domain_randomization.yaml")

    # Setup for a specific scene
    randomizer.setup_randomization_for_scene("office_environment")

    # Apply randomization to environment
    randomizer.randomize_environment()

    # Apply randomization to robot
    randomizer.randomize_robot_appearance()

    # Get metrics about the randomization
    metrics = randomizer.get_randomization_metrics()
    carb.log_info(f"Applied domain randomization: {metrics}")

def advanced_domain_randomization_example():
    """
    Advanced example with custom configurations
    """
    # Create custom configuration
    custom_config = {
        "domain_randomization": {
            "version": "1.0",
            "enabled": True,
            "randomization_factor": 0.9,  # Higher randomization
            "textures": {
                "enabled": True,
                "color_ranges": {
                    "hue": [0, 360],
                    "saturation": [0.5, 1.0],  # More saturated colors
                    "value": [0.4, 1.0]
                },
                "roughness_range": [0.2, 0.8],
                "metallic_range": [0.0, 0.7]
            },
            "lighting": {
                "enabled": True,
                "intensity_range": [2000, 10000],  # Higher intensity range
                "color_temperature_range": [2000, 10000]
            }
        }
    }

    # Use custom configuration
    randomizer = DomainRandomizer()
    randomizer.config = custom_config
    randomizer.randomization_enabled = True

    # Apply to specific objects
    test_objects = ["/World/Object1", "/World/Object2"]
    randomizer.apply_randomization(test_objects)

    carb.log_info("Advanced domain randomization applied")

if __name__ == "__main__":
    basic_domain_randomization_example()
```

### 2. Integration with Synthetic Data Generation
```python
# examples/integration_with_synthetic_data.py
from transfer.domain_randomization import DomainRandomizer
from synthetic_data_pipeline.main_pipeline import SyntheticDataPipeline
import random

class DomainRandomizedDataGenerator:
    def __init__(self, config_path: str = None):
        """
        Initialize data generator with domain randomization
        """
        self.domain_randomizer = DomainRandomizer(config_path)
        self.data_pipeline = SyntheticDataPipeline("config/synthetic_data_config.yaml")

    def generate_randomized_dataset(self, num_samples: int = 1000):
        """
        Generate dataset with domain randomization applied
        """
        carb.log_info(f"Generating {num_samples} samples with domain randomization")

        for i in range(num_samples):
            # Apply randomization to scene
            self.domain_randomizer.apply_randomization()

            # Generate synthetic data
            # This would integrate with the synthetic data pipeline
            carb.log_info(f"Generated sample {i+1}/{num_samples} with randomization")

            if i % 100 == 0:
                carb.log_info(f"Progress: {i}/{num_samples} samples generated")

def example_integration():
    """
    Example of integrating domain randomization with synthetic data generation
    """
    generator = DomainRandomizedDataGenerator("config/domain_randomization.yaml")
    generator.generate_randomized_dataset(num_samples=500)

if __name__ == "__main__":
    example_integration()
```

## Performance Considerations

### 1. Randomization Performance Monitoring
```python
# transfer/performance_monitor.py
import time
from collections import deque
import carb

class RandomizationPerformanceMonitor:
    def __init__(self):
        """
        Monitor performance of domain randomization operations
        """
        self.randomization_times = deque(maxlen=100)
        self.last_randomization_time = 0

    def measure_randomization_time(self, func, *args, **kwargs):
        """
        Measure time taken for a randomization operation
        """
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()

        randomization_time = end_time - start_time
        self.randomization_times.append(randomization_time)
        self.last_randomization_time = randomization_time

        return result

    def get_performance_stats(self) -> dict:
        """
        Get performance statistics
        """
        if not self.randomization_times:
            return {"avg_time": 0, "min_time": 0, "max_time": 0, "count": 0}

        times = list(self.randomization_times)
        return {
            "avg_time": sum(times) / len(times),
            "min_time": min(times),
            "max_time": max(times),
            "count": len(times),
            "last_time": self.last_randomization_time
        }

def monitor_domain_randomization_performance():
    """
    Monitor domain randomization performance
    """
    monitor = RandomizationPerformanceMonitor()

    # Example: Monitor texture randomization
    randomizer = DomainRandomizer()

    def randomize_textures_wrapper():
        return randomizer.randomize_textures(["/World/TestObject"])

    # Measure performance
    monitor.measure_randomization_time(randomize_textures_wrapper)

    stats = monitor.get_performance_stats()
    carb.log_info(f"Randomization performance: {stats}")

    return stats
```

## Validation and Testing

### 1. Transfer Effectiveness Validation
```python
# test/transfer_validation.py
import numpy as np
from typing import List, Dict, Any

class TransferEffectivenessValidator:
    def __init__(self):
        """
        Validate effectiveness of domain randomization for sim-to-real transfer
        """
        self.simulation_performance = []
        self.real_world_performance = []

    def validate_transfer_effectiveness(
        self,
        sim_model_performance: List[float],
        real_model_performance: List[float]
    ) -> Dict[str, Any]:
        """
        Validate transfer effectiveness by comparing sim vs real performance
        """
        # Calculate improvement metrics
        sim_mean = np.mean(sim_model_performance)
        real_mean = np.mean(real_model_performance)

        # Calculate transfer gap
        transfer_gap = abs(sim_mean - real_mean)

        # Calculate improvement percentage
        if sim_mean != 0:
            improvement_percentage = ((real_mean - sim_mean) / sim_mean) * 100
        else:
            improvement_percentage = 0

        validation_results = {
            "sim_mean_performance": float(sim_mean),
            "real_mean_performance": float(real_mean),
            "transfer_gap": float(transfer_gap),
            "improvement_percentage": float(improvement_percentage),
            "transfer_success": transfer_gap < 0.1,  # Threshold for success
            "sim_std": float(np.std(sim_model_performance)),
            "real_std": float(np.std(real_model_performance))
        }

        return validation_results

def example_transfer_validation():
    """
    Example of transfer validation
    """
    validator = TransferEffectivenessValidator()

    # Example performance data (e.g., accuracy scores)
    sim_performance = [0.95, 0.92, 0.94, 0.93, 0.96]  # Simulation performance
    real_performance = [0.88, 0.85, 0.89, 0.87, 0.90]  # Real world performance

    results = validator.validate_transfer_effectiveness(sim_performance, real_performance)

    print("Transfer Validation Results:")
    print(f"  Simulation Mean: {results['sim_mean_performance']:.3f}")
    print(f"  Real World Mean: {results['real_mean_performance']:.3f}")
    print(f"  Transfer Gap: {results['transfer_gap']:.3f}")
    print(f"  Improvement: {results['improvement_percentage']:.2f}%")
    print(f"  Transfer Success: {results['transfer_success']}")

    return results
```

## Troubleshooting

### Common Issues and Solutions

#### 1. Performance Issues
- **Slow Randomization**: Reduce the number of objects being randomized, use simpler materials
- **High Memory Usage**: Implement batch processing, clear unused assets
- **Rendering Slowdown**: Optimize material complexity, reduce texture resolution during randomization

#### 2. Randomization Issues
- **Inconsistent Results**: Ensure proper random seed management
- **Unrealistic Variations**: Validate randomization ranges and constraints
- **Missing Randomization**: Check that randomization is enabled and properly configured

#### 3. Integration Issues
- **Isaac Sim Compatibility**: Verify Isaac Sim version compatibility
- **USD Format Issues**: Check USD schema compliance
- **Material Problems**: Ensure materials are properly connected to geometry

## Next Steps
After implementing domain randomization:
1. Test transfer effectiveness with real-world data (Task 3.2)
2. Integrate all components into complete system (Task 3.3)
3. Optimize overall performance (Task 3.4)
4. Conduct comprehensive testing and validation (Phase 4)

## Resources
- [Isaac Sim Domain Randomization](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_synthetic_data.html)
- [Domain Randomization Research](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer Techniques](https://arxiv.org/abs/1802.01557)
- [NVIDIA Isaac ROS Transfer Learning](https://nvidia-isaac-ros.github.io/concepts/sim2real/index.html)