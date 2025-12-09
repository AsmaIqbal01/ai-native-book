# Synthetic Data Generation Pipeline Guide

## Overview
This guide provides instructions for creating tools and pipelines for generating synthetic training data using Isaac Sim. The synthetic data will be used for training perception models that can transfer to real-world applications.

## Data Generation Architecture

### 1. Pipeline Components
- **Scene Generator**: Creates varied environments with different configurations
- **Domain Randomizer**: Applies randomization to textures, lighting, and physics
- **Data Capture System**: Captures images, depth, segmentation, and annotations
- **Annotation Engine**: Automatically generates ground truth labels
- **Data Storage**: Organizes and stores the generated datasets

### 2. Data Types Generated
- **RGB Images**: Color images for object detection and classification
- **Depth Maps**: Depth information for 3D understanding
- **Semantic Segmentation**: Pixel-level class labels
- **Instance Segmentation**: Object instance boundaries
- **Pose Data**: 6D pose information for objects
- **Bounding Boxes**: 2D bounding box annotations
- **Point Clouds**: 3D point cloud data

## Configuration Files

### 1. Data Generation Configuration
```yaml
# config/synthetic_data_config.yaml
synthetic_data_generation:
  pipeline:
    name: "isaac_synthetic_data_pipeline"
    version: "1.0"
    enabled: true
    parallel_generation: true
    max_workers: 4
    batch_size: 32
    output_format: "coco"

  scenes:
    base_scenes:
      - "simple_room"
      - "office_environment"
      - "industrial_warehouse"
      - "urban_street"
    scene_variation_count: 100  # Number of variations per base scene
    randomization_factor: 0.8   # How much to randomize each scene

  cameras:
    primary_camera:
      name: "rgb_camera"
      position: [0.1, 0, 1.2]
      fov: 60
      resolution: [1920, 1080]
      format: "RGB8"
      frame_rate: 10
    depth_camera:
      name: "depth_camera"
      position: [0.1, 0, 1.2]
      fov: 60
      resolution: [1920, 1080]
      format: "32FC1"
      frame_rate: 10
    segmentation_camera:
      name: "seg_camera"
      position: [0.1, 0, 1.2]
      fov: 60
      resolution: [1920, 1080]
      format: "RGB8"
      frame_rate: 10

  annotation:
    object_detection: true
    semantic_segmentation: true
    instance_segmentation: true
    pose_estimation: true
    depth_maps: true
    format: "coco"
    save_visualization: true

  domain_randomization:
    textures: true
    lighting: true
    physics: false  # Usually not randomized for perception tasks
    camera_noise: true
    background_clutter: true

  output:
    base_directory: "./datasets/synthetic_data"
    dataset_name: "isaac_synthetic_perception_dataset"
    max_samples_per_class: 10000
    train_val_test_split: [0.7, 0.2, 0.1]
    compression: "lz4"
    validation_enabled: true
```

### 2. Class Definition Configuration
```yaml
# config/data_classes.yaml
classes:
  robot_related:
    humanoid_robot:
      id: 1
      color: [255, 0, 0]
      category: "robot"
      description: "Simple humanoid robot model"
    robot_arm:
      id: 2
      color: [255, 100, 0]
      category: "robot"
      description: "Robot arm components"

  furniture:
    chair:
      id: 3
      color: [0, 255, 0]
      category: "furniture"
      description: "Office chair"
    desk:
      id: 4
      color: [0, 200, 0]
      category: "furniture"
      description: "Office desk"
    table:
      id: 5
      color: [0, 150, 0]
      category: "furniture"
      description: "Various tables"

  obstacles:
    box:
      id: 6
      color: [0, 0, 255]
      category: "obstacle"
      description: "Rectangular boxes"
    cylinder:
      id: 7
      color: [100, 0, 255]
      category: "obstacle"
      description: "Cylindrical obstacles"

  environment:
    floor:
      id: 8
      color: [100, 100, 100]
      category: "environment"
      description: "Floor surfaces"
    wall:
      id: 9
      color: [200, 200, 200]
      category: "environment"
      description: "Wall surfaces"
    ceiling:
      id: 10
      color: [220, 220, 220]
      category: "environment"
      description: "Ceiling surfaces"

  humans:
    person:
      id: 11
      color: [255, 255, 0]
      category: "human"
      description: "Human figures"

  objects:
    monitor:
      id: 12
      color: [255, 0, 255]
      category: "object"
      description: "Computer monitors"
    plant:
      id: 13
      color: [0, 255, 255]
      category: "object"
      description: "Plants and vegetation"
```

## Data Generation Pipeline Implementation

### 1. Main Pipeline Controller
```python
# synthetic_data_pipeline/main_pipeline.py
import os
import json
import yaml
import random
import numpy as np
from datetime import datetime
from pathlib import Path
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

class SyntheticDataPipeline:
    def __init__(self, config_path):
        """
        Initialize the synthetic data generation pipeline
        """
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.output_dir = Path(self.config['synthetic_data_generation']['output']['base_directory'])
        self.dataset_name = self.config['synthetic_data_generation']['output']['dataset_name']
        self.scene_variations = self.config['synthetic_data_generation']['scenes']['scene_variation_count']
        self.randomization_factor = self.config['synthetic_data_generation']['scenes']['randomization_factor']

        # Create output directories
        self.setup_output_directories()

        # Load class definitions
        self.load_class_definitions()

        print(f"Synthetic Data Pipeline initialized for dataset: {self.dataset_name}")

    def setup_output_directories(self):
        """
        Create the necessary output directories
        """
        # Create base output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Create dataset-specific directories
        self.dataset_dir = self.output_dir / self.dataset_name
        self.dataset_dir.mkdir(exist_ok=True)

        # Create train/val/test splits
        for split in ['train', 'val', 'test']:
            (self.dataset_dir / split / 'images').mkdir(parents=True, exist_ok=True)
            (self.dataset_dir / split / 'labels').mkdir(parents=True, exist_ok=True)
            (self.dataset_dir / split / 'annotations').mkdir(parents=True, exist_ok=True)

        # Create visualization directory
        (self.dataset_dir / 'visualizations').mkdir(exist_ok=True)

        print(f"Output directories created at: {self.dataset_dir}")

    def load_class_definitions(self):
        """
        Load class definitions from YAML file
        """
        classes_config_path = "config/data_classes.yaml"
        if os.path.exists(classes_config_path):
            with open(classes_config_path, 'r') as f:
                self.classes = yaml.safe_load(f)
        else:
            # Default classes if config file doesn't exist
            self.classes = {
                "objects": {
                    "robot": {"id": 1, "color": [255, 0, 0], "category": "robot"},
                    "obstacle": {"id": 2, "color": [0, 255, 0], "category": "obstacle"},
                    "furniture": {"id": 3, "color": [0, 0, 255], "category": "furniture"}
                }
            }

        print(f"Loaded {len(self.classes)} class definitions")

    def generate_dataset(self, num_samples=1000):
        """
        Generate synthetic dataset with specified number of samples
        """
        print(f"Starting dataset generation for {num_samples} samples...")

        # Determine train/val/test split
        split_config = self.config['synthetic_data_generation']['output']['train_val_test_split']
        train_count = int(num_samples * split_config[0])
        val_count = int(num_samples * split_config[1])
        test_count = num_samples - train_count - val_count

        split_counts = {
            'train': train_count,
            'val': val_count,
            'test': test_count
        }

        # Generate samples for each split
        for split, count in split_counts.items():
            if count > 0:
                print(f"Generating {count} samples for {split} split...")
                self.generate_split_samples(split, count)

        # Generate COCO format annotations
        self.generate_coco_annotations()

        print("Dataset generation completed!")

    def generate_split_samples(self, split, count):
        """
        Generate samples for a specific split (train/val/test)
        """
        for i in range(count):
            # Generate a random scene variation
            scene_config = self.generate_scene_variation()

            # Capture data from the scene
            sample_data = self.capture_sample_data(scene_config)

            # Save the sample
            self.save_sample(split, i, sample_data)

            if i % 100 == 0:
                print(f"Generated {i}/{count} samples for {split} split")

    def generate_scene_variation(self):
        """
        Generate a scene variation with domain randomization
        """
        # This would interface with Isaac Sim to create a scene
        # For now, we'll simulate the process
        scene_config = {
            'scene_type': random.choice(['simple_room', 'office', 'warehouse', 'street']),
            'lighting': self.randomize_lighting(),
            'textures': self.randomize_textures(),
            'object_positions': self.randomize_object_positions(),
            'camera_config': self.randomize_camera_config()
        }

        return scene_config

    def randomize_lighting(self):
        """
        Apply domain randomization to lighting
        """
        dr_config = self.config['synthetic_data_generation']['domain_randomization']
        if not dr_config['lighting']:
            return {'intensity': 3000, 'color': [1.0, 1.0, 1.0]}

        intensity_range = [1000, 8000]
        return {
            'intensity': random.uniform(intensity_range[0], intensity_range[1]),
            'color': [
                random.uniform(0.7, 1.0),
                random.uniform(0.7, 1.0),
                random.uniform(0.7, 1.0)
            ],
            'direction': [
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(-1, 0)
            ]
        }

    def randomize_textures(self):
        """
        Apply domain randomization to textures
        """
        dr_config = self.config['synthetic_data_generation']['domain_randomization']
        if not dr_config['textures']:
            return {}

        return {
            'roughness': random.uniform(0.1, 0.9),
            'metallic': random.uniform(0.0, 1.0),
            'base_color': [
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0)
            ]
        }

    def randomize_object_positions(self):
        """
        Randomize object positions in the scene
        """
        # Generate random object positions
        objects = []
        for _ in range(random.randint(5, 15)):  # 5-15 objects per scene
            obj = {
                'class_id': random.randint(1, len(self.classes['objects'])),
                'position': [
                    random.uniform(-5, 5),
                    random.uniform(-5, 5),
                    random.uniform(0.1, 2.0)
                ],
                'rotation': [
                    random.uniform(-3.14, 3.14),
                    random.uniform(-3.14, 3.14),
                    random.uniform(-3.14, 3.14)
                ],
                'scale': random.uniform(0.5, 2.0)
            }
            objects.append(obj)

        return objects

    def randomize_camera_config(self):
        """
        Randomize camera configuration
        """
        dr_config = self.config['synthetic_data_generation']['domain_randomization']
        if not dr_config['camera_noise']:
            return {}

        return {
            'position_noise': [random.uniform(-0.01, 0.01) for _ in range(3)],
            'rotation_noise': [random.uniform(-0.05, 0.05) for _ in range(3)],
            'fov_jitter': random.uniform(-2, 2)
        }

    def capture_sample_data(self, scene_config):
        """
        Capture sample data from the scene (simulated)
        """
        # In real implementation, this would capture data from Isaac Sim
        # For simulation, we'll generate mock data

        sample_data = {
            'rgb_image': self.generate_mock_image((1920, 1080, 3)),
            'depth_map': self.generate_mock_depth((1920, 1080)),
            'segmentation': self.generate_mock_segmentation((1920, 1080)),
            'bounding_boxes': self.generate_mock_bboxes(scene_config['object_positions']),
            'poses': self.generate_mock_poses(scene_config['object_positions']),
            'scene_config': scene_config
        }

        return sample_data

    def generate_mock_image(self, shape):
        """
        Generate a mock RGB image
        """
        # Create a mock image with some basic patterns
        image = np.random.randint(0, 255, shape, dtype=np.uint8)
        # Add some basic shapes to simulate objects
        for _ in range(5):
            x, y = random.randint(100, shape[1]-100), random.randint(100, shape[0]-100)
            cv2 = __import__('cv2')
            cv2.rectangle(image, (x-50, y-50), (x+50, y+50),
                         (random.randint(0,255), random.randint(0,255), random.randint(0,255)), -1)
        return image

    def generate_mock_depth(self, shape):
        """
        Generate a mock depth map
        """
        depth = np.random.uniform(0.1, 10.0, shape).astype(np.float32)
        return depth

    def generate_mock_segmentation(self, shape):
        """
        Generate a mock segmentation map
        """
        segmentation = np.random.randint(0, len(self.classes['objects']) + 1, shape, dtype=np.uint8)
        return segmentation

    def generate_mock_bboxes(self, objects):
        """
        Generate mock bounding boxes for objects
        """
        bboxes = []
        for obj in objects:
            # Generate a mock bounding box
            x, y = obj['position'][0], obj['position'][1]
            width, height = random.uniform(50, 200), random.uniform(50, 200)
            bbox = {
                'class_id': obj['class_id'],
                'bbox': [x - width/2, y - height/2, width, height],  # x, y, width, height
                'confidence': 1.0
            }
            bboxes.append(bbox)
        return bboxes

    def generate_mock_poses(self, objects):
        """
        Generate mock pose information for objects
        """
        poses = []
        for obj in objects:
            pose = {
                'class_id': obj['class_id'],
                'position': obj['position'],
                'rotation': obj['rotation'],
                'scale': obj['scale']
            }
            poses.append(pose)
        return poses

    def save_sample(self, split, index, sample_data):
        """
        Save a sample to the appropriate directory
        """
        # Save RGB image
        image_path = self.dataset_dir / split / 'images' / f"{index:06d}.jpg"
        import cv2
        cv2.imwrite(str(image_path), cv2.cvtColor(sample_data['rgb_image'], cv2.COLOR_RGB2BGR))

        # Save depth map
        depth_path = self.dataset_dir / split / 'images' / f"{index:06d}_depth.exr"
        cv2.imwrite(str(depth_path), sample_data['depth_map'])

        # Save segmentation
        seg_path = self.dataset_dir / split / 'labels' / f"{index:06d}_seg.png"
        cv2.imwrite(str(seg_path), sample_data['segmentation'])

        # Save annotations
        annotation_path = self.dataset_dir / split / 'annotations' / f"{index:06d}.json"
        annotation_data = {
            'image_id': index,
            'split': split,
            'width': 1920,
            'height': 1080,
            'bounding_boxes': sample_data['bounding_boxes'],
            'poses': sample_data['poses'],
            'scene_config': sample_data['scene_config']
        }

        with open(annotation_path, 'w') as f:
            json.dump(annotation_data, f, indent=2)

    def generate_coco_annotations(self):
        """
        Generate COCO format annotations for the dataset
        """
        print("Generating COCO annotations...")

        coco_format = {
            "info": {
                "description": f"Synthetic dataset generated with Isaac Sim - {self.dataset_name}",
                "version": "1.0",
                "year": datetime.now().year,
                "contributor": "Isaac Sim Synthetic Data Generator",
                "date_created": datetime.now().isoformat()
            },
            "licenses": [
                {
                    "id": 1,
                    "name": "Synthetic Data License",
                    "url": "http://creativecommons.org/licenses/by/4.0/"
                }
            ],
            "categories": [],
            "images": [],
            "annotations": []
        }

        # Add categories
        cat_id = 1
        for cat_name, cat_info in self.classes['objects'].items():
            coco_format["categories"].append({
                "id": cat_info['id'],
                "name": cat_name,
                "supercategory": cat_info['category']
            })

        # Process all splits
        annotation_id = 1
        for split in ['train', 'val', 'test']:
            split_dir = self.dataset_dir / split / 'annotations'
            for ann_file in split_dir.glob("*.json"):
                with open(ann_file, 'r') as f:
                    ann_data = json.load(f)

                # Add image info
                image_info = {
                    "id": ann_data['image_id'],
                    "width": ann_data['width'],
                    "height": ann_data['height'],
                    "file_name": f"{ann_data['image_id']:06d}.jpg",
                    "license": 1,
                    "flickr_url": "",
                    "coco_url": "",
                    "date_captured": datetime.now().isoformat()
                }
                coco_format["images"].append(image_info)

                # Add annotations
                for bbox in ann_data['bounding_boxes']:
                    x, y, w, h = bbox['bbox']
                    annotation_info = {
                        "id": annotation_id,
                        "image_id": ann_data['image_id'],
                        "category_id": bbox['class_id'],
                        "bbox": [float(x), float(y), float(w), float(h)],
                        "area": float(w * h),
                        "iscrowd": 0
                    }
                    coco_format["annotations"].append(annotation_info)
                    annotation_id += 1

        # Save COCO annotations
        coco_path = self.dataset_dir / "annotations.json"
        with open(coco_path, 'w') as f:
            json.dump(coco_format, f, indent=2)

        print(f"COCO annotations saved to: {coco_path}")
        print(f"Dataset statistics:")
        print(f"  Total images: {len(coco_format['images'])}")
        print(f"  Total annotations: {len(coco_format['annotations'])}")
        print(f"  Categories: {len(coco_format['categories'])}")

def main():
    """
    Main function to run the synthetic data generation pipeline
    """
    pipeline = SyntheticDataPipeline("config/synthetic_data_config.yaml")
    pipeline.generate_dataset(num_samples=1000)  # Generate 1000 samples

if __name__ == "__main__":
    main()
```

### 2. Domain Randomization Module
```python
# synthetic_data_pipeline/domain_randomization.py
import random
import numpy as np
from pxr import Gf, Usd, UsdGeom
import omni

class DomainRandomizer:
    def __init__(self, config):
        self.config = config
        self.dr_config = config['synthetic_data_generation']['domain_randomization']

    def apply_texture_randomization(self, stage, prim_path):
        """
        Apply texture randomization to a USD prim
        """
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return

        # Randomize material properties
        if self.dr_config['textures']:
            material_path = f"{prim_path}_Material"
            material = UsdShade.Material.Define(stage, material_path)

            # Create shader
            shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")

            # Randomize base color
            base_color = Gf.Vec3f(
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0),
                random.uniform(0.1, 1.0)
            )
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(base_color)

            # Randomize roughness
            roughness = random.uniform(0.1, 0.9)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)

            # Randomize metallic
            metallic = random.uniform(0.0, 1.0)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

            # Connect shader to material
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    def apply_lighting_randomization(self, stage):
        """
        Apply lighting randomization to the scene
        """
        if not self.dr_config['lighting']:
            return

        # Find or create dome light
        dome_light_path = "/World/DomeLight"
        dome_light = UsdGeom.DomeLight.Get(stage, dome_light_path)
        if not dome_light:
            dome_light = UsdGeom.DomeLight.Define(stage, dome_light_path)

        # Randomize lighting properties
        intensity = random.uniform(1000, 8000)
        dome_light.GetIntensityAttr().Set(intensity)

        color = Gf.Vec3f(
            random.uniform(0.7, 1.0),
            random.uniform(0.7, 1.0),
            random.uniform(0.7, 1.0)
        )
        dome_light.GetColorAttr().Set(color)

    def apply_object_placement_randomization(self, stage, object_paths):
        """
        Randomize object placements in the scene
        """
        for obj_path in object_paths:
            prim = stage.GetPrimAtPath(obj_path)
            if not prim.IsValid():
                continue

            # Get current transform
            xform = UsdGeom.Xformable(prim)
            ops = xform.GetOrderedXformOps()

            # Apply random translation
            current_pos = Gf.Vec3d(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(0.1, 2.0))

            # Find translate op or create one
            translate_op = None
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break

            if not translate_op:
                translate_op = xform.AddTranslateOp()

            translate_op.Set(current_pos)

    def apply_camera_randomization(self, camera_prim):
        """
        Apply randomization to camera parameters
        """
        if not self.dr_config['camera_noise']:
            return

        # Add small random noise to camera position/rotation
        # This would be implemented based on the specific camera setup
        pass

    def randomize_scene(self, stage, scene_objects):
        """
        Apply all domain randomization techniques to a scene
        """
        self.apply_lighting_randomization(stage)
        self.apply_object_placement_randomization(stage, scene_objects)

        # Apply texture randomization to all objects
        for obj_path in scene_objects:
            self.apply_texture_randomization(stage, obj_path)

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
        "physics": {
            "enabled": False,  # Usually disabled for perception tasks
            "friction_range": [0.1, 0.9],
            "restitution_range": [0.0, 0.5]
        },
        "background": {
            "enabled": True,
            "clutter_density": [0.0, 0.8],
            "complexity": [0.1, 0.9]
        }
    }

    return dr_config
```

### 3. Data Validation Module
```python
# synthetic_data_pipeline/validation.py
import json
import cv2
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

class DataValidator:
    def __init__(self, config):
        self.config = config

    def validate_image_quality(self, image_path):
        """
        Validate the quality of generated images
        """
        img = cv2.imread(str(image_path))
        if img is None:
            return {"valid": False, "error": "Could not load image"}

        results = {
            "valid": True,
            "resolution": img.shape[:2],
            "mean_brightness": float(np.mean(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))),
            "contrast": float(np.std(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))),
            "sharpness": self.calculate_sharpness(img)
        }

        return results

    def calculate_sharpness(self, image):
        """
        Calculate image sharpness using Laplacian variance
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        return float(laplacian_var)

    def validate_annotations(self, annotation_path, image_path):
        """
        Validate annotations against the corresponding image
        """
        with open(annotation_path, 'r') as f:
            ann_data = json.load(f)

        img = cv2.imread(str(image_path))
        if img is None:
            return {"valid": False, "error": "Could not load image for annotation validation"}

        h, w = img.shape[:2]
        results = {"valid": True, "annotation_issues": []}

        # Check if bounding boxes are within image bounds
        for bbox in ann_data.get('bounding_boxes', []):
            x, y, box_w, box_h = bbox['bbox']

            if x < 0 or y < 0 or x + box_w > w or y + box_h > h:
                results["annotation_issues"].append(f"Bounding box out of bounds: {bbox}")

            if box_w <= 0 or box_h <= 0:
                results["annotation_issues"].append(f"Invalid bounding box dimensions: {bbox}")

        return results

    def validate_depth_map(self, depth_path):
        """
        Validate depth map quality
        """
        depth = cv2.imread(str(depth_path), cv2.IMREAD_ANYDEPTH)
        if depth is None:
            return {"valid": False, "error": "Could not load depth map"}

        results = {
            "valid": True,
            "min_depth": float(np.min(depth)),
            "max_depth": float(np.max(depth)),
            "mean_depth": float(np.mean(depth)),
            "valid_pixels": int(np.count_nonzero(~np.isnan(depth)))
        }

        return results

    def validate_segmentation(self, seg_path, expected_classes):
        """
        Validate segmentation map
        """
        seg = cv2.imread(str(seg_path), cv2.IMREAD_UNCHANGED)
        if seg is None:
            return {"valid": False, "error": "Could not load segmentation map"}

        unique_values = np.unique(seg)
        unexpected_classes = [v for v in unique_values if v not in expected_classes]

        results = {
            "valid": len(unexpected_classes) == 0,
            "unique_values": unique_values.tolist(),
            "unexpected_classes": unexpected_classes,
            "coverage_ratio": float(np.count_nonzero(seg) / seg.size)
        }

        return results

    def generate_validation_report(self, dataset_dir):
        """
        Generate a comprehensive validation report for the dataset
        """
        report = {
            "dataset_path": str(dataset_dir),
            "validation_date": "2025-12-07",
            "summary": {},
            "detailed_results": {}
        }

        splits = ['train', 'val', 'test']
        for split in splits:
            split_dir = Path(dataset_dir) / split
            if not split_dir.exists():
                continue

            split_results = {
                "image_validation": [],
                "annotation_validation": [],
                "depth_validation": [],
                "segmentation_validation": []
            }

            # Validate images and annotations
            images_dir = split_dir / 'images'
            labels_dir = split_dir / 'labels'
            annotations_dir = split_dir / 'annotations'

            for img_file in images_dir.glob("*.jpg"):
                # Validate image
                img_result = self.validate_image_quality(img_file)
                split_results["image_validation"].append(img_result)

                # Validate corresponding annotation
                ann_file = annotations_dir / f"{img_file.stem}.json"
                if ann_file.exists():
                    ann_result = self.validate_annotations(ann_file, img_file)
                    split_results["annotation_validation"].append(ann_result)

                # Validate depth if exists
                depth_file = images_dir / f"{img_file.stem}_depth.exr"
                if depth_file.exists():
                    depth_result = self.validate_depth_map(depth_file)
                    split_results["depth_validation"].append(depth_result)

                # Validate segmentation if exists
                seg_file = labels_dir / f"{img_file.stem}_seg.png"
                if seg_file.exists():
                    expected_classes = list(range(15))  # Adjust based on your classes
                    seg_result = self.validate_segmentation(seg_file, expected_classes)
                    split_results["segmentation_validation"].append(seg_result)

            report["detailed_results"][split] = split_results

        # Generate summary statistics
        self.generate_summary_statistics(report)

        # Save report
        report_path = Path(dataset_dir) / "validation_report.json"
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)

        print(f"Validation report saved to: {report_path}")
        return report

    def generate_summary_statistics(self, report):
        """
        Generate summary statistics for the validation report
        """
        summary = {}

        for split, results in report["detailed_results"].items():
            split_summary = {}

            # Image validation summary
            img_valid_count = sum(1 for r in results["image_validation"] if r["valid"])
            total_img = len(results["image_validation"])
            split_summary["image_validation_rate"] = img_valid_count / total_img if total_img > 0 else 0

            if results["image_validation"]:
                avg_brightness = np.mean([r.get("mean_brightness", 0) for r in results["image_validation"]])
                avg_contrast = np.mean([r.get("contrast", 0) for r in results["image_validation"]])
                avg_sharpness = np.mean([r.get("sharpness", 0) for r in results["image_validation"]])

                split_summary["avg_brightness"] = float(avg_brightness)
                split_summary["avg_contrast"] = float(avg_contrast)
                split_summary["avg_sharpness"] = float(avg_sharpness)

            # Annotation validation summary
            ann_valid_count = sum(1 for r in results["annotation_validation"] if r["valid"])
            total_ann = len(results["annotation_validation"])
            split_summary["annotation_validation_rate"] = ann_valid_count / total_ann if total_ann > 0 else 0

            # Depth validation summary
            depth_valid_count = sum(1 for r in results["depth_validation"] if r["valid"])
            total_depth = len(results["depth_validation"])
            split_summary["depth_validation_rate"] = depth_valid_count / total_depth if total_depth > 0 else 0

            # Segmentation validation summary
            seg_valid_count = sum(1 for r in results["segmentation_validation"] if r["valid"])
            total_seg = len(results["segmentation_validation"])
            split_summary["segmentation_validation_rate"] = seg_valid_count / total_seg if total_seg > 0 else 0

            summary[split] = split_summary

        report["summary"] = summary

def validate_dataset_quality(dataset_path):
    """
    Convenience function to validate an entire dataset
    """
    config = {"validation": {"enabled": True}}
    validator = DataValidator(config)
    report = validator.generate_validation_report(Path(dataset_path))
    return report
```

## Quality Assurance

### 1. Data Quality Metrics
```python
# synthetic_data_pipeline/quality_metrics.py
import numpy as np
import cv2
from scipy import stats
import matplotlib.pyplot as plt

class QualityMetrics:
    @staticmethod
    def calculate_image_quality_metrics(image):
        """
        Calculate various image quality metrics
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        metrics = {
            # Sharpness metrics
            "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            "sobel_magnitude": float(np.mean(np.sqrt(
                cv2.Sobel(gray, cv2.CV_64F, 1, 0)**2 + cv2.Sobel(gray, cv2.CV_64F, 0, 1)**2
            ))),

            # Contrast metrics
            "contrast_std": float(np.std(gray)),
            "entropy": QualityMetrics.calculate_entropy(gray),

            # Color metrics
            "colorfulness": QualityMetrics.calculate_colorfulness(image),

            # Brightness metrics
            "brightness_mean": float(np.mean(gray)),
            "brightness_std": float(np.std(gray))
        }

        return metrics

    @staticmethod
    def calculate_entropy(image):
        """
        Calculate Shannon entropy of an image
        """
        hist, _ = np.histogram(image.flatten(), bins=256, range=[0, 256])
        hist = hist[hist > 0]  # Remove zero values
        prob = hist / hist.sum()
        entropy = -np.sum(prob * np.log2(prob))
        return float(entropy)

    @staticmethod
    def calculate_colorfulness(image):
        """
        Calculate colorfulness metric as per Hasler and Suesstrunk (2003)
        """
        (B, G, R) = cv2.split(image.astype("float"))

        rg = np.absolute(R - G)
        yb = np.absolute(0.5 * (R + G) - B)

        (rbMean, rbStd) = (np.mean(rg), np.std(rg))
        (ybMean, ybStd) = (np.mean(yb), np.std(yb))

        stdRoot = np.sqrt((rbStd ** 2) + (ybStd ** 2))
        meanRoot = np.sqrt((rbMean ** 2) + (ybMean ** 2))

        return float(stdRoot + (0.3 * meanRoot))

    @staticmethod
    def compare_to_real_data(synthetic_data, real_data_distribution):
        """
        Compare synthetic data distribution to real data
        """
        # Calculate statistical similarity metrics
        # This would involve comparing feature distributions
        # between synthetic and real data

        results = {
            "ks_statistic": 0.0,  # Kolmogorov-Smirnov test
            "js_divergence": 0.0, # Jensen-Shannon divergence
            "correlation": 0.0,   # Correlation between distributions
            "similarity_score": 0.0
        }

        return results

    @staticmethod
    def generate_quality_report(dataset_path):
        """
        Generate a comprehensive quality report
        """
        # This would analyze the entire dataset
        # and generate statistics about quality metrics
        pass
```

## Performance Optimization

### 1. Efficient Data Generation
```python
# synthetic_data_pipeline/optimization.py
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
import queue
import threading

class PipelineOptimizer:
    def __init__(self, max_workers=None):
        if max_workers is None:
            max_workers = min(32, (mp.cpu_count() or 1) + 4)
        self.max_workers = max_workers

    def parallel_scene_generation(self, scene_configs):
        """
        Generate multiple scenes in parallel
        """
        with ProcessPoolExecutor(max_workers=self.max_workers) as executor:
            futures = [executor.submit(self.generate_single_scene, config)
                      for config in scene_configs]

            results = []
            for future in futures:
                try:
                    result = future.result(timeout=300)  # 5 minute timeout
                    results.append(result)
                except Exception as e:
                    print(f"Scene generation failed: {e}")
                    continue

        return results

    def generate_single_scene(self, config):
        """
        Generate a single scene (this would be implemented with Isaac Sim)
        """
        # This is a placeholder for actual Isaac Sim scene generation
        import time
        time.sleep(0.1)  # Simulate processing time

        return {"status": "success", "config": config}

    def memory_efficient_data_pipeline(self):
        """
        Implement memory-efficient data generation using generators
        """
        def data_generator():
            for i in range(1000):  # Simulate 1000 samples
                # Generate sample data without storing everything in memory
                sample = {
                    "id": i,
                    "image": f"sample_{i:06d}.jpg",
                    "annotations": f"ann_{i:06d}.json"
                }
                yield sample

        return data_generator()
```

## Troubleshooting

### Common Issues

#### 1. Memory Issues
- **Solution**: Use batch processing, implement data generators, optimize image formats

#### 2. Quality Issues
- **Solution**: Validate data quality, adjust domain randomization parameters

#### 3. Performance Issues
- **Solution**: Optimize pipeline, use parallel processing, adjust scene complexity

## Next Steps
After implementing the synthetic data generation pipeline:
1. Test perception pipeline integration (Task 2.3)
2. Configure navigation system (Task 2.4)
3. Implement domain randomization techniques (Task 3.1)
4. Validate synthetic-to-real transfer effectiveness

## Resources
- [Isaac Sim Synthetic Data Generation](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_synthetic_data.html)
- [Domain Randomization Best Practices](https://arxiv.org/abs/1703.06907)
- [COCO Dataset Format](https://cocodataset.org/#format-data)
- [Synthetic Data for Computer Vision](https://developer.nvidia.com/blog/generating-synthetic-data-with-isaac-sim/)