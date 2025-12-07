# VLA Module Mini Dataset

**Module**: Module 4 - Vision-Language-Action Systems
**Purpose**: Curated labeled dataset for perception pipeline validation
**Size**: 50-100 images (<50MB total)
**Source**: Gazebo/Isaac Sim simulation environments

## Dataset Overview

This mini dataset provides consistent, labeled examples for students to validate their perception code before moving to live simulation. All images are captured from simulation environments to ensure consistency with later exercises.

### Dataset Contents

```
datasets/
├── images/          # RGB images (640x480 or 1920x1080)
├── depth/           # Depth maps (same resolution as RGB)
├── masks/           # Segmentation masks (same resolution as RGB)
├── labels.json      # Bounding boxes and object categories
├── data_loader.py   # Utility script for loading dataset
└── README.md        # This file
```

---

## Dataset Specification

### Images (RGB)

**Format**: PNG or JPG
**Resolution**: 640x480 (primary) or 1920x1080 (optional high-res)
**Color Space**: RGB
**Naming**: `scene_NNN.png` (e.g., `scene_001.png`, `scene_002.png`)

**Scene Variety**:
- Table-top workspace scenes (40-50 images)
- Robot navigation scenes (20-30 images)
- Object manipulation scenes (10-20 images)

**Object Diversity**:
- Geometric primitives: cubes, cylinders, spheres
- Common objects: bottles, boxes, tools
- Varied colors: red, blue, green, yellow, white, black
- Varied positions and orientations

---

### Depth Maps

**Format**: PNG (16-bit grayscale) or NumPy `.npy` format
**Resolution**: Same as corresponding RGB image
**Naming**: `depth_NNN.png` or `depth_NNN.npy` (matching RGB `scene_NNN.png`)

**Depth Range**:
- Min: 0.1m (10cm)
- Max: 10.0m
- Units: Meters
- Invalid/missing depth: 0 or NaN

**Generation**:
- Captured from simulation depth camera
- Ground truth depth values
- Aligned with RGB camera (same intrinsics and pose)

---

### Segmentation Masks

**Format**: PNG (indexed color) or NumPy `.npy` format
**Resolution**: Same as corresponding RGB image
**Naming**: `mask_NNN.png` or `mask_NNN.npy` (matching RGB `scene_NNN.png`)

**Mask Format**:
- Each pixel value represents object ID or class ID
- Background: 0
- Object 1: 1
- Object 2: 2
- etc.

**Class Categories** (example):
```python
CLASSES = {
    0: "background",
    1: "table",
    2: "red_cube",
    3: "blue_cylinder",
    4: "green_sphere",
    5: "robot_gripper",
    6: "obstacle",
    # ... add more as needed
}
```

---

### Labels (Bounding Boxes and Categories)

**Format**: JSON file (`labels.json`)

**Structure**:
```json
{
  "version": "1.0",
  "dataset": "vla-module-mini",
  "total_images": 75,
  "classes": {
    "0": "background",
    "1": "table",
    "2": "red_cube",
    "3": "blue_cylinder",
    "4": "green_sphere",
    "5": "robot_gripper",
    "6": "obstacle"
  },
  "images": [
    {
      "id": 1,
      "filename": "scene_001.png",
      "width": 640,
      "height": 480,
      "depth_file": "depth_001.png",
      "mask_file": "mask_001.png",
      "objects": [
        {
          "class_id": 2,
          "class_name": "red_cube",
          "bbox": [120, 200, 180, 260],  // [x_min, y_min, x_max, y_max]
          "center_2d": [150, 230],        // [x, y]
          "center_3d": [0.5, -0.2, 0.8],  // [x, y, z] in meters
          "confidence": 1.0
        },
        {
          "class_id": 3,
          "class_name": "blue_cylinder",
          "bbox": [350, 180, 410, 280],
          "center_2d": [380, 230],
          "center_3d": [0.6, 0.1, 0.75],
          "confidence": 1.0
        }
      ]
    },
    {
      "id": 2,
      "filename": "scene_002.png",
      "width": 640,
      "height": 480,
      "depth_file": "depth_002.png",
      "mask_file": "mask_002.png",
      "objects": [
        // ... objects for scene 2
      ]
    }
    // ... more images
  ]
}
```

---

## Data Loader Usage

The `data_loader.py` script provides utilities for loading and visualizing the dataset.

### Basic Usage

```python
from data_loader import VLADataset

# Load dataset
dataset = VLADataset(root_dir='examples/vla-module/datasets/')

# Get number of images
print(f"Dataset size: {len(dataset)}")

# Load single sample
sample = dataset[0]
rgb = sample['rgb']          # NumPy array (H, W, 3)
depth = sample['depth']      # NumPy array (H, W)
mask = sample['mask']        # NumPy array (H, W)
labels = sample['labels']    # List of object dictionaries

# Visualize sample
dataset.visualize(0, show_depth=True, show_mask=True, show_bbox=True)
```

### Iteration

```python
for idx, sample in enumerate(dataset):
    rgb = sample['rgb']
    depth = sample['depth']
    mask = sample['mask']
    labels = sample['labels']

    # Process sample
    # ...
```

### Filtering by Object Class

```python
# Get all images containing red cubes
red_cube_images = dataset.filter_by_class("red_cube")

# Get all images with at least 2 objects
multi_object_images = dataset.filter_by_object_count(min_count=2)
```

---

## Validation and Verification

### Dataset Integrity Check

Run the validation script to verify dataset integrity:

```bash
cd examples/vla-module/datasets/
python data_loader.py --validate
```

**Checks performed**:
- ✅ All RGB images exist and are valid
- ✅ All depth maps exist and match RGB dimensions
- ✅ All masks exist and match RGB dimensions
- ✅ labels.json is valid JSON and contains all images
- ✅ Bounding boxes are within image bounds
- ✅ Class IDs are valid

### Expected Output

```
Validating VLA Mini Dataset...
✓ Found 75 RGB images
✓ Found 75 depth maps
✓ Found 75 segmentation masks
✓ labels.json is valid
✓ All bounding boxes within bounds
✓ All class IDs valid
✓ Dataset integrity check PASSED

Dataset Statistics:
  Total images: 75
  Total objects: 142
  Average objects per image: 1.89
  Object distribution:
    red_cube: 32
    blue_cylinder: 28
    green_sphere: 25
    table: 75
    robot_gripper: 12
    obstacle: 10
```

---

## Dataset Generation (For Instructors)

If you need to regenerate or extend the dataset:

### Using Gazebo

```bash
# Launch Gazebo with VLA world
ros2 launch vla_scenarios dataset_capture.launch.py

# Run capture script
python scripts/capture_dataset.py \
  --num_images 100 \
  --output_dir examples/vla-module/datasets/ \
  --randomize_objects \
  --randomize_lighting
```

### Using Isaac Sim

```bash
# Launch Isaac Sim (requires Isaac installation)
python scripts/isaac_dataset_capture.py \
  --num_scenes 100 \
  --output_dir examples/vla-module/datasets/ \
  --resolution 640x480
```

---

## Dataset Licensing

**License**: MIT License (for educational use)

**Attribution**: This dataset is generated from simulation environments (Gazebo/Isaac Sim) for educational purposes in the AI-Native Robotics textbook.

**Usage**: Free to use for educational purposes. If you use this dataset in research or publications, please cite the AI-Native Robotics textbook.

---

## Dataset Statistics (When Complete)

**Target**:
- Total images: 50-100
- Total objects: 100-200
- Average objects per image: 1-3
- Scene variety: Table-top (50%), Navigation (30%), Manipulation (20%)

**Actual** (to be filled after generation):
- Total images: [TBD]
- Total objects: [TBD]
- Average objects per image: [TBD]
- Object distribution: [TBD]

---

## Troubleshooting

### Issue: Images not loading

**Solution**: Check file paths are correct and images exist in `images/` directory.

### Issue: Depth maps have wrong scale

**Solution**: Depth maps are in meters. If using PNG, convert from 16-bit values:
```python
depth_m = depth_png.astype(float) / 1000.0  # Convert mm to m
```

### Issue: Segmentation masks not aligned

**Solution**: Ensure RGB, depth, and mask all have same resolution and camera intrinsics.

### Issue: Bounding boxes out of bounds

**Solution**: Verify bbox format is [x_min, y_min, x_max, y_max] and within [0, width] x [0, height].

---

## Next Steps

1. ✅ Set up dataset directory structure
2. ⏳ Generate dataset from simulation (requires Gazebo/Isaac setup)
3. ⏳ Validate dataset integrity
4. ⏳ Create example perception code using dataset
5. ⏳ Test perception accuracy on dataset before live simulation

---

## Contact

For questions about the dataset or to report issues:
- File an issue in the textbook repository
- Contact: [Add contact information]
