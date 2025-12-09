# VLA Module Mini Dataset

**Purpose**: Small curated dataset for perception pipeline validation and testing.

**Size**: 50-100 labeled images (<50MB total)

**Tasks**: T010-T016

## Dataset Structure

```
datasets/
├── images/          # RGB images from simulation (PNG/JPG)
├── depth/           # Corresponding depth maps (PNG)
├── masks/           # Segmentation masks (PNG)
├── labels.json      # Annotations (bounding boxes, object categories)
├── data_loader.py   # Dataset loader utility
└── README.md        # This file
```

## Dataset Contents

### RGB Images (images/)
- Format: PNG or JPG
- Resolution: 640x480 (default) or 1280x720
- Source: Captured from Gazebo/Isaac Sim
- Scenes: Table workspaces with various objects

### Depth Maps (depth/)
- Format: PNG (16-bit or 32-bit)
- Units: Millimeters or meters (see labels.json metadata)
- Correspondence: One depth map per RGB image (same filename)

### Segmentation Masks (masks/)
- Format: PNG (grayscale or indexed color)
- Encoding: Each object class has unique pixel value
- Classes: Background (0), Table (1), Objects (2-N)

### Annotations (labels.json)
Format:
```json
{
  "metadata": {
    "dataset_version": "1.0",
    "capture_date": "2025-12-09",
    "resolution": [640, 480],
    "depth_units": "meters"
  },
  "classes": [
    {"id": 0, "name": "background"},
    {"id": 1, "name": "table"},
    {"id": 2, "name": "red_cube"},
    {"id": 3, "name": "blue_cylinder"}
  ],
  "samples": [
    {
      "id": "000001",
      "rgb_file": "images/000001.png",
      "depth_file": "depth/000001.png",
      "mask_file": "masks/000001.png",
      "objects": [
        {
          "class_id": 2,
          "bbox": [100, 150, 200, 250],
          "center_3d": [0.5, 0.0, 0.8]
        }
      ]
    }
  ]
}
```

## Usage

### Loading Dataset in Python

```python
from data_loader import VLADataset

# Initialize dataset
dataset = VLADataset('path/to/datasets/')

# Get sample
sample = dataset.get_sample(0)
rgb = sample['rgb']
depth = sample['depth']
mask = sample['mask']
annotations = sample['annotations']
```

### Validation

```bash
cd examples/vla-module/datasets
python3 data_loader.py
```

Expected output:
```
Dataset loaded: 50 samples
Validation: {'valid': True, 'num_samples': 50, 'errors': []}
```

## Creating Your Own Dataset

### 1. Capture Images from Simulation

```bash
# Launch Gazebo with workspace
ros2 launch vla_scenarios workspace_simple.launch.py

# Capture images (TODO: Add capture script)
ros2 run vla_perception capture_dataset.py --output datasets/
```

### 2. Generate Depth Maps

Depth maps are automatically captured alongside RGB if using RGB-D cameras in simulation.

### 3. Generate Segmentation Masks

Use simulation ground truth segmentation or run lightweight segmentation model:

```bash
python3 generate_masks.py --input datasets/images --output datasets/masks
```

### 4. Annotate Objects

Manually create labels.json following the format above, or use annotation tool.

## Attribution

- Images: Generated from Gazebo Classic / Isaac Sim
- Objects: Standard simulation models (cubes, cylinders, etc.)
- License: MIT (for educational use)

## Verification Checklist

- [ ] RGB images present (50-100 samples)
- [ ] Depth maps present (same count as RGB)
- [ ] Segmentation masks present (same count as RGB)
- [ ] labels.json exists and valid JSON
- [ ] All filenames in labels.json match actual files
- [ ] Bounding boxes within image bounds
- [ ] Dataset loader runs without errors
- [ ] Sample visualizations look correct

## Next Steps

- Use dataset for perception validation (Chapter 2)
- Test segmentation accuracy on dataset
- Verify object detection accuracy ≥80%
- Move to live simulation after validation
