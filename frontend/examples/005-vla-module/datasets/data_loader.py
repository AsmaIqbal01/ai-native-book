"""
VLA Module Mini Dataset - Data Loader

This module provides utilities for loading and processing the VLA mini dataset.
Supports RGB images, depth maps, segmentation masks, and bounding box labels.

Usage:
    from data_loader import VLADataset

    dataset = VLADataset(root_dir='examples/vla-module/datasets/')
    sample = dataset[0]  # Get first sample
    dataset.visualize(0)  # Visualize first sample
"""

import os
import json
from typing import Dict, List, Optional, Tuple
from pathlib import Path

import numpy as np
try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("Warning: OpenCV not installed. Install with: pip install opencv-python")

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: Matplotlib not installed. Install with: pip install matplotlib")


class VLADataset:
    """
    VLA Mini Dataset loader for perception pipeline validation.

    Attributes:
        root_dir: Root directory containing the dataset
        labels: Loaded labels from labels.json
        classes: Dictionary mapping class IDs to class names
    """

    def __init__(self, root_dir: str = 'examples/vla-module/datasets/'):
        """
        Initialize the VLA dataset loader.

        Args:
            root_dir: Path to dataset root directory
        """
        self.root_dir = Path(root_dir)
        self.images_dir = self.root_dir / 'images'
        self.depth_dir = self.root_dir / 'depth'
        self.masks_dir = self.root_dir / 'masks'
        self.labels_file = self.root_dir / 'labels.json'

        # Load labels
        if not self.labels_file.exists():
            raise FileNotFoundError(
                f"labels.json not found at {self.labels_file}. "
                "Dataset may not be generated yet."
            )

        with open(self.labels_file, 'r') as f:
            labels_data = json.load(f)

        self.labels = labels_data['images']
        self.classes = labels_data.get('classes', {})
        self.version = labels_data.get('version', 'unknown')

    def __len__(self) -> int:
        """Return number of images in dataset."""
        return len(self.labels)

    def __getitem__(self, idx: int) -> Dict:
        """
        Load a sample from the dataset.

        Args:
            idx: Sample index

        Returns:
            Dictionary containing:
                - 'rgb': RGB image (H, W, 3) uint8
                - 'depth': Depth map (H, W) float32 in meters
                - 'mask': Segmentation mask (H, W) int32
                - 'labels': List of object dictionaries
                - 'filename': Image filename
                - 'id': Image ID
        """
        if idx < 0 or idx >= len(self):
            raise IndexError(f"Index {idx} out of range [0, {len(self)})")

        image_info = self.labels[idx]

        # Load RGB image
        rgb_path = self.images_dir / image_info['filename']
        if not HAS_OPENCV:
            raise RuntimeError("OpenCV is required to load images")
        rgb = cv2.imread(str(rgb_path))
        if rgb is None:
            raise FileNotFoundError(f"Could not load RGB image: {rgb_path}")
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

        # Load depth map
        depth_file = image_info.get('depth_file', '')
        depth_path = self.depth_dir / depth_file

        if depth_path.suffix == '.npy':
            depth = np.load(str(depth_path))
        elif depth_path.suffix in ['.png', '.tiff']:
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            if depth is None:
                raise FileNotFoundError(f"Could not load depth map: {depth_path}")
            # Convert from mm to meters (assuming 16-bit PNG in mm)
            depth = depth.astype(np.float32) / 1000.0
        else:
            raise ValueError(f"Unsupported depth format: {depth_path.suffix}")

        # Load segmentation mask
        mask_file = image_info.get('mask_file', '')
        mask_path = self.masks_dir / mask_file

        if mask_path.suffix == '.npy':
            mask = np.load(str(mask_path))
        elif mask_path.suffix == '.png':
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise FileNotFoundError(f"Could not load mask: {mask_path}")
        else:
            raise ValueError(f"Unsupported mask format: {mask_path.suffix}")

        # Prepare sample
        sample = {
            'rgb': rgb,
            'depth': depth,
            'mask': mask,
            'labels': image_info.get('objects', []),
            'filename': image_info['filename'],
            'id': image_info['id']
        }

        return sample

    def visualize(self, idx: int, show_depth: bool = True,
                  show_mask: bool = True, show_bbox: bool = True,
                  save_path: Optional[str] = None):
        """
        Visualize a dataset sample with RGB, depth, mask, and bounding boxes.

        Args:
            idx: Sample index to visualize
            show_depth: Whether to display depth map
            show_mask: Whether to display segmentation mask
            show_bbox: Whether to draw bounding boxes on RGB
            save_path: Optional path to save visualization
        """
        if not HAS_MATPLOTLIB:
            raise RuntimeError("Matplotlib is required for visualization")

        sample = self[idx]
        rgb = sample['rgb']
        depth = sample['depth']
        mask = sample['mask']
        labels = sample['labels']

        # Calculate subplot layout
        n_plots = 1 + show_depth + show_mask
        fig, axes = plt.subplots(1, n_plots, figsize=(6 * n_plots, 6))
        if n_plots == 1:
            axes = [axes]

        plot_idx = 0

        # Plot RGB with bounding boxes
        rgb_display = rgb.copy()
        if show_bbox and HAS_OPENCV:
            for obj in labels:
                bbox = obj['bbox']  # [x_min, y_min, x_max, y_max]
                class_name = obj['class_name']

                # Draw rectangle
                cv2.rectangle(rgb_display,
                            (bbox[0], bbox[1]),
                            (bbox[2], bbox[3]),
                            (0, 255, 0), 2)

                # Draw label
                cv2.putText(rgb_display, class_name,
                          (bbox[0], bbox[1] - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                          (0, 255, 0), 2)

        axes[plot_idx].imshow(rgb_display)
        axes[plot_idx].set_title(f'RGB - {sample["filename"]}')
        axes[plot_idx].axis('off')
        plot_idx += 1

        # Plot depth map
        if show_depth:
            depth_display = axes[plot_idx].imshow(depth, cmap='plasma')
            axes[plot_idx].set_title('Depth Map (meters)')
            axes[plot_idx].axis('off')
            plt.colorbar(depth_display, ax=axes[plot_idx], fraction=0.046)
            plot_idx += 1

        # Plot segmentation mask
        if show_mask:
            mask_display = axes[plot_idx].imshow(mask, cmap='tab20')
            axes[plot_idx].set_title('Segmentation Mask')
            axes[plot_idx].axis('off')
            plt.colorbar(mask_display, ax=axes[plot_idx], fraction=0.046)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Visualization saved to: {save_path}")
        else:
            plt.show()

    def filter_by_class(self, class_name: str) -> List[int]:
        """
        Get indices of all images containing a specific object class.

        Args:
            class_name: Name of the class to filter by

        Returns:
            List of image indices containing the class
        """
        indices = []
        for idx, image_info in enumerate(self.labels):
            for obj in image_info.get('objects', []):
                if obj['class_name'] == class_name:
                    indices.append(idx)
                    break  # Found at least one, move to next image
        return indices

    def filter_by_object_count(self, min_count: int = 1,
                               max_count: Optional[int] = None) -> List[int]:
        """
        Get indices of images with object count in specified range.

        Args:
            min_count: Minimum number of objects
            max_count: Maximum number of objects (None = no limit)

        Returns:
            List of image indices matching criteria
        """
        indices = []
        for idx, image_info in enumerate(self.labels):
            obj_count = len(image_info.get('objects', []))
            if obj_count >= min_count:
                if max_count is None or obj_count <= max_count:
                    indices.append(idx)
        return indices

    def get_statistics(self) -> Dict:
        """
        Compute dataset statistics.

        Returns:
            Dictionary with dataset statistics
        """
        stats = {
            'total_images': len(self),
            'total_objects': 0,
            'class_distribution': {},
            'avg_objects_per_image': 0.0
        }

        for image_info in self.labels:
            objects = image_info.get('objects', [])
            stats['total_objects'] += len(objects)

            for obj in objects:
                class_name = obj['class_name']
                stats['class_distribution'][class_name] = \
                    stats['class_distribution'].get(class_name, 0) + 1

        if stats['total_images'] > 0:
            stats['avg_objects_per_image'] = \
                stats['total_objects'] / stats['total_images']

        return stats

    def validate(self) -> bool:
        """
        Validate dataset integrity.

        Returns:
            True if dataset passes all checks, False otherwise
        """
        print("Validating VLA Mini Dataset...")
        all_valid = True

        # Check RGB images exist
        for image_info in self.labels:
            rgb_path = self.images_dir / image_info['filename']
            if not rgb_path.exists():
                print(f"✗ Missing RGB image: {rgb_path}")
                all_valid = False

        if all_valid:
            print(f"✓ Found {len(self)} RGB images")

        # Check depth maps exist
        for image_info in self.labels:
            depth_path = self.depth_dir / image_info.get('depth_file', '')
            if not depth_path.exists():
                print(f"✗ Missing depth map: {depth_path}")
                all_valid = False

        if all_valid:
            print(f"✓ Found {len(self)} depth maps")

        # Check masks exist
        for image_info in self.labels:
            mask_path = self.masks_dir / image_info.get('mask_file', '')
            if not mask_path.exists():
                print(f"✗ Missing mask: {mask_path}")
                all_valid = False

        if all_valid:
            print(f"✓ Found {len(self)} segmentation masks")

        # Check bounding boxes
        for image_info in self.labels:
            width = image_info.get('width', 640)
            height = image_info.get('height', 480)

            for obj in image_info.get('objects', []):
                bbox = obj['bbox']
                if not (0 <= bbox[0] < bbox[2] <= width and
                       0 <= bbox[1] < bbox[3] <= height):
                    print(f"✗ Invalid bbox in {image_info['filename']}: {bbox}")
                    all_valid = False

        if all_valid:
            print("✓ All bounding boxes within bounds")

        # Check class IDs
        for image_info in self.labels:
            for obj in image_info.get('objects', []):
                class_id = str(obj['class_id'])
                if class_id not in self.classes:
                    print(f"✗ Invalid class ID: {class_id}")
                    all_valid = False

        if all_valid:
            print("✓ All class IDs valid")

        if all_valid:
            print("✓ Dataset integrity check PASSED\n")
            stats = self.get_statistics()
            print("Dataset Statistics:")
            print(f"  Total images: {stats['total_images']}")
            print(f"  Total objects: {stats['total_objects']}")
            print(f"  Average objects per image: {stats['avg_objects_per_image']:.2f}")
            print("  Object distribution:")
            for class_name, count in stats['class_distribution'].items():
                print(f"    {class_name}: {count}")
        else:
            print("✗ Dataset integrity check FAILED")

        return all_valid


def main():
    """Main function for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(description='VLA Dataset Loader')
    parser.add_argument('--root_dir', type=str,
                       default='examples/vla-module/datasets/',
                       help='Dataset root directory')
    parser.add_argument('--validate', action='store_true',
                       help='Validate dataset integrity')
    parser.add_argument('--visualize', type=int, metavar='IDX',
                       help='Visualize sample at index IDX')
    parser.add_argument('--stats', action='store_true',
                       help='Print dataset statistics')

    args = parser.parse_args()

    try:
        dataset = VLADataset(root_dir=args.root_dir)

        if args.validate:
            dataset.validate()

        if args.visualize is not None:
            dataset.visualize(args.visualize)

        if args.stats:
            stats = dataset.get_statistics()
            print("Dataset Statistics:")
            print(f"  Total images: {stats['total_images']}")
            print(f"  Total objects: {stats['total_objects']}")
            print(f"  Average objects per image: {stats['avg_objects_per_image']:.2f}")
            print("  Object distribution:")
            for class_name, count in stats['class_distribution'].items():
                print(f"    {class_name}: {count}")

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nDataset not yet generated. Generate it using:")
        print("  python scripts/capture_dataset.py")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
