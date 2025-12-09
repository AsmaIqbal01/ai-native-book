# Sim-to-Real Transfer Techniques Guide

## Overview
This guide provides instructions for developing and validating sim-to-real transfer methods. The techniques will enable models trained in Isaac Sim to perform effectively on real robots, addressing the reality gap between simulation and real-world environments.

## Sim-to-Real Transfer Concepts

### 1. Key Transfer Learning Approaches
- **Domain Randomization**: Making simulation diverse enough to cover real-world variations
- **Domain Adaptation**: Adapting synthetic-trained models to real data
- **GAN-based Methods**: Using generative models for domain transfer
- **System Identification**: Adjusting simulation to match reality
- **Calibration**: Aligning simulation and real-world behavior

### 2. Transfer Learning Strategies
- **Direct Transfer**: Using simulation-trained models directly on real robots
- **Fine-tuning**: Adapting models with small amounts of real data
- **Adversarial Training**: Training models to be domain-invariant
- **Progressive Domain Transfer**: Gradually moving from simulation to reality

## Configuration Files

### 1. Transfer Configuration
```yaml
# config/transfer_config.yaml
sim2real_transfer:
  version: "1.0"
  enabled: true
  transfer_method: "domain_randomization"  # domain_randomization, domain_adaptation, direct_transfer
  validation_enabled: true
  performance_threshold: 0.80  # Minimum performance for successful transfer
  domain_gap_threshold: 0.10   # Maximum acceptable gap between sim and real

  domain_randomization:
    enabled: true
    randomization_factor: 0.8
    max_variations: 10000
    texture_variations: 500
    lighting_variations: 300
    physics_variations: 200

  domain_adaptation:
    enabled: false
    adaptation_method: "correlation_alignment"  # correlation_alignment, mmd, adversarial
    source_domain: "simulation"
    target_domain: "real_world"
    adaptation_samples: 100

  fine_tuning:
    enabled: true
    real_data_samples: 50
    learning_rate: 0.0001
    epochs: 10
    batch_size: 32

  validation:
    metrics:
      - "accuracy"
      - "precision"
      - "recall"
      - "f1_score"
      - "mean_average_error"
    comparison_threshold: 0.05  # Maximum acceptable performance drop
    validation_frequency: 100   # Validate every 100 episodes
</`

### 2. Transfer Validation Configuration
```yaml
# config/transfer_validation.yaml
transfer_validation:
  version: "1.0"
  validation_methods:
    - "performance_comparison"
    - "behavior_similarity"
    - "safety_verification"

  performance_comparison:
    metrics:
      - "success_rate"
      - "accuracy"
      - "precision"
      - "recall"
      - "f1_score"
      - "mean_time_to_completion"
    threshold: 0.90  # Minimum acceptable performance ratio (real/sim)
    sample_size: 100

  behavior_similarity:
    metrics:
      - "trajectory_similarity"
      - "action_similarity"
      - "state_similarity"
    similarity_threshold: 0.85
    comparison_method: "dynamic_time_warping"  # dtw, cosine_similarity, euclidean

  safety_verification:
    checks:
      - "collision_avoidance"
      - "stability_verification"
      - "joint_limit_compliance"
      - "velocity_limit_compliance"
    safety_threshold: 0.95  # Minimum safety compliance rate

  transfer_success_criteria:
    performance_score: 0.80
    similarity_score: 0.85
    safety_score: 0.95
    overall_threshold: 0.85
```

## Implementation Files

### 1. Transfer Learning Module
```python
# transfer/transfer_learning.py
import numpy as np
import torch
import torch.nn as nn
from typing import List, Dict, Any, Tuple, Optional
import yaml
import pickle
import os
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score
from abc import ABC, abstractmethod

class TransferMethod(ABC):
    """Abstract base class for transfer learning methods"""

    @abstractmethod
    def transfer(self, source_model: Any, target_data: Any) -> Any:
        """Transfer knowledge from source to target domain"""
        pass

    @abstractmethod
    def validate_transfer(self, transferred_model: Any, test_data: Any) -> Dict[str, float]:
        """Validate the effectiveness of the transfer"""
        pass


class DomainRandomizationTransfer(TransferMethod):
    """Domain randomization based transfer method"""

    def __init__(self, config_path: str = None):
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

    def _get_default_config(self) -> Dict:
        return {
            "domain_randomization": {
                "randomization_factor": 0.8,
                "max_variations": 10000,
                "texture_variations": 500,
                "lighting_variations": 300,
                "physics_variations": 200
            }
        }

    def transfer(self, source_model: Any, target_data: Any) -> Any:
        """
        Apply domain randomization to improve transfer
        """
        # In practice, this would involve retraining with randomized data
        # For this example, we'll return the source model as is
        # since the randomization happens during data generation
        print("Domain randomization applied during data generation phase")
        return source_model

    def validate_transfer(self, transferred_model: Any, test_data: Any) -> Dict[str, float]:
        """
        Validate transfer effectiveness using test data
        """
        # This is a simplified validation
        # In practice, this would involve real-world testing
        if isinstance(test_data, tuple) and len(test_data) == 2:
            X_test, y_test = test_data
            # Simulate predictions
            y_pred = np.random.choice([0, 1], size=len(y_test), p=[0.3, 0.7])

            metrics = {
                "accuracy": accuracy_score(y_test, y_pred),
                "precision": precision_score(y_test, y_pred, average='weighted', zero_division=0),
                "recall": recall_score(y_test, y_pred, average='weighted', zero_division=0),
                "f1_score": f1_score(y_test, y_pred, average='weighted', zero_division=0)
            }
        else:
            # If no test data provided, return mock metrics
            metrics = {
                "accuracy": 0.85,
                "precision": 0.82,
                "recall": 0.84,
                "f1_score": 0.83
            }

        return metrics


class DomainAdaptationTransfer(TransferMethod):
    """Domain adaptation based transfer method"""

    def __init__(self, config_path: str = None):
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

    def _get_default_config(self) -> Dict:
        return {
            "domain_adaptation": {
                "adaptation_method": "correlation_alignment",
                "source_domain": "simulation",
                "target_domain": "real_world",
                "adaptation_samples": 100,
                "learning_rate": 0.001,
                "epochs": 10
            }
        }

    def transfer(self, source_model: Any, target_data: Any) -> Any:
        """
        Adapt source model to target domain using limited real data
        """
        print("Applying domain adaptation...")

        # This is a simplified example of domain adaptation
        # In practice, this would involve techniques like:
        # - Correlation Alignment (CORAL)
        # - Maximum Mean Discrepancy (MMD)
        # - Adversarial Domain Adaptation

        if target_data is not None:
            print(f"Adapting model using {len(target_data) if hasattr(target_data, '__len__') else 'unknown'} target samples")

        # For this example, we'll simulate adaptation by slightly adjusting the model
        adapted_model = source_model  # In reality, this would involve actual adaptation

        print("Domain adaptation completed")
        return adapted_model

    def validate_transfer(self, transferred_model: Any, test_data: Any) -> Dict[str, float]:
        """
        Validate domain adaptation effectiveness
        """
        # Simulate validation with improved metrics due to adaptation
        metrics = {
            "accuracy": 0.88,
            "precision": 0.86,
            "recall": 0.87,
            "f1_score": 0.86
        }

        return metrics


class FineTuningTransfer(TransferMethod):
    """Fine-tuning based transfer method"""

    def __init__(self, config_path: str = None):
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

    def _get_default_config(self) -> Dict:
        return {
            "fine_tuning": {
                "real_data_samples": 50,
                "learning_rate": 0.0001,
                "epochs": 10,
                "batch_size": 32,
                "freeze_base_layers": true
            }
        }

    def transfer(self, source_model: Any, target_data: Any) -> Any:
        """
        Fine-tune source model with real-world data
        """
        print("Fine-tuning model with real-world data...")

        if target_data is not None:
            print(f"Fine-tuning using {len(target_data) if hasattr(target_data, '__len__') else 'unknown'} real samples")

        # This would involve actual fine-tuning in practice
        fine_tuned_model = source_model  # Placeholder

        print("Fine-tuning completed")
        return fine_tuned_model

    def validate_transfer(self, transferred_model: Any, test_data: Any) -> Dict[str, float]:
        """
        Validate fine-tuning effectiveness
        """
        # Simulate validation with metrics reflecting fine-tuning improvement
        metrics = {
            "accuracy": 0.90,
            "precision": 0.89,
            "recall": 0.89,
            "f1_score": 0.89
        }

        return metrics


class TransferValidator:
    """Validate sim-to-real transfer effectiveness"""

    def __init__(self, config_path: str = None):
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_validation_config()

    def _get_default_validation_config(self) -> Dict:
        return {
            "transfer_validation": {
                "performance_comparison": {
                    "threshold": 0.90,
                    "sample_size": 100
                },
                "behavior_similarity": {
                    "similarity_threshold": 0.85,
                    "comparison_method": "dynamic_time_warping"
                },
                "safety_verification": {
                    "safety_threshold": 0.95
                },
                "transfer_success_criteria": {
                    "performance_score": 0.80,
                    "similarity_score": 0.85,
                    "safety_score": 0.95,
                    "overall_threshold": 0.85
                }
            }
        }

    def validate_performance(self, sim_performance: Dict[str, float],
                           real_performance: Dict[str, float]) -> Dict[str, Any]:
        """
        Validate performance transfer between simulation and real world
        """
        comparison_results = {}

        for metric in sim_performance.keys():
            if metric in real_performance:
                ratio = real_performance[metric] / sim_performance[metric] if sim_performance[metric] != 0 else 0
                comparison_results[f"{metric}_ratio"] = ratio
                comparison_results[f"{metric}_transfer_success"] = ratio >= self.config["transfer_validation"]["performance_comparison"]["threshold"]

        # Overall performance score
        ratios = [comparison_results[key] for key in comparison_results.keys() if key.endswith('_ratio')]
        overall_score = sum(ratios) / len(ratios) if ratios else 0

        comparison_results["overall_performance_score"] = overall_score
        comparison_results["performance_transfer_success"] = overall_score >= self.config["transfer_validation"]["transfer_success_criteria"]["performance_score"]

        return comparison_results

    def validate_behavior_similarity(self, sim_trajectory: List, real_trajectory: List) -> Dict[str, float]:
        """
        Validate behavioral similarity between simulation and real trajectories
        """
        # Calculate similarity using the configured method
        method = self.config["transfer_validation"]["behavior_similarity"]["comparison_method"]

        if method == "euclidean":
            similarity = self._euclidean_similarity(sim_trajectory, real_trajectory)
        elif method == "cosine":
            similarity = self._cosine_similarity(sim_trajectory, real_trajectory)
        else:  # Default to DTW
            similarity = self._dtw_similarity(sim_trajectory, real_trajectory)

        threshold = self.config["transfer_validation"]["behavior_similarity"]["similarity_threshold"]

        return {
            "similarity_score": similarity,
            "similarity_threshold": threshold,
            "behavior_transfer_success": similarity >= threshold
        }

    def _euclidean_similarity(self, traj1: List, traj2: List) -> float:
        """Calculate Euclidean similarity between trajectories"""
        if not traj1 or not traj2:
            return 0.0

        # Convert to numpy arrays for calculation
        arr1 = np.array(traj1)
        arr2 = np.array(traj2)

        # Pad shorter array if needed
        if len(arr1) != len(arr2):
            min_len = min(len(arr1), len(arr2))
            arr1 = arr1[:min_len]
            arr2 = arr2[:min_len]

        # Calculate Euclidean distance
        distance = np.linalg.norm(arr1 - arr2)

        # Convert to similarity (inverse relationship)
        max_possible_distance = np.linalg.norm(np.ones_like(arr1) * 10)  # Arbitrary max
        similarity = 1 / (1 + distance / max_possible_distance)

        return float(similarity)

    def _cosine_similarity(self, traj1: List, traj2: List) -> float:
        """Calculate cosine similarity between trajectories"""
        if not traj1 or not traj2:
            return 0.0

        # Convert to numpy arrays
        arr1 = np.array(traj1).flatten()
        arr2 = np.array(traj2).flatten()

        # Pad shorter array if needed
        if len(arr1) != len(arr2):
            min_len = min(len(arr1), len(arr2))
            arr1 = arr1[:min_len]
            arr2 = arr2[:min_len]

        # Calculate cosine similarity
        dot_product = np.dot(arr1, arr2)
        norm1 = np.linalg.norm(arr1)
        norm2 = np.linalg.norm(arr2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        similarity = dot_product / (norm1 * norm2)

        # Convert to positive similarity score
        return float((similarity + 1) / 2)  # Normalize to [0, 1]

    def _dtw_similarity(self, traj1: List, traj2: List) -> float:
        """Calculate Dynamic Time Warping similarity between trajectories"""
        # For simplicity, we'll use a basic implementation
        # In practice, use a library like dtaidistance or fastdtw

        if not traj1 or not traj2:
            return 0.0

        # Convert to numpy arrays
        arr1 = np.array(traj1)
        arr2 = np.array(traj2)

        # Simple DTW-like calculation (not a full DTW implementation)
        # This is a simplified version for demonstration
        min_len = min(len(arr1), len(arr2))
        distance = np.mean(np.abs(arr1[:min_len] - arr2[:min_len]))

        # Convert to similarity score
        max_distance = 5.0  # Arbitrary max distance
        similarity = max(0, 1 - (distance / max_distance))

        return float(similarity)

    def validate_safety(self, safety_metrics: Dict[str, float]) -> Dict[str, Any]:
        """
        Validate safety compliance during transfer
        """
        threshold = self.config["transfer_validation"]["safety_verification"]["safety_threshold"]

        safety_score = np.mean(list(safety_metrics.values())) if safety_metrics else 0.0

        safety_results = {
            "safety_score": float(safety_score),
            "safety_threshold": threshold,
            "safety_transfer_success": safety_score >= threshold,
            "safety_metrics": safety_metrics
        }

        return safety_results

    def calculate_overall_transfer_success(self, performance_results: Dict[str, Any],
                                         behavior_results: Dict[str, float],
                                         safety_results: Dict[str, Any]) -> Dict[str, Any]:
        """
        Calculate overall transfer success based on all validation results
        """
        criteria = self.config["transfer_validation"]["transfer_success_criteria"]

        performance_score = performance_results.get("overall_performance_score", 0)
        behavior_score = behavior_results.get("similarity_score", 0)
        safety_score = safety_results.get("safety_score", 0)

        # Weighted average (equal weights for simplicity)
        overall_score = (performance_score + behavior_score + safety_score) / 3

        success = (
            performance_score >= criteria["performance_score"] and
            behavior_score >= criteria["similarity_score"] and
            safety_score >= criteria["safety_score"] and
            overall_score >= criteria["overall_threshold"]
        )

        return {
            "overall_transfer_score": overall_score,
            "transfer_success": success,
            "detailed_scores": {
                "performance": performance_score,
                "behavior": behavior_score,
                "safety": safety_score
            }
        }


def create_transfer_pipeline(transfer_method: str = "domain_randomization",
                           config_path: str = None) -> Tuple[TransferMethod, TransferValidator]:
    """
    Create a transfer pipeline with the specified method
    """
    if transfer_method == "domain_adaptation":
        transfer_method_impl = DomainAdaptationTransfer(config_path)
    elif transfer_method == "fine_tuning":
        transfer_method_impl = FineTuningTransfer(config_path)
    else:  # Default to domain randomization
        transfer_method_impl = DomainRandomizationTransfer(config_path)

    validator = TransferValidator(config_path)

    return transfer_method_impl, validator
```

### 2. Transfer Integration with Isaac Sim and Isaac ROS
```python
# transfer/isaac_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import numpy as np
from typing import Dict, Any
import pickle
import os
from .transfer_learning import TransferMethod, TransferValidator

class IsaacTransferIntegrator(Node):
    def __init__(self, transfer_method: str = "domain_randomization"):
        super().__init__('isaac_transfer_integrator')

        # Initialize transfer components
        self.transfer_method, self.validator = self._setup_transfer_components(transfer_method)

        # Publishers and subscribers
        self.perception_sub = self.create_subscription(
            Image, '/camera/color/image_rect_color', self.perception_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.transfer_status_pub = self.create_publisher(Float32, '/transfer_status', 10)

        # Storage for collected data
        self.sim_data_buffer = []
        self.real_data_buffer = []

        # Transfer state
        self.transfer_performed = False
        self.transfer_success = False

        self.get_logger().info('Isaac Transfer Integrator initialized')

    def _setup_transfer_components(self, transfer_method: str):
        """Setup transfer learning components"""
        transfer_impl, validator = create_transfer_pipeline(transfer_method)
        return transfer_impl, validator

    def perception_callback(self, msg):
        """Handle perception data from Isaac Sim"""
        # Process perception data
        # This would typically feed into the transfer pipeline
        self.get_logger().debug('Received perception data for transfer validation')

    def scan_callback(self, msg):
        """Handle LIDAR scan data"""
        # Process LIDAR data
        self.get_logger().debug('Received LIDAR data for transfer validation')

    def perform_transfer(self, source_model_path: str, target_data_path: str = None):
        """
        Perform sim-to-real transfer
        """
        try:
            # Load source model (simulated trained model)
            if os.path.exists(source_model_path):
                with open(source_model_path, 'rb') as f:
                    source_model = pickle.load(f)
            else:
                # If no model file, create a mock model for demonstration
                source_model = {"model_type": "mock", "trained_on": "simulation"}

            # Load target data if available
            target_data = None
            if target_data_path and os.path.exists(target_data_path):
                with open(target_data_path, 'rb') as f:
                    target_data = pickle.load(f)

            # Perform transfer
            transferred_model = self.transfer_method.transfer(source_model, target_data)

            # Validate transfer
            validation_results = self.validate_transfer(transferred_model, target_data)

            self.transfer_performed = True
            self.transfer_success = validation_results.get('transfer_success', False)

            # Publish transfer status
            status_msg = Float32()
            status_msg.data = 1.0 if self.transfer_success else 0.0
            self.transfer_status_pub.publish(status_msg)

            self.get_logger().info(f'Transfer performed: Success={self.transfer_success}')

            return transferred_model, validation_results

        except Exception as e:
            self.get_logger().error(f'Error during transfer: {e}')
            return None, {"error": str(e)}

    def validate_transfer(self, model, test_data = None):
        """
        Validate the transferred model
        """
        try:
            # If no test data provided, use mock data for validation
            if test_data is None:
                # Generate mock test data for validation
                test_data = self._generate_mock_test_data()

            # Validate using the transfer validator
            validation_results = self.transfer_method.validate_transfer(model, test_data)

            # Add additional validation metrics
            if hasattr(self, 'validator'):
                # Mock performance comparison (in real scenario, this would compare sim vs real)
                sim_performance = {"accuracy": 0.92, "precision": 0.90, "recall": 0.91, "f1_score": 0.90}
                real_performance = {"accuracy": 0.85, "precision": 0.83, "recall": 0.84, "f1_score": 0.83}

                performance_results = self.validator.validate_performance(sim_performance, real_performance)

                # Mock behavior similarity
                sim_traj = [[0, 0], [1, 1], [2, 2], [3, 1], [4, 0]]  # Mock trajectory
                real_traj = [[0, 0], [1.1, 0.9], [2.1, 2.1], [3.1, 1.1], [4.1, 0.1]]  # Mock trajectory
                behavior_results = self.validator.validate_behavior_similarity(sim_traj, real_traj)

                # Mock safety metrics
                safety_metrics = {"collision_free": 0.98, "stability": 0.96, "limit_compliance": 0.99}
                safety_results = self.validator.validate_safety(safety_metrics)

                # Calculate overall success
                overall_results = self.validator.calculate_overall_transfer_success(
                    performance_results, behavior_results, safety_results)

                # Combine all results
                validation_results.update(performance_results)
                validation_results.update(behavior_results)
                validation_results.update(safety_results)
                validation_results.update(overall_results)

            return validation_results

        except Exception as e:
            self.get_logger().error(f'Error during validation: {e}')
            return {"error": str(e)}

    def _generate_mock_test_data(self):
        """
        Generate mock test data for transfer validation
        """
        # This would normally come from real-world testing
        # For simulation purposes, we create mock data
        return (np.random.random((100, 4)), np.random.randint(0, 2, 100))  # Mock X, y data

    def save_transfer_model(self, model, path: str):
        """
        Save the transferred model
        """
        try:
            with open(path, 'wb') as f:
                pickle.dump(model, f)
            self.get_logger().info(f'Transferred model saved to {path}')
        except Exception as e:
            self.get_logger().error(f'Error saving model: {e}')


def main(args=None):
    rclpy.init(args=args)

    # Create transfer integrator node
    transfer_node = IsaacTransferIntegrator(transfer_method="domain_randomization")

    # Example: Perform transfer (in a real scenario, this would be triggered by events)
    # transfer_node.perform_transfer("models/sim_model.pkl", "data/real_samples.pkl")

    try:
        rclpy.spin(transfer_node)
    except KeyboardInterrupt:
        pass
    finally:
        transfer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Transfer Validation and Testing
```python
# test/transfer_validation_test.py
import unittest
import numpy as np
from transfer.transfer_learning import (
    DomainRandomizationTransfer,
    DomainAdaptationTransfer,
    FineTuningTransfer,
    TransferValidator
)

class TestTransferValidation(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures"""
        self.validator = TransferValidator()

        # Mock data for testing
        self.sim_performance = {
            "accuracy": 0.92,
            "precision": 0.90,
            "recall": 0.91,
            "f1_score": 0.90
        }

        self.real_performance = {
            "accuracy": 0.85,
            "precision": 0.83,
            "recall": 0.84,
            "f1_score": 0.83
        }

    def test_performance_validation(self):
        """Test performance validation between sim and real"""
        results = self.validator.validate_performance(
            self.sim_performance,
            self.real_performance
        )

        # Check that ratios are calculated
        self.assertIn('accuracy_ratio', results)
        self.assertGreaterEqual(results['accuracy_ratio'], 0)
        self.assertLessEqual(results['accuracy_ratio'], 1.5)  # Allow for some improvement

        # Check overall score calculation
        self.assertIn('overall_performance_score', results)
        self.assertGreaterEqual(results['overall_performance_score'], 0)
        self.assertLessEqual(results['overall_performance_score'], 1)

    def test_behavior_similarity_euclidean(self):
        """Test behavior similarity using Euclidean distance"""
        traj1 = [[0, 0], [1, 1], [2, 2]]
        traj2 = [[0, 0], [1, 1], [2, 2]]  # Same trajectory

        # Temporarily set method to euclidean
        original_method = self.validator.config["transfer_validation"]["behavior_similarity"]["comparison_method"]
        self.validator.config["transfer_validation"]["behavior_similarity"]["comparison_method"] = "euclidean"

        results = self.validator.validate_behavior_similarity(traj1, traj2)

        # Same trajectories should have high similarity
        self.assertGreaterEqual(results['similarity_score'], 0.9)
        self.assertTrue(results['behavior_transfer_success'])

        # Restore original method
        self.validator.config["transfer_validation"]["behavior_similarity"]["comparison_method"] = original_method

    def test_behavior_similarity_different(self):
        """Test behavior similarity with different trajectories"""
        traj1 = [[0, 0], [1, 1], [2, 2]]
        traj2 = [[0, 1], [1, 2], [2, 3]]  # Different trajectory

        results = self.validator.validate_behavior_similarity(traj1, traj2)

        # Different trajectories should have lower similarity
        self.assertLessEqual(results['similarity_score'], 1.0)
        # But should still be a valid score
        self.assertGreaterEqual(results['similarity_score'], 0)

    def test_safety_validation(self):
        """Test safety validation"""
        safety_metrics = {
            "collision_free": 0.98,
            "stability": 0.96,
            "limit_compliance": 0.99
        }

        results = self.validator.validate_safety(safety_metrics)

        self.assertGreaterEqual(results['safety_score'], 0.95)
        self.assertTrue(results['safety_transfer_success'])

        # Test with poor safety metrics
        poor_safety = {
            "collision_free": 0.5,
            "stability": 0.6,
            "limit_compliance": 0.4
        }

        poor_results = self.validator.validate_safety(poor_safety)
        self.assertFalse(poor_results['safety_transfer_success'])

    def test_overall_transfer_success(self):
        """Test overall transfer success calculation"""
        performance_results = {
            "overall_performance_score": 0.85,
            "accuracy_ratio": 0.9
        }

        behavior_results = {
            "similarity_score": 0.90,
            "behavior_transfer_success": True
        }

        safety_results = {
            "safety_score": 0.96,
            "safety_transfer_success": True
        }

        overall_results = self.validator.calculate_overall_transfer_success(
            performance_results, behavior_results, safety_results
        )

        self.assertTrue(overall_results['transfer_success'])
        self.assertGreaterEqual(overall_results['overall_transfer_score'], 0.85)


class TestTransferMethods(unittest.TestCase):
    def test_domain_randomization_transfer(self):
        """Test domain randomization transfer method"""
        transfer_method = DomainRandomizationTransfer()

        # Mock source model
        source_model = {"type": "mock_model", "trained": True}
        target_data = (np.random.random((10, 4)), np.random.randint(0, 2, 10))

        # Transfer should return the model (domain randomization happens during data generation)
        transferred_model = transfer_method.transfer(source_model, target_data)
        self.assertEqual(transferred_model, source_model)

        # Validate transfer
        validation_results = transfer_method.validate_transfer(transferred_model, target_data)
        self.assertIn('accuracy', validation_results)
        self.assertIn('f1_score', validation_results)

    def test_domain_adaptation_transfer(self):
        """Test domain adaptation transfer method"""
        transfer_method = DomainAdaptationTransfer()

        source_model = {"type": "mock_model", "trained": True}
        target_data = [1, 2, 3, 4, 5]  # Mock target data

        # Transfer
        adapted_model = transfer_method.transfer(source_model, target_data)
        self.assertIsNotNone(adapted_model)

        # Validate
        validation_results = transfer_method.validate_transfer(adapted_model, target_data)
        self.assertIn('accuracy', validation_results)

    def test_fine_tuning_transfer(self):
        """Test fine-tuning transfer method"""
        transfer_method = FineTuningTransfer()

        source_model = {"type": "mock_model", "trained": True}
        target_data = [1, 2, 3, 4, 5]  # Mock target data

        # Transfer
        fine_tuned_model = transfer_method.transfer(source_model, target_data)
        self.assertIsNotNone(fine_tuned_model)

        # Validate
        validation_results = transfer_method.validate_transfer(fine_tuned_model, target_data)
        self.assertIn('accuracy', validation_results)


def run_transfer_validation_tests():
    """Run all transfer validation tests"""
    print("Running Transfer Validation Tests...")

    # Create test suites
    validation_suite = unittest.TestLoader().loadTestsFromTestCase(TestTransferValidation)
    transfer_suite = unittest.TestLoader().loadTestsFromTestCase(TestTransferMethods)

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)

    print("\n=== Transfer Validation Tests ===")
    runner.run(validation_suite)

    print("\n=== Transfer Method Tests ===")
    runner.run(transfer_suite)


if __name__ == '__main__':
    run_transfer_validation_tests()
```

## Usage Examples

### 1. Basic Transfer Usage
```python
# examples/basic_transfer_example.py
from transfer.transfer_learning import create_transfer_pipeline, TransferValidator

def basic_transfer_example():
    """
    Basic example of sim-to-real transfer
    """
    print("=== Basic Transfer Example ===")

    # Create transfer pipeline
    transfer_method, validator = create_transfer_pipeline("domain_randomization")

    # Mock source model (simulated trained model)
    source_model = {"model_type": "perception_model", "training_domain": "simulation", "accuracy": 0.92}

    # Mock target data (real-world samples)
    target_data = ([(1, 2, 3, 4), (5, 6, 7, 8)], [0, 1])  # X, y format

    print(f"Original model performance: {source_model['accuracy']}")

    # Perform transfer
    transferred_model = transfer_method.transfer(source_model, target_data)

    # Validate transfer
    validation_results = transfer_method.validate_transfer(transferred_model, target_data)
    print(f"Transferred model validation: {validation_results}")

def advanced_transfer_example():
    """
    Advanced example with validation
    """
    print("\n=== Advanced Transfer Example ===")

    # Create transfer pipeline with domain adaptation
    transfer_method, validator = create_transfer_pipeline("domain_adaptation")

    # Mock performance data
    sim_performance = {
        "accuracy": 0.92, "precision": 0.90, "recall": 0.91, "f1_score": 0.90
    }
    real_performance = {
        "accuracy": 0.85, "precision": 0.83, "recall": 0.84, "f1_score": 0.83
    }

    # Validate performance transfer
    performance_results = validator.validate_performance(sim_performance, real_performance)
    print(f"Performance transfer results: {performance_results}")

    # Validate behavior similarity
    sim_traj = [[0, 0], [1, 1], [2, 2], [3, 1], [4, 0]]
    real_traj = [[0, 0.1], [1.1, 1.1], [2.1, 2.1], [3.1, 1.2], [4.1, 0.1]]
    behavior_results = validator.validate_behavior_similarity(sim_traj, real_traj)
    print(f"Behavior transfer results: {behavior_results}")

    # Validate safety
    safety_metrics = {"collision_free": 0.98, "stability": 0.96, "limit_compliance": 0.99}
    safety_results = validator.validate_safety(safety_metrics)
    print(f"Safety validation results: {safety_results}")

    # Calculate overall success
    overall_results = validator.calculate_overall_transfer_success(
        performance_results, behavior_results, safety_results
    )
    print(f"Overall transfer success: {overall_results}")

if __name__ == "__main__":
    basic_transfer_example()
    advanced_transfer_example()
```

### 2. Integration with Perception Pipeline
```python
# examples/perception_transfer_example.py
from transfer.transfer_learning import create_transfer_pipeline
from perception.object_detection import IsaacROSObjectDetection
import numpy as np

class PerceptionTransferIntegrator:
    def __init__(self, transfer_method: str = "domain_randomization"):
        """
        Integrate transfer learning with perception pipeline
        """
        self.transfer_method, self.validator = create_transfer_pipeline(transfer_method)
        self.perception_model = None
        self.transferred_model = None

    def setup_perception_with_transfer(self, sim_model_path: str, real_samples_path: str = None):
        """
        Setup perception system with transfer learning
        """
        print("Setting up perception system with sim-to-real transfer...")

        # Load simulation-trained model
        self.perception_model = self.load_model(sim_model_path)

        # Apply transfer learning if real samples are available
        if real_samples_path:
            real_samples = self.load_real_samples(real_samples_path)
            self.transferred_model = self.transfer_method.transfer(self.perception_model, real_samples)
            print("Transfer learning applied to perception model")
        else:
            # Use domain randomization approach
            self.transferred_model = self.perception_model
            print("Using domain randomization approach for perception model")

        print("Perception system setup with transfer learning completed")

    def load_model(self, path: str):
        """
        Load a pre-trained model
        """
        # In real implementation, this would load the actual model
        return {"model_path": path, "type": "perception", "domain": "simulation"}

    def load_real_samples(self, path: str):
        """
        Load real-world samples for transfer
        """
        # In real implementation, this would load actual real-world data
        return [(np.random.random(640*480*3), 0) for _ in range(10)]  # Mock real samples

    def validate_perception_transfer(self):
        """
        Validate perception model transfer effectiveness
        """
        if self.transferred_model:
            # Mock validation data
            validation_data = (np.random.random((20, 640*480*3)), np.random.randint(0, 10, 20))

            validation_results = self.transfer_method.validate_transfer(
                self.transferred_model, validation_data
            )

            print(f"Perception transfer validation: {validation_results}")
            return validation_results
        else:
            print("No transferred model to validate")
            return None

def example_perception_transfer():
    """
    Example of integrating transfer learning with perception
    """
    print("=== Perception Transfer Integration Example ===")

    integrator = PerceptionTransferIntegrator("domain_randomization")

    # Setup with mock paths
    integrator.setup_perception_with_transfer(
        sim_model_path="models/sim_perception_model.pkl",
        real_samples_path="data/real_samples.pkl"
    )

    # Validate transfer
    results = integrator.validate_perception_transfer()

    print("Perception transfer integration completed successfully")

if __name__ == "__main__":
    example_perception_transfer()
```

## Performance Optimization

### 1. Transfer Performance Monitoring
```python
# transfer/performance_monitor.py
import time
from collections import deque
import numpy as np
from typing import Dict, List, Any

class TransferPerformanceMonitor:
    def __init__(self):
        """
        Monitor performance of transfer learning operations
        """
        self.transfer_times = deque(maxlen=100)
        self.validation_times = deque(maxlen=100)
        self.success_rates = deque(maxlen=100)
        self.data_efficiency_scores = deque(maxlen=100)

    def measure_transfer_time(self, transfer_func, *args, **kwargs) -> Any:
        """
        Measure time taken for transfer operation
        """
        start_time = time.time()
        result = transfer_func(*args, **kwargs)
        end_time = time.time()

        transfer_time = end_time - start_time
        self.transfer_times.append(transfer_time)

        return result

    def measure_validation_time(self, validation_func, *args, **kwargs) -> Any:
        """
        Measure time taken for validation operation
        """
        start_time = time.time()
        result = validation_func(*args, **kwargs)
        end_time = time.time()

        validation_time = end_time - start_time
        self.validation_times.append(validation_time)

        return result

    def log_success_rate(self, success: bool):
        """
        Log success rate of transfer operations
        """
        self.success_rates.append(1.0 if success else 0.0)

    def log_data_efficiency(self, efficiency_score: float):
        """
        Log data efficiency of transfer operations
        """
        self.data_efficiency_scores.append(efficiency_score)

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get comprehensive performance metrics
        """
        metrics = {}

        if self.transfer_times:
            metrics["avg_transfer_time"] = float(np.mean(self.transfer_times))
            metrics["min_transfer_time"] = float(np.min(self.transfer_times))
            metrics["max_transfer_time"] = float(np.max(self.transfer_times))
            metrics["transfer_time_std"] = float(np.std(self.transfer_times))

        if self.validation_times:
            metrics["avg_validation_time"] = float(np.mean(self.validation_times))
            metrics["min_validation_time"] = float(np.min(self.validation_times))
            metrics["max_validation_time"] = float(np.max(self.validation_times))

        if self.success_rates:
            metrics["avg_success_rate"] = float(np.mean(self.success_rates))
            metrics["success_rate_std"] = float(np.std(self.success_rates))

        if self.data_efficiency_scores:
            metrics["avg_data_efficiency"] = float(np.mean(self.data_efficiency_scores))
            metrics["data_efficiency_std"] = float(np.std(self.data_efficiency_scores))

        return metrics

def monitor_transfer_performance():
    """
    Example of monitoring transfer performance
    """
    monitor = TransferPerformanceMonitor()

    # Example: Simulate transfer operations
    for i in range(10):
        # Simulate transfer operation
        time.sleep(0.1)  # Simulate processing time
        monitor.log_success_rate(True)
        monitor.log_data_efficiency(0.75 + np.random.random() * 0.2)

    metrics = monitor.get_performance_metrics()
    print("Transfer Performance Metrics:")
    for key, value in metrics.items():
        print(f"  {key}: {value:.4f}" if isinstance(value, float) else f"  {key}: {value}")

if __name__ == "__main__":
    monitor_transfer_performance()
```

## Best Practices and Guidelines

### 1. Transfer Learning Best Practices
```python
# transfer/best_practices.py
class TransferLearningBestPractices:
    """
    Best practices and guidelines for effective sim-to-real transfer
    """

    @staticmethod
    def domain_randomization_best_practices():
        """
        Best practices for domain randomization
        """
        practices = [
            "Vary textures, lighting, and physics parameters widely enough to cover real-world variations",
            "Ensure randomization ranges are realistic and physically plausible",
            "Use sufficient variation to make the model robust, but not so much that it loses focus",
            "Validate that randomization doesn't make the simulation too unrealistic",
            "Monitor training performance to ensure the model is learning meaningful features",
            "Start with moderate randomization and gradually increase",
            "Use curriculum learning to gradually increase randomization complexity"
        ]
        return practices

    @staticmethod
    def domain_adaptation_best_practices():
        """
        Best practices for domain adaptation
        """
        practices = [
            "Use a small but representative set of real-world data for adaptation",
            "Preserve important features from the source domain while adapting to the target",
            "Monitor for negative transfer where adaptation hurts performance",
            "Use appropriate adaptation techniques based on the domain gap",
            "Validate adaptation results on separate test sets",
            "Consider the cost of collecting real data vs. adaptation benefits"
        ]
        return practices

    @staticmethod
    def fine_tuning_best_practices():
        """
        Best practices for fine-tuning
        """
        practices = [
            "Use a lower learning rate for fine-tuning than initial training",
            "Freeze early layers and only fine-tune later layers initially",
            "Monitor for overfitting on the small real dataset",
            "Use data augmentation to increase effective dataset size",
            "Validate on a held-out real dataset to prevent overfitting",
            "Consider using techniques like dropout and batch normalization appropriately"
        ]
        return practices

    @staticmethod
    def validation_best_practices():
        """
        Best practices for transfer validation
        """
        practices = [
            "Test on diverse real-world scenarios, not just the training conditions",
            "Use multiple metrics to evaluate transfer effectiveness",
            "Validate safety-critical behaviors separately",
            "Test long-term stability and reliability, not just immediate performance",
            "Compare against appropriate baselines",
            "Document and analyze failure cases to improve future transfers"
        ]
        return practices

def print_transfer_best_practices():
    """
    Print all transfer learning best practices
    """
    practices = TransferLearningBestPractices()

    print("=== Sim-to-Real Transfer Best Practices ===\n")

    print("Domain Randomization:")
    for i, practice in enumerate(practices.domain_randomization_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nDomain Adaptation:")
    for i, practice in enumerate(practices.domain_adaptation_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nFine-tuning:")
    for i, practice in enumerate(practices.fine_tuning_best_practices(), 1):
        print(f"  {i}. {practice}")

    print("\nValidation:")
    for i, practice in enumerate(practices.validation_best_practices(), 1):
        print(f"  {i}. {practice}")

if __name__ == "__main__":
    print_transfer_best_practices()
```

## Troubleshooting

### Common Issues and Solutions

#### 1. Transfer Performance Issues
- **Large Domain Gap**: Increase domain randomization or collect more real data
- **Negative Transfer**: Reduce learning rate, use fewer adaptation steps, or validate intermediate results
- **Overfitting to Real Data**: Use regularization, data augmentation, or early stopping

#### 2. Validation Issues
- **Poor Real-World Performance**: Check for data drift, revalidate sim-to-real assumptions
- **Safety Violations**: Implement more rigorous safety validation, reduce transfer aggressiveness
- **Inconsistent Results**: Ensure proper random seed management, validate data quality

#### 3. Integration Issues
- **Isaac Sim Compatibility**: Verify Isaac Sim and Isaac ROS version compatibility
- **Performance Degradation**: Optimize transfer algorithms, validate computational requirements
- **Data Format Issues**: Ensure consistent data formats between sim and real systems

## Next Steps
After implementing sim-to-real transfer techniques:
1. Integrate all components into complete autonomous system (Task 3.3)
2. Optimize overall system performance (Task 3.4)
3. Conduct comprehensive testing and validation (Phase 4)

## Resources
- [NVIDIA Isaac Sim Transfer Learning](https://nvidia-isaac-ros.github.io/concepts/sim2real/index.html)
- [Domain Randomization Research](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Overview](https://arxiv.org/abs/1802.01557)
- [Transfer Learning in Robotics](https://arxiv.org/abs/1909.11622)