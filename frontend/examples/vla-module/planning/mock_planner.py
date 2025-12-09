#!/usr/bin/env python3
"""
Mock Planner for VLA Systems (Offline Fallback)

Provides template-based planning for students without LLM API access.

Tasks: T088, T044
"""

import rclpy
from rclpy.node import Node
import json


class MockPlanner(Node):
    """
    Mock planner using predefined response templates.
    Enables offline learning without API costs.
    """

    def __init__(self):
        super().__init__('mock_planner')

        # Load response templates
        self.templates = self.load_templates()

        self.get_logger().info('Mock Planner initialized (offline mode)')

    def load_templates(self):
        """
        Load predefined response templates for common scenarios.

        Returns:
            dict: Templates keyed by command patterns
        """
        # TODO: Load from JSON file or define comprehensive templates
        templates = {
            'navigate_to': {
                "task_type": "navigate",
                "parameters": {
                    "target": "{target}",
                    "approach_distance": 0.5
                },
                "safety_constraints": {
                    "max_velocity": 0.5,
                    "collision_avoidance": True
                },
                "preconditions": ["robot_localized", "path_clear"]
            },
            'approach_object': {
                "task_type": "approach",
                "parameters": {
                    "object": "{object}",
                    "final_distance": 0.3
                },
                "safety_constraints": {
                    "max_velocity": 0.3,
                    "collision_avoidance": True
                },
                "preconditions": ["object_detected", "path_clear"]
            },
            'align_with': {
                "task_type": "align",
                "parameters": {
                    "target_object": "{object}",
                    "alignment_axis": "forward"
                },
                "safety_constraints": {
                    "max_angular_velocity": 0.2
                },
                "preconditions": ["near_object"]
            }
        }

        return templates

    def generate_plan(self, observation, command):
        """
        Generate plan using template matching.

        Args:
            observation (dict): Scene observations
            command (str): Natural language command

        Returns:
            dict: Generated plan from template
        """
        # Simple keyword matching
        command_lower = command.lower()

        # TODO: Implement more sophisticated matching
        if any(word in command_lower for word in ['move', 'go', 'navigate']):
            template = self.templates['navigate_to'].copy()
            # Extract target from command
            target = self.extract_target(command)
            template['parameters']['target'] = target
            return template

        elif any(word in command_lower for word in ['approach', 'get close']):
            template = self.templates['approach_object'].copy()
            obj = self.extract_object(command, observation)
            template['parameters']['object'] = obj
            return template

        elif any(word in command_lower for word in ['align', 'face']):
            template = self.templates['align_with'].copy()
            obj = self.extract_object(command, observation)
            template['parameters']['target_object'] = obj
            return template

        else:
            self.get_logger().warn(f'No template match for command: {command}')
            return {}

    def extract_target(self, command):
        """Extract target location/object from command."""
        # TODO: Implement extraction logic
        # Simple placeholder: return last word
        words = command.strip().split()
        return words[-1] if words else "unknown"

    def extract_object(self, command, observation):
        """Extract object name from command and observations."""
        # TODO: Match command keywords with observation objects
        # Simple placeholder
        if 'objects' in observation:
            return observation['objects'][0] if observation['objects'] else "unknown"
        return "unknown"


def main(args=None):
    rclpy.init(args=args)
    node = MockPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
