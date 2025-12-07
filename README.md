# AI-Native Robotics Textbook

An educational textbook teaching robotics fundamentals with a focus on AI integration, humanoid robots, and modern simulation tools.

## Overview

This textbook provides hands-on learning experiences in:
- **Physical AI & Embodied Intelligence**: Understanding robots that interact with the physical world
- **ROS 2 Fundamentals**: Nodes, topics, services, actions, and URDF
- **Simulation-First Approach**: Safe learning with Gazebo Classic
- **AI Integration**: Bridging Python agents to robot controllers
- **Humanoid Robotics**: Custom simplified humanoid models for education

## Project Structure

```
ai-native-book/
├── docs/modules/              # Textbook chapters/modules
├── examples/ros2-module/      # Module 1 examples
├── examples/digital-twin-module/ # Module 2 examples
├── assets/diagrams/           # ASCII architecture diagrams
├── specs/                     # Feature specifications
└── history/prompts/           # Development history (PHRs)
```

## Setup Instructions

### Prerequisites

- **OS**: Ubuntu 22.04 LTS (or WSL2/Docker for Windows/macOS)
- **ROS 2**: Humble Hawksbill or Iron Irwini
- **Gazebo**: Classic 11.x
- **Python**: 3.8+

### Installation

Detailed setup instructions are available in [SETUP.md](./SETUP.md).

Quick start:
```bash
# Install ROS 2 Humble (Ubuntu 22.04)
# See SETUP.md for complete instructions

# Clone repository
git clone https://github.com/AsmaIqbal01/ai-native-book.git
cd ai-native-book

# Install Python dependencies (if any)
pip install -r requirements.txt  # (when available)
```

### Running Examples

Each module includes runnable ROS 2 examples. See individual module documentation for usage:

**Module 1: The Robotic Nervous System (ROS 2)**
```bash
# Example: Run simple publisher
cd examples/ros2-module/nodes
python3 simple_publisher.py

# Verify in another terminal
ros2 topic echo /chatter
```

See [docs/modules/ros2-nervous-system.md](./docs/modules/ros2-nervous-system.md) for complete module content.

## Modules

### Module 1: The Robotic Nervous System (ROS 2)
Learn ROS 2 fundamentals including:
- ROS 2 architecture and computation graph
- Creating publisher/subscriber nodes in Python
- Services and actions for robot control
- Bridging Python AI agents to ROS controllers
- URDF for humanoid robot modeling

**Status**: 🚧 In Development

### Module 2: The Digital Twin (Gazebo & Unity)
Learn to create and use Digital Twins for humanoid robotics using Gazebo Classic, with an introduction to Unity.
- Simulate physics, sensors (LiDAR, Depth Camera, IMU), and robot-environment interactions.
- Integrate simulated sensors with ROS 2 nodes.

**Status**: 🚧 In Development

## Troubleshooting

Common issues and solutions are documented in [TROUBLESHOOTING.md](./TROUBLESHOOTING.md).

For ROS 2 installation problems, environment setup, or code example errors, refer to the troubleshooting guide.

## Contributing

This is an educational project. Contributions welcome:
- Content improvements and clarifications
- Additional code examples
- Bug fixes in existing examples
- Documentation enhancements

Please ensure all code examples:
- Follow PEP 8 style guidelines
- Include inline comments explaining robotics concepts
- Are tested in Gazebo simulation before submission
- Include verification instructions (expected output, ros2 commands)

## License

[License information to be added]

## Acknowledgments

Built using [Specify](https://github.com/anthropics/specify) - Spec-Driven Development workflow.

Educational content follows constitution-driven quality standards:
- ✅ All code examples tested in simulation
- ✅ Beginner-friendly explanations
- ✅ Safety-first approach (simulation-only)
- ✅ Consistent terminology throughout

---

**Version**: 0.1.0 (Initial Development)
**Last Updated**: 2025-12-07
