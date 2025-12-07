# AI-Native Robotics Textbook

An educational textbook teaching robotics fundamentals with a focus on AI integration, humanoid robots, and modern simulation tools.

## Overview

This textbook provides hands-on learning experiences in:
- **Physical AI & Embodied Intelligence**: Understanding robots that interact with the physical world
- **ROS 2 Fundamentals**: Nodes, topics, services, actions, and URDF
- **Simulation-First Approach**: Safe learning with Gazebo Classic and NVIDIA Isaac Sim
- **AI Integration**: Bridging Python agents to robot controllers
- **Vision-Language-Action Systems**: Modern multimodal AI for robotics
- **Humanoid Robotics**: Custom simplified humanoid models for education

## Final Book Structure

This textbook follows a progressive, hands-on curriculum:

1. **Chapter 1: Introduction to Physical AI** - Foundational concepts, embodied intelligence, humanoid robotics landscape
2. **Chapter 2 / Module 1: The Robotic Nervous System (ROS 2)** - ROS 2 fundamentals, nodes, topics, services, actions
3. **Chapter 3 / Module 2: The Digital Twin (Gazebo & Unity)** - Simulation with Gazebo Classic, sensor integration
4. **Chapter 4 / Module 3: The AI-Robot Brain (NVIDIA Isaac™)** - Advanced perception, Isaac Sim, Isaac ROS
5. **Chapter 5 / Module 4: Vision-Language-Action (VLA)** - LLM-driven robot control, multimodal AI

## Project Structure

```
ai-native-book/
├── docs/modules/                      # Textbook chapters/modules
├── specs/
│   ├── 001-physical-ai-intro/         # Chapter 1 specification
│   ├── 002-ros2-module/               # Chapter 2 / Module 1 specification
│   ├── 003-digital-twin-module/       # Chapter 3 / Module 2 specification
│   ├── 004-isaac-module/              # Chapter 4 / Module 3 specification
│   └── 005-vla-module/                # Chapter 5 / Module 4 specification
├── examples/
│   ├── ros2-module/                   # Module 1 code examples
│   ├── digital-twin-module/           # Module 2 code examples
│   ├── 004-isaac-module/              # Module 3 code examples
│   └── 005-vla-module/                # Module 4 code examples
├── assets/diagrams/                   # ASCII architecture diagrams
├── .specify/memory/constitution.md    # Project principles and standards
└── history/prompts/                   # Development history (PHRs)
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

### Chapter 1: Introduction to Physical AI
**Status**: ✅ Specification Complete

Foundational concepts including:
- What is Physical AI and embodied intelligence
- From digital AI to robots that understand physics
- Humanoid robotics landscape (Tesla Optimus, Figure 01, Unitree H1, Agility Digit)
- Sensor systems in humanoid robots (LiDAR, cameras, IMU, force/torque)

### Chapter 2 / Module 1: The Robotic Nervous System (ROS 2)
**Status**: ✅ Specification Complete | 🚧 Implementation In Progress

Learn ROS 2 fundamentals including:
- ROS 2 architecture and computation graph
- Creating publisher/subscriber nodes in Python
- Services and actions for robot control
- Bridging Python AI agents to ROS controllers
- URDF for humanoid robot modeling (SimpleHumanoid 5-7 DOF)

### Chapter 3 / Module 2: The Digital Twin (Gazebo & Unity)
**Status**: ✅ Specification Complete | 🚧 Implementation In Progress

Learn to create and use Digital Twins:
- Simulate physics, sensors (LiDAR, Depth Camera, IMU) in Gazebo Classic
- Create custom Gazebo worlds and spawn robots
- Integrate simulated sensors with ROS 2 nodes
- Unity visualization (teaser with external resources)

### Chapter 4 / Module 3: The AI-Robot Brain (NVIDIA Isaac™)
**Status**: ✅ Specification Complete

Advanced perception and AI training:
- NVIDIA Isaac Sim for photorealistic simulation
- Isaac ROS for hardware-accelerated perception
- Nav2 navigation and path planning
- Sim-to-real transfer techniques

### Chapter 5 / Module 4: Vision-Language-Action (VLA)
**Status**: ✅ Specification Complete

Modern multimodal AI for robotics:
- VLA system architecture (vision → language → action)
- Robot perception pipeline (RGB, depth, segmentation)
- LLM-based high-level task planning with safety filters
- Complete VLA scenarios in simulation

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

Built using [Specify]((https://github.com/panaversity/spec-kit-plus)] - Spec-Driven Development workflow.

Educational content follows constitution-driven quality standards:
- ✅ All code examples tested in simulation
- ✅ Beginner-friendly explanations
- ✅ Safety-first approach (simulation-only)
- ✅ Consistent terminology throughout

---

**Version**: 0.1.0 (Initial Development)
**Last Updated**: 2025-12-07
