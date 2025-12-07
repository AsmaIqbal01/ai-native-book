import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * AI-Native Robotics Textbook Sidebar
 *
 * Structure:
 * - 5 main chapters covering physical AI, ROS2, simulation, NVIDIA Isaac, and VLA systems
 * - Chapter 1 expanded by default for new readers
 * - Sequential learning path with clear prerequisites
 * - Resources section for setup and troubleshooting
 */

const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',

    // Chapter 1: Introduction to Physical AI (Foundation)
    {
      type: 'category',
      label: '📖 Chapter 1: Introduction to Physical AI',
      collapsed: false,
      items: [
        'chapter1/what-is-physical-ai',
        'chapter1/embodied-intelligence',
        'chapter1/physical-laws-in-robotics',
        'chapter1/humanoid-landscape',
        'chapter1/sensor-systems',
      ],
    },

    // Chapter 2: ROS 2 (Robotic Nervous System)
    {
      type: 'category',
      label: '🧠 Chapter 2: The Robotic Nervous System (ROS 2)',
      collapsed: true,
      items: [
        'chapter2/ros2-intro',
        'chapter2/ros2-architecture',
        'chapter2/nodes-and-topics',
        'chapter2/publish-subscribe',
        'chapter2/services-and-actions',
        'chapter2/python-rclpy-basics',
        'chapter2/urdf-humanoid-robots',
        'chapter2/gazebo-integration',
      ],
    },

    // Chapter 3: Digital Twin (Gazebo & Unity)
    {
      type: 'category',
      label: '🌐 Chapter 3: The Digital Twin',
      collapsed: true,
      items: [
        'chapter3/digital-twin-concepts',
        'chapter3/gazebo-world-creation',
        'chapter3/sensor-simulation',
        'chapter3/physics-simulation',
        'chapter3/closed-loop-control',
        'chapter3/ros2-gazebo-integration',
        'chapter3/rviz-visualization',
        'chapter3/unity-teaser',
      ],
    },

    // Chapter 4: NVIDIA Isaac (AI-Robot Brain)
    {
      type: 'category',
      label: '🚀 Chapter 4: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: true,
      items: [
        'chapter4/isaac-overview',
        'chapter4/isaac-sim-setup',
        'chapter4/photorealistic-simulation',
        'chapter4/isaac-ros-perception',
        'chapter4/nav2-navigation',
        'chapter4/sim-to-real-transfer',
        'chapter4/capstone-integration',
      ],
    },

    // Chapter 5: Vision-Language-Action (VLA) Systems
    {
      type: 'category',
      label: '👁️ Chapter 5: Vision-Language-Action Systems',
      collapsed: true,
      items: [
        'chapter5/vla-architecture',
        'chapter5/vlm-vs-vla',
        'chapter5/perception-pipeline',
        'chapter5/language-command-parsing',
        'chapter5/task-planning-with-llms',
        'chapter5/safety-filters',
        'chapter5/behavior-trees',
        'chapter5/simulation-scenarios',
        'chapter5/ethics-and-limitations',
      ],
    },

    // Resources and Appendices
    {
      type: 'category',
      label: '🛠️ Resources',
      collapsed: true,
      items: [
        'resources/setup-guide',
        'resources/ros2-installation',
        'resources/gazebo-installation',
        'resources/isaac-installation',
        'resources/troubleshooting',
        'resources/hardware-requirements',
        'resources/references',
        'resources/glossary',
      ],
    },
  ],
};

export default sidebars;
