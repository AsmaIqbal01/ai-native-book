import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main tutorial sidebar for AI-Native Robotics
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapter 1: Physical AI Foundations',
      collapsed: false,
      items: [
        'chapter1/embodied-intelligence',
        'chapter1/humanoid-landscape',
        'chapter1/physical-laws-in-robotics',
        'chapter1/sensor-systems',
        'chapter1/what-is-physical-ai',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS2 Basics',
      collapsed: false,
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
    {
      type: 'category',
      label: 'Chapter 3: Simulation & Digital Twins',
      collapsed: false,
      items: [
        'chapter3/digital-twin-concepts',
        'chapter3/gazebo-world-creation',
        'chapter3/physics-simulation',
        'chapter3/sensor-simulation',
        'chapter3/ros2-gazebo-integration',
        'chapter3/rviz-visualization',
        'chapter3/closed-loop-control',
        'chapter3/unity-teaser',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: Advanced Robotics Integration',
      collapsed: false,
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
    {
      type: 'category',
      label: 'Chapter 5: AI & Robotics Applications',
      collapsed: false,
      items: [
        'chapter5/perception-pipeline',
        'chapter5/behavior-trees',
        'chapter5/task-planning-with-llms',
        'chapter5/language-command-parsing',
        'chapter5/vlm-vs-vla',
        'chapter5/vla-architecture',
        'chapter5/safety-filters',
        'chapter5/simulation-scenarios',
        'chapter5/ethics-and-limitations',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: false,
      items: [
        'resources/setup-guide',
        'resources/hardware-requirements',
        'resources/ros2-installation',
        'resources/gazebo-installation',
        'resources/isaac-installation',
        'resources/troubleshooting',
        'resources/glossary',
        'resources/references',
      ],
    },
  ],
};

export default sidebars;
