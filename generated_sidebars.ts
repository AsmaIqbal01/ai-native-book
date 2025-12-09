// Generated automatically by Qwen. Do not edit manually.
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
    '13-week-strategy.mdx',
    'book-introduction.mdx',
    'book-summary-roadmap.mdx',
    'glossary.mdx',
    'introduction.mdx',
    {
      type: 'category',
      label: '001. Chapter',
      collapsed: false,
      items: [
        'chapter1/physical-ai.mdx',
        'chapter1/embodied-intelligence.mdx',
        'chapter1/physical-laws-in-robotics.mdx',
        'chapter1/sensor-systems.mdx',
        'chapter1/digital-to-physical.mdx',
        'chapter1/humanoid-landscape.mdx',
        {
          type: 'category',
          label: 'Archive',
          collapsed: true,
          items: [
            'chapter1/archive/digital-to-physical.mdx',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: '002. Chapter',
      collapsed: false,
      items: [
        'chapter2/ros2-architecture.mdx',
        'chapter2/nodes-and-topics.mdx',
        'chapter2/publish-subscribe.mdx',
        'chapter2/services-and-actions.mdx',
        'chapter2/python-rclpy-basics.mdx',
        'chapter2/creating-running-nodes.mdx',
        'chapter2/urdf-humanoid-robots.mdx',
        'chapter2/gazebo-integration.mdx',
        'chapter2/chapter-plan-ros2.mdx',
      ],
    },
    {
      type: 'category',
      label: '003. Chapter',
      collapsed: false,
      items: [
        'chapter3/digital-twin-concepts.mdx',
        'chapter3/gazebo-world-creation.mdx',
        'chapter3/physics-simulation.mdx',
        'chapter3/sensor-simulation.mdx',
        'chapter3/ros2-gazebo-integration.mdx',
        'chapter3/rviz-visualization.mdx',
        'chapter3/unity-teaser.mdx',
        'chapter3/closed-loop-control.mdx',
        'chapter3/chapter3-plan.mdx',
      ],
    },
    {
      type: 'category',
      label: '004. Chapter',
      collapsed: false,
      items: [
        'chapter4/isaac-overview.mdx',
        'chapter4/isaac-sim-setup.mdx',
        'chapter4/photorealistic-simulation.mdx',
        'chapter4/isaac-ros-perception.mdx',
        'chapter4/isaac-ros-perception-pipeline.mdx',
        'chapter4/navigation-path-planning.mdx',
        'chapter4/nav2-navigation.mdx',
        'chapter4/nvidia-isaac-overview.mdx',
        'chapter4/isaac-sim-advanced-training.mdx',
        'chapter4/sim-to-real-transfer.mdx',
        'chapter4/capstone-integration.mdx',
        'chapter4/chapter4-plan.mdx',
      ],
    },
    {
      type: 'category',
      label: '005. Chapter',
      collapsed: false,
      items: [
        'chapter5/perception-pipeline.mdx',
        'chapter5/behavior-trees.mdx',
        'chapter5/task-planning-with-llms.mdx',
        'chapter5/language-command-parsing.mdx',
        'chapter5/vlm-vs-vla.mdx',
        'chapter5/safety-filters.mdx',
        'chapter5/simulation-scenarios.mdx',
        'chapter5/ethics-and-limitations.mdx',
        'chapter5/multimodal-ai-overview.mdx',
        'chapter5/multimodal-ai-models.mdx',
        'chapter5/vision-processing-vla.mdx',
        'chapter5/vla-introduction.mdx',
        'chapter5/vla-architecture.mdx',
        'chapter5/vla-conclusion.mdx',
        'chapter5/ros2-isaac-implementation.mdx',
        'chapter5/sensor-fusion-pipelines.mdx',
        'chapter5/natural-language-understanding.mdx',
        'chapter5/chapter5-plan.mdx',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 Vla',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Ch01 Introduction',
          collapsed: true,
          items: [
            'module4-vla/ch01-introduction/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch02 Perception',
          collapsed: true,
          items: [
            'module4-vla/ch02-perception/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch03 Language',
          collapsed: true,
          items: [
            'module4-vla/ch03-language/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch04 Architecture',
          collapsed: true,
          items: [
            'module4-vla/ch04-architecture/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch05 Planning',
          collapsed: true,
          items: [
            'module4-vla/ch05-planning/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch06 Execution',
          collapsed: true,
          items: [
            'module4-vla/ch06-execution/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch07 Scenarios',
          collapsed: true,
          items: [
            'module4-vla/ch07-scenarios/index.md',
          ],
        },
        {
          type: 'category',
          label: 'Ch08 Safety Ethics',
          collapsed: true,
          items: [
            'module4-vla/ch08-safety-ethics/index.md',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Modules',
      collapsed: false,
      items: [
        'modules/digital-twin-module.md',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: false,
      items: [
        'resources/setup-guide.mdx',
        'resources/hardware-requirements.mdx',
        'resources/ros2-installation.mdx',
        'resources/gazebo-installation.mdx',
        'resources/isaac-installation.mdx',
        'resources/troubleshooting.mdx',
        'resources/glossary.mdx',
        'resources/references.mdx',
      ],
    },
  ],
};

export default sidebars;