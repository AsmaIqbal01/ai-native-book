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
        'chapter1/physical-ai',
        'chapter1/digital-to-physical',
        'chapter1/embodied-intelligence',
        'chapter1/humanoid-landscape',
        'chapter1/physical-laws-in-robotics',
        'chapter1/sensor-systems',
      ],
    },

    // Resources and Appendices
    {
      type: 'category',
      label: '🛠️ Resources',
      collapsed: true,
      items: [
        'resources/setup-guide',
        'resources/hardware-requirements',
        'resources/ros2-installation',
        'resources/isaac-installation',
        'resources/troubleshooting',
        'resources/glossary',
        'resources/references',
      ],
    },
  ],
};

export default sidebars;
