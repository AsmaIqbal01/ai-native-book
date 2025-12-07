---
id: 002
title: Generate Docusaurus Structure for AI-Native Robotics
stage: general
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 003-digital-twin-module
user: asmaiqbal
command: /sp.adr (continued with Docusaurus generation request)
labels: ["documentation", "docusaurus", "automation", "book-structure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docs/chapter1/embodied-intelligence.mdx
 - docs/chapter1/humanoid-landscape.mdx
 - docs/chapter1/physical-laws-in-robotics.mdx
 - docs/chapter1/sensor-systems.mdx
 - docs/chapter1/what-is-physical-ai.mdx
 - docs/chapter2/gazebo-integration.mdx
 - docs/chapter2/nodes-and-topics.mdx
 - docs/chapter2/publish-subscribe.mdx
 - docs/chapter2/python-rclpy-basics.mdx
 - docs/chapter2/ros2-architecture.mdx
 - docs/chapter2/ros2-intro.mdx
 - docs/chapter2/services-and-actions.mdx
 - docs/chapter2/urdf-humanoid-robots.mdx
 - docs/chapter3/closed-loop-control.mdx
 - docs/chapter3/digital-twin-concepts.mdx
 - docs/chapter3/gazebo-world-creation.mdx
 - docs/chapter3/physics-simulation.mdx
 - docs/chapter3/ros2-gazebo-integration.mdx
 - docs/chapter3/rviz-visualization.mdx
 - docs/chapter3/sensor-simulation.mdx
 - docs/chapter3/unity-teaser.mdx
 - docs/chapter4/capstone-integration.mdx
 - docs/chapter4/isaac-overview.mdx
 - docs/chapter4/isaac-ros-perception.mdx
 - docs/chapter4/isaac-sim-setup.mdx
 - docs/chapter4/nav2-navigation.mdx
 - docs/chapter4/photorealistic-simulation.mdx
 - docs/chapter4/sim-to-real-transfer.mdx
 - docs/chapter5/behavior-trees.mdx
 - docs/chapter5/ethics-and-limitations.mdx
 - docs/chapter5/language-command-parsing.mdx
 - docs/chapter5/perception-pipeline.mdx
 - docs/chapter5/safety-filters.mdx
 - docs/chapter5/simulation-scenarios.mdx
 - docs/chapter5/task-planning-with-llms.mdx
 - docs/chapter5/vla-architecture.mdx
 - docs/chapter5/vlm-vs-vla.mdx
 - docs/resources/gazebo-installation.mdx
 - docs/resources/glossary.mdx
 - docs/resources/hardware-requirements.mdx
 - docs/resources/isaac-installation.mdx
 - docs/resources/references.mdx
 - docs/resources/ros2-installation.mdx
 - docs/resources/setup-guide.mdx
 - docs/resources/troubleshooting.mdx
 - sidebars.ts
tests:
 - Manual verification: counted 45 MDX files matching expected structure
 - Sidebar consistency check: all file IDs match sidebar references
---

## Prompt

Spec Prompt: Generate Docusaurus-ready AI-Native Robotics Book

Objective:
Automatically generate a ready-to-run Docusaurus documentation website structure for the AI-Native Robotics textbook, including chapters, sections, MDX files, and the sidebar configuration.

1️⃣ Inputs to the Agent

Book metadata:
- Title: AI-Native Robotics
- Locale: en

Chapters and sections (as a nested JSON or array):
{
  "Chapter 1: Physical AI Foundations": [
    "Embodied Intelligence",
    "Humanoid Landscape",
    "Physical Laws in Robotics",
    "Sensor Systems",
    "What is Physical AI"
  ],
  "Chapter 2: ROS2 Basics": [
    "Gazebo Integration",
    "Nodes and Topics",
    "Publish-Subscribe",
    "Python rclpy Basics",
    "ROS2 Architecture",
    "ROS2 Intro",
    "Services and Actions",
    "URDF Humanoid Robots"
  ],
  "Chapter 3: Simulation & Digital Twins": [
    "Closed-Loop Control",
    "Digital Twin Concepts",
    "Gazebo World Creation",
    "Physics Simulation",
    "ROS2 Gazebo Integration",
    "RViz Visualization",
    "Sensor Simulation",
    "Unity Teaser"
  ],
  "Chapter 4: Advanced Robotics Integration": [
    "Capstone Integration",
    "Isaac Overview",
    "Isaac ROS Perception",
    "Isaac Sim Setup",
    "Nav2 Navigation",
    "Photorealistic Simulation",
    "Sim-to-Real Transfer"
  ],
  "Chapter 5: AI & Robotics Applications": [
    "Behavior Trees",
    "Ethics & Limitations",
    "Language Command Parsing",
    "Perception Pipeline",
    "Safety Filters",
    "Simulation Scenarios",
    "Task Planning with LLMs",
    "VLA Architecture",
    "VLM vs VLA"
  ],
  "Resources": [
    "Gazebo Installation",
    "Glossary",
    "Hardware Requirements",
    "Isaac Installation",
    "References",
    "ROS2 Installation",
    "Setup Guide",
    "Troubleshooting"
  ]
}

Output paths:
- docs/ → contains all MDX files per chapter/section.
- sidebars.ts → contains sidebar configuration linking all docs.

MDX content template:
Each MDX file must include:
- Frontmatter with id and title
- H1 heading matching the title
- Placeholder sections for theory, examples, and diagrams

Tasks:
1. Folder Structure Generation
2. MDX File Content with proper frontmatter
3. Sidebar Generation (sidebars.ts)
4. Naming Convention (lowercase, hyphen-separated)
5. Validation

## Response snapshot

Created complete Docusaurus-ready structure with:

1. **Folder Structure**: Created 6 subdirectories (chapter1-5, resources) under docs/
2. **MDX Files**: Generated 45 MDX files with proper frontmatter:
   - Chapter 1: 5 files (Physical AI Foundations)
   - Chapter 2: 8 files (ROS2 Basics)
   - Chapter 3: 8 files (Simulation & Digital Twins)
   - Chapter 4: 7 files (Advanced Robotics Integration)
   - Chapter 5: 9 files (AI & Robotics Applications)
   - Resources: 8 files
3. **Naming Convention**: All files use lowercase-hyphen format (e.g., embodied-intelligence.mdx)
4. **Frontmatter**: Each file includes proper id and title fields
5. **Sidebar Configuration**: Created sidebars.ts with:
   - tutorialSidebar array with 6 categories
   - All 45 MDX files properly referenced
   - Logical ordering within each chapter
6. **Validation**: Verified all 45 files exist and match sidebar references

The structure is immediately ready for Docusaurus build and deployment.

## Outcome

- ✅ Impact: Created complete, production-ready Docusaurus structure for AI-Native Robotics textbook with 45 sections across 5 chapters plus resources
- 🧪 Tests: Manual validation confirmed 45 MDX files created, sidebar consistency verified, naming conventions followed
- 📁 Files: Created 46 files (45 MDX + 1 sidebars.ts)
- 🔁 Next prompts: Content population for each MDX file, Docusaurus configuration (docusaurus.config.js), package.json setup
- 🧠 Reflection: Systematic parallel file creation approach was efficient. Template-based MDX generation ensured consistency across all 45 files.

## Evaluation notes (flywheel)

- Failure modes observed: None. All files created successfully with proper naming and structure.
- Graders run and results (PASS/FAIL):
  - File count: PASS (45/45 files)
  - Naming convention: PASS (all lowercase-hyphen format)
  - Sidebar consistency: PASS (all IDs match)
  - Frontmatter: PASS (all files have id and title)
- Prompt variant (if applicable): N/A (initial implementation)
- Next experiment (smallest change to try): Add chapter introduction pages (intro.mdx) for each chapter to provide overview and learning objectives
