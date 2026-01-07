/**
 * Bulk Translation Script
 *
 * Translates all English docs to Urdu and outputs to i18n/ur directory
 */

const fs = require('fs');
const path = require('path');
const { UrduTranslator } = require('./urdu-translator');

const DOCS_DIR = path.join(__dirname, '..', 'docs');
const OUTPUT_DIR = path.join(__dirname, '..', 'i18n', 'ur', 'docusaurus-plugin-content-docs', 'current');

// Files to translate (excluding introduction.mdx which already exists)
const FILES_TO_TRANSLATE = [
  '13-week-strategy.mdx',
  'book-introduction.mdx',
  'book-summary-roadmap.mdx',
  'glossary.mdx',

  // Chapter 1
  'chapter1/digital-to-physical.mdx',
  'chapter1/embodied-intelligence.mdx',
  'chapter1/humanoid-landscape.mdx',
  'chapter1/physical-ai.mdx',
  'chapter1/physical-laws-in-robotics.mdx',
  'chapter1/sensor-systems.mdx',

  // Chapter 2
  'chapter2/chapter2-plan.mdx',
  'chapter2/creating-running-nodes.mdx',
  'chapter2/ros2-architecture.mdx',
  'chapter2/services-and-actions.mdx',
  'chapter2/urdf-humanoid-robots.mdx',

  // Chapter 3
  'chapter3/chapter3-plan.mdx',
  'chapter3/closed-loop-control.mdx',
  'chapter3/digital-twin-concepts.mdx',
  'chapter3/gazebo-world-creation.mdx',
  'chapter3/physics-simulation.mdx',
  'chapter3/ros2-gazebo-integration.mdx',
  'chapter3/rviz-visualization.mdx',
  'chapter3/sensor-simulation.mdx',
  'chapter3/unity-teaser.mdx',

  // Chapter 4
  'chapter4/capstone-integration.mdx',
  'chapter4/chapter4-plan.mdx',
  'chapter4/isaac-overview.mdx',
  'chapter4/isaac-ros-perception.mdx',
  'chapter4/isaac-ros-perception-pipeline.mdx',
  'chapter4/isaac-sim-advanced-training.mdx',
  'chapter4/isaac-sim-setup.mdx',
  'chapter4/nav2-navigation.mdx',
  'chapter4/navigation-path-planning.mdx',
  'chapter4/nvidia-isaac-overview.mdx',
  'chapter4/photorealistic-simulation.mdx',
  'chapter4/sim-to-real-transfer.mdx',

  // Chapter 5
  'chapter5/chapter5-plan.mdx',
  'chapter5/multimodal-ai-models.mdx',
  'chapter5/multimodal-ai-overview.mdx',
  'chapter5/natural-language-understanding.mdx',
  'chapter5/ros2-isaac-implementation.mdx',
  'chapter5/sensor-fusion-pipelines.mdx',
  'chapter5/vision-processing-vla.mdx',
  'chapter5/vla-architecture.mdx',
  'chapter5/vla-conclusion.mdx',
  'chapter5/vla-introduction.mdx',

  // Resources
  'resources/gazebo-installation.mdx',
  'resources/hardware-requirements.mdx',
  'resources/isaac-installation.mdx',
  'resources/references.mdx',
  'resources/ros2-installation.mdx',
  'resources/setup-guide.mdx',
  'resources/troubleshooting.mdx',
];

function main() {
  const translator = new UrduTranslator();
  let successCount = 0;
  let failCount = 0;

  console.log(`Starting translation of ${FILES_TO_TRANSLATE.length} files...\n`);

  for (const file of FILES_TO_TRANSLATE) {
    const inputPath = path.join(DOCS_DIR, file);
    const outputPath = path.join(OUTPUT_DIR, file);

    // Check if input file exists
    if (!fs.existsSync(inputPath)) {
      console.log(`⚠ Skipped (not found): ${file}`);
      failCount++;
      continue;
    }

    const success = translator.translateFile(inputPath, outputPath);

    if (success) {
      console.log(`✓ Translated: ${file}`);
      successCount++;
    } else {
      console.log(`✗ Failed: ${file}`);
      failCount++;
    }
  }

  console.log(`\n=== Translation Complete ===`);
  console.log(`Success: ${successCount}`);
  console.log(`Failed: ${failCount}`);
  console.log(`Total: ${FILES_TO_TRANSLATE.length}`);
}

main();
