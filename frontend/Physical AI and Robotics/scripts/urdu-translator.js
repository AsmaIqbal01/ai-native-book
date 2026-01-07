/**
 * Urdu Translator Utility
 *
 * Translates English MDX files to Urdu while preserving:
 * - Frontmatter (YAML)
 * - Code blocks
 * - Links and image references
 * - Technical terms
 * - MDX structure
 */

const fs = require('fs');
const path = require('path');

// Import contentTranslations from TypeScript file
const contentTranslations = {
  // Chapter titles
  'Introduction to AI-Native Robotics': 'AI-Native روبوٹکس کا تعارف',
  'Chapter 1: Introduction to Physical AI': 'باب 1: فزیکل AI کا تعارف',
  'Chapter 2: ROS2 Fundamentals': 'باب 2: ROS2 کی بنیادی باتیں',
  'Chapter 3: Simulation Environments': 'باب 3: نقلی ماحول',
  'Chapter 4: Vision-Language-Action Models': 'باب 4: Vision-Language-Action ماڈلز',
  'Chapter 5: Advanced Topics': 'باب 5: جدید موضوعات',

  // Common headings
  'Learning Objectives': 'سیکھنے کے مقاصد',
  'Prerequisites': 'پیشگی ضروریات',
  'Introduction': 'تعارف',
  'Overview': 'جائزہ',
  'Summary': 'خلاصہ',
  'Key Takeaways': 'کلیدی نکات',
  'Next Steps': 'اگلے مراحل',

  // Add more translations as needed
};

class UrduTranslator {
  constructor() {
    this.translations = contentTranslations;
  }

  /**
   * Translate a single text string
   */
  translate(text) {
    const trimmed = text.trim();
    if (!trimmed) return text;

    // Check if translation exists
    if (this.translations[trimmed]) {
      return this.translations[trimmed];
    }

    // Return original if no translation
    return text;
  }

  /**
   * Translate MDX content while preserving structure
   */
  translateContent(content) {
    const lines = content.split('\n');
    const result = [];
    let inCodeBlock = false;
    let inFrontmatter = false;

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // Track frontmatter
      if (line.trim() === '---') {
        if (i === 0) {
          inFrontmatter = true;
        } else if (inFrontmatter) {
          inFrontmatter = false;
        }
        result.push(line);
        continue;
      }

      // Preserve frontmatter
      if (inFrontmatter) {
        // Translate title and description fields
        if (line.startsWith('title:')) {
          const title = line.substring(6).trim();
          const translated = this.translate(title);
          result.push(`title: ${translated}`);
        } else if (line.startsWith('description:')) {
          const desc = line.substring(12).trim();
          const translated = this.translate(desc);
          result.push(`description: ${translated}`);
        } else {
          result.push(line);
        }
        continue;
      }

      // Track code blocks
      if (line.trim().startsWith('```')) {
        inCodeBlock = !inCodeBlock;
        result.push(line);
        continue;
      }

      // Preserve code blocks
      if (inCodeBlock) {
        result.push(line);
        continue;
      }

      // Skip empty lines
      if (!line.trim()) {
        result.push(line);
        continue;
      }

      // Translate headings
      if (line.startsWith('#')) {
        const hashCount = line.match(/^#+/)[0].length;
        const heading = line.substring(hashCount).trim();
        const translated = this.translate(heading);
        result.push('#'.repeat(hashCount) + ' ' + translated);
        continue;
      }

      // Translate regular text
      const translated = this.translate(line);
      result.push(translated);
    }

    return result.join('\n');
  }

  /**
   * Translate a single MDX file
   */
  translateFile(inputPath, outputPath) {
    try {
      // Read input file
      const content = fs.readFileSync(inputPath, 'utf8');

      // Translate content
      const translated = this.translateContent(content);

      // Ensure output directory exists
      const outputDir = path.dirname(outputPath);
      if (!fs.existsSync(outputDir)) {
        fs.mkdirSync(outputDir, { recursive: true });
      }

      // Write output file
      fs.writeFileSync(outputPath, translated, 'utf8');

      return true;
    } catch (error) {
      console.error(`Error translating ${inputPath}:`, error.message);
      return false;
    }
  }
}

// Export for use in other scripts
module.exports = { UrduTranslator };

// CLI usage
if (require.main === module) {
  const translator = new UrduTranslator();
  const args = process.argv.slice(2);

  if (args.length !== 2) {
    console.log('Usage: node urdu-translator.js <input-file> <output-file>');
    process.exit(1);
  }

  const [inputPath, outputPath] = args;
  const success = translator.translateFile(inputPath, outputPath);

  if (success) {
    console.log(`✓ Translated: ${inputPath} → ${outputPath}`);
  } else {
    console.log(`✗ Failed: ${inputPath}`);
    process.exit(1);
  }
}
