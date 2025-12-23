/**
 * TranslationService - Core Translation Engine
 *
 * Hybrid translation system that handles:
 * 1. Static UI translations (buttons, labels, navigation)
 * 2. Dynamic content translation (MDX content, headings, paragraphs)
 * 3. Smart caching for performance
 * 4. Extensible for backend API integration
 */

export type Language = 'en' | 'ur';

interface TranslationCache {
  [key: string]: {
    [lang in Language]?: string;
  };
}

class TranslationService {
  private cache: TranslationCache = {};
  private static instance: TranslationService;

  private constructor() {}

  static getInstance(): TranslationService {
    if (!TranslationService.instance) {
      TranslationService.instance = new TranslationService();
    }
    return TranslationService.instance;
  }

  /**
   * Get cached translation or return original text
   */
  getTranslation(text: string, targetLang: Language): string {
    if (targetLang === 'en') return text;

    // Check cache first
    if (this.cache[text]?.[targetLang]) {
      return this.cache[text][targetLang]!;
    }

    return text; // Return original if no translation available
  }

  /**
   * Add translation to cache
   */
  addTranslation(original: string, translation: string, lang: Language): void {
    if (!this.cache[original]) {
      this.cache[original] = {};
    }
    this.cache[original][lang] = translation;
  }

  /**
   * Bulk add translations
   */
  addTranslations(translations: Record<string, string>, lang: Language): void {
    Object.entries(translations).forEach(([original, translation]) => {
      this.addTranslation(original, translation, lang);
    });
  }

  /**
   * Translate HTML/MDX content while preserving structure
   * Handles: headings, paragraphs, lists, tables
   * Preserves: code blocks, links, formatting
   */
  translateContent(content: string, targetLang: Language): string {
    if (targetLang === 'en') return content;

    // Split content into segments, preserving code blocks and special elements
    const codeBlockRegex = /```[\s\S]*?```|`[^`]+`/g;
    const codeBlocks: string[] = [];

    // Extract and preserve code blocks
    let processedContent = content.replace(codeBlockRegex, (match) => {
      const placeholder = `__CODE_BLOCK_${codeBlocks.length}__`;
      codeBlocks.push(match);
      return placeholder;
    });

    // Translate text segments
    const lines = processedContent.split('\n');
    const translatedLines = lines.map((line) => {
      // Skip empty lines
      if (!line.trim()) return line;

      // Check if this line has a translation
      const trimmed = line.trim();
      const translated = this.getTranslation(trimmed, targetLang);

      if (translated !== trimmed) {
        // Preserve leading whitespace/formatting
        const leadingSpace = line.match(/^\s*/)?.[0] || '';
        return leadingSpace + translated;
      }

      return line;
    });

    // Restore code blocks
    let result = translatedLines.join('\n');
    codeBlocks.forEach((block, index) => {
      result = result.replace(`__CODE_BLOCK_${index}__`, block);
    });

    return result;
  }

  /**
   * Clear translation cache
   */
  clearCache(): void {
    this.cache = {};
  }

  /**
   * Get cache size (for debugging)
   */
  getCacheSize(): number {
    return Object.keys(this.cache).length;
  }

  /**
   * Export cache (for persistence)
   */
  exportCache(): TranslationCache {
    return { ...this.cache };
  }

  /**
   * Import cache (for loading saved translations)
   */
  importCache(cache: TranslationCache): void {
    this.cache = { ...cache };
  }
}

export default TranslationService.getInstance();
