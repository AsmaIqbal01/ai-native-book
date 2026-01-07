/**
 * UrduTranslatorAgent - Content translation wrapper
 * Wraps TranslationService.translateContent() with agent interface
 */

import { Language } from '../contexts/TranslationContext';
import TranslationService from '../services/TranslationService';

class UrduTranslatorAgent {
  private static instance: UrduTranslatorAgent;

  private constructor() {}

  static getInstance(): UrduTranslatorAgent {
    if (!UrduTranslatorAgent.instance) {
      UrduTranslatorAgent.instance = new UrduTranslatorAgent();
    }
    return UrduTranslatorAgent.instance;
  }

  /**
   * Translate MDX/HTML content while preserving structure
   * Delegates to TranslationService
   */
  translateContent(content: string, targetLang: Language): string {
    return TranslationService.translateContent(content, targetLang);
  }

  /**
   * Translate a single text segment
   */
  translateText(text: string, targetLang: Language): string {
    return TranslationService.getTranslation(text, targetLang);
  }

  /**
   * Check if translation exists in cache
   */
  hasTranslation(text: string, targetLang: Language): boolean {
    const translated = TranslationService.getTranslation(text, targetLang);
    return translated !== text;
  }

  /**
   * Add translation to service cache
   */
  addTranslation(original: string, translation: string, lang: Language): void {
    TranslationService.addTranslation(original, translation, lang);
  }

  /**
   * Bulk add translations
   */
  addTranslations(translations: Record<string, string>, lang: Language): void {
    TranslationService.addTranslations(translations, lang);
  }
}

export default UrduTranslatorAgent.getInstance();
