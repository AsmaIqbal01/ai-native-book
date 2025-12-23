import { useTranslation } from '../contexts/TranslationContext';
import TranslationService from '../services/TranslationService';

/**
 * useContentTranslation Hook
 *
 * Simplified hook for translating content in MDX pages and components.
 * Automatically uses TranslationService with fallback to original text.
 *
 * Usage:
 * const tc = useContentTranslation();
 * <h1>{tc('Learning Objectives')}</h1>
 * <p>{tc('This section covers...')}</p>
 */

export function useContentTranslation() {
  const { language } = useTranslation();

  /**
   * Translate content text
   * @param text - Original English text
   * @returns Translated text (or original if no translation available)
   */
  const tc = (text: string): string => {
    if (language === 'en') return text;
    return TranslationService.getTranslation(text, language);
  };

  return tc;
}

/**
 * useMultiContentTranslation Hook
 *
 * Batch translation for multiple strings.
 * Useful for translating arrays or objects.
 *
 * Usage:
 * const translate = useMultiContentTranslation();
 * const items = translate(['Item 1', 'Item 2', 'Item 3']);
 */

export function useMultiContentTranslation() {
  const { language } = useTranslation();

  const translate = (texts: string[]): string[] => {
    if (language === 'en') return texts;
    return texts.map((text) => TranslationService.getTranslation(text, language));
  };

  return translate;
}

export default useContentTranslation;
