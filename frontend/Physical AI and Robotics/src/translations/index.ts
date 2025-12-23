/**
 * Translation System - Central Export
 *
 * Exports all translation-related utilities, components, and services
 * for easy access throughout the application.
 */

// Services
export { default as TranslationService } from '../services/TranslationService';

// Contexts
export { TranslationProvider, useTranslation } from '../contexts/TranslationContext';
export type { Language } from '../contexts/TranslationContext';

// Components
export { default as MDXTranslator, TranslatableText, TranslatableHeading } from '../components/MDXTranslator';

// Hooks
export { default as useContentTranslation, useMultiContentTranslation } from '../hooks/useContentTranslation';

// Translations
export { contentTranslations, getAllTranslationKeys, hasTranslation } from './contentTranslations';
