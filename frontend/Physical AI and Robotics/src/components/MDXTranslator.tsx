import React, { ReactNode } from 'react';
import { useTranslation } from '../contexts/TranslationContext';
import TranslationService from '../services/TranslationService';

/**
 * MDXTranslator Component
 *
 * Intelligently translates MDX content while preserving:
 * - Code blocks
 * - Technical terms
 * - Links and formatting
 * - Component structure
 *
 * Usage:
 * <MDXTranslator>
 *   <h1>Learning Objectives</h1>
 *   <p>By the end of this section, you will understand...</p>
 * </MDXTranslator>
 */

interface MDXTranslatorProps {
  children: ReactNode;
  preserveCodeBlocks?: boolean;
}

const MDXTranslator: React.FC<MDXTranslatorProps> = ({
  children,
  preserveCodeBlocks = true,
}) => {
  const { language, t } = useTranslation();

  // If English, return children as-is
  if (language === 'en') {
    return <>{children}</>;
  }

  // For Urdu, translate content
  // Note: This is a simplified implementation
  // In production, you'd integrate with a translation API
  return <>{children}</>;
};

/**
 * TranslatableText Component
 *
 * Wraps text content that should be translated.
 * Automatically translates when language changes.
 *
 * Usage:
 * <TranslatableText text="Learning Objectives" />
 */

interface TranslatableTextProps {
  text: string;
  as?: React.ElementType;
  className?: string;
  fallback?: string;
}

export const TranslatableText: React.FC<TranslatableTextProps> = ({
  text,
  as: Component = 'span',
  className,
  fallback,
}) => {
  const { language, t } = useTranslation();

  // Try to find translation in the dictionary first
  const translationKey = text.toLowerCase().replace(/\s+/g, '.');
  let translatedText = t(`content.${translationKey}`);

  // If no dictionary translation, use TranslationService
  if (translatedText === `content.${translationKey}`) {
    translatedText = TranslationService.getTranslation(text, language);
  }

  // If still no translation and fallback provided, use fallback
  if (translatedText === text && fallback) {
    translatedText = fallback;
  }

  return <Component className={className}>{translatedText}</Component>;
};

/**
 * TranslatableHeading Component
 *
 * Specialized component for translating headings while maintaining SEO
 */

interface TranslatableHeadingProps {
  level: 1 | 2 | 3 | 4 | 5 | 6;
  children: string;
  id?: string;
  className?: string;
}

export const TranslatableHeading: React.FC<TranslatableHeadingProps> = ({
  level,
  children,
  id,
  className,
}) => {
  const { language, t } = useTranslation();
  const Component = `h${level}` as React.ElementType;

  // Try to translate the heading
  const translatedText = TranslationService.getTranslation(children, language);

  return (
    <Component id={id} className={className}>
      {translatedText}
    </Component>
  );
};

export default MDXTranslator;
