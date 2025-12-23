import React, { ReactNode, useMemo } from 'react';
import { useTranslation } from '../contexts/TranslationContext';
import TranslationService from '../services/TranslationService';

/**
 * TranslatableContent Component
 *
 * Automatically translates text content while preserving:
 * - Code blocks and inline code
 * - HTML structure and formatting
 * - Links and images
 * - Component hierarchy
 *
 * Usage in MDX:
 * ```mdx
 * import TranslatableContent from '@site/src/components/TranslatableContent';
 *
 * <TranslatableContent>
 *   <h1>Learning Objectives</h1>
 *   <p>By the end of this section, you will understand...</p>
 * </TranslatableContent>
 * ```
 */

interface TranslatableContentProps {
  children: ReactNode;
  preserveCode?: boolean;
  className?: string;
}

/**
 * Recursively translate React children
 */
function translateNode(node: ReactNode, lang: string, preserveCode: boolean): ReactNode {
  if (lang === 'en') return node;

  // Handle null/undefined
  if (node == null) return node;

  // Handle string nodes
  if (typeof node === 'string') {
    const trimmed = node.trim();
    if (!trimmed) return node;

    // Get translation
    const translated = TranslationService.getTranslation(trimmed, lang as 'ur');

    // If translated, return with same surrounding whitespace
    if (translated !== trimmed) {
      const leadingSpace = node.match(/^\s*/)?.[0] || '';
      const trailingSpace = node.match(/\s*$/)?.[0] || '';
      return leadingSpace + translated + trailingSpace;
    }

    return node;
  }

  // Handle arrays
  if (Array.isArray(node)) {
    return node.map((child, index) =>
      React.cloneElement(
        <React.Fragment key={index}>{translateNode(child, lang, preserveCode)}</React.Fragment>,
        {}
      )
    );
  }

  // Handle React elements
  if (React.isValidElement(node)) {
    const element = node as React.ReactElement<any>;

    // Skip code elements if preserveCode is true
    if (preserveCode) {
      const type = element.type;
      const className = element.props?.className || '';

      if (
        type === 'code' ||
        type === 'pre' ||
        className.includes('language-') ||
        className.includes('prism-code') ||
        className.includes('token')
      ) {
        return node;
      }
    }

    // Translate children
    if (element.props?.children) {
      const translatedChildren = translateNode(element.props.children, lang, preserveCode);
      return React.cloneElement(element, {
        ...element.props,
        children: translatedChildren,
      });
    }
  }

  return node;
}

const TranslatableContent: React.FC<TranslatableContentProps> = ({
  children,
  preserveCode = true,
  className = '',
}) => {
  const { language } = useTranslation();

  // Memoize translated content to avoid unnecessary re-renders
  const translatedContent = useMemo(() => {
    return translateNode(children, language, preserveCode);
  }, [children, language, preserveCode]);

  return (
    <div
      className={`translatable-content ${className} ${language === 'ur' ? 'urdu-content' : ''}`}
      style={{
        fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit',
        direction: language === 'ur' ? 'rtl' : 'ltr',
      }}
    >
      {translatedContent}
    </div>
  );
};

export default TranslatableContent;

/**
 * TranslatableText Component - For inline text translation
 *
 * Usage:
 * <TranslatableText>Learning Objectives</TranslatableText>
 */
export const TranslatableText: React.FC<{ children: string; as?: React.ElementType }> = ({
  children,
  as: Component = 'span',
}) => {
  const { language } = useTranslation();
  const translated = TranslationService.getTranslation(children, language);

  return (
    <Component
      style={{
        fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit',
      }}
    >
      {translated}
    </Component>
  );
};
