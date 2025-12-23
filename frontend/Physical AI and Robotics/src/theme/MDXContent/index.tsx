import React, { ReactNode, useEffect, useState } from 'react';
import MDXContent from '@theme-original/MDXContent';
import { useTranslation } from '../../contexts/TranslationContext';
import TranslationService from '../../services/TranslationService';

/**
 * MDXContent Wrapper - Translates all MDX page content
 *
 * This component intercepts Docusaurus MDX content rendering and
 * applies translations when the language is set to Urdu.
 *
 * Translation Strategy:
 * 1. Intercept text nodes in the React tree
 * 2. Look up translations in TranslationService cache
 * 3. Preserve code blocks, technical terms, and formatting
 * 4. Apply Urdu typography for translated content
 */

interface MDXContentProps {
  children: ReactNode;
}

/**
 * Recursively translate React children nodes
 */
function translateChildren(children: ReactNode, language: string): ReactNode {
  if (language === 'en') {
    return children;
  }

  return React.Children.map(children, (child) => {
    // Handle text nodes
    if (typeof child === 'string') {
      const trimmed = child.trim();
      if (!trimmed) return child; // Preserve whitespace

      // Get translation from service
      const translated = TranslationService.getTranslation(trimmed, language as 'ur');
      return translated;
    }

    // Handle React elements
    if (React.isValidElement(child)) {
      const element = child as React.ReactElement<any>;
      const { type, props } = element;

      // Skip translation for code blocks and technical elements
      if (
        type === 'code' ||
        type === 'pre' ||
        (props && props.className && typeof props.className === 'string' &&
          (props.className.includes('language-') ||
           props.className.includes('code') ||
           props.className.includes('token')))
      ) {
        return child;
      }

      // Recursively translate children of this element
      if (props && props.children) {
        const translatedChildren = translateChildren(props.children, language);
        return React.cloneElement(element, { ...props, children: translatedChildren });
      }
    }

    return child;
  });
}

export default function MDXContentWrapper(props: MDXContentProps): JSX.Element {
  const { language } = useTranslation();
  const [translatedContent, setTranslatedContent] = useState<ReactNode>(props.children);

  useEffect(() => {
    // Apply translation when language changes
    const translated = translateChildren(props.children, language);
    setTranslatedContent(translated);
  }, [props.children, language]);

  return (
    <div
      className={language === 'ur' ? 'urdu-content' : ''}
      style={{
        fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit',
        direction: language === 'ur' ? 'rtl' : 'ltr',
      }}
    >
      <MDXContent {...props}>
        {translatedContent}
      </MDXContent>
    </div>
  );
}
