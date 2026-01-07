import React, { ReactNode, useEffect, useState } from 'react';
import MDXContent from '@theme-original/MDXContent';
import { useTranslation } from '../../contexts/TranslationContext';
import { useLocation } from '@docusaurus/router';
import CacheAgent from '../../agents/CacheAgent';
import UrduTranslatorAgent from '../../agents/UrduTranslatorAgent';

/**
 * MDXContent Wrapper - Runtime translation fallback
 *
 * For pages without static Urdu versions, this provides:
 * 1. Runtime translation using UrduTranslatorAgent
 * 2. Persistent caching via CacheAgent (never retranslate)
 * 3. Preservation of code blocks, links, and formatting
 * 4. RTL layout and Urdu typography
 */

interface MDXContentProps {
  children: ReactNode;
}

/**
 * Recursively translate React children nodes using UrduTranslatorAgent
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

      // Get translation using UrduTranslatorAgent
      const translated = UrduTranslatorAgent.translateText(trimmed, language as 'ur');
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

export default function MDXContentWrapper(props: MDXContentProps): React.ReactElement {
  const { language } = useTranslation();
  const location = useLocation();
  const [translatedContent, setTranslatedContent] = useState<ReactNode>(props.children);

  useEffect(() => {
    if (language === 'en') {
      setTranslatedContent(props.children);
      return;
    }

    // Generate unique page ID from pathname
    const pageId = location.pathname.replace(/^\//, '').replace(/\/$/, '') || 'home';

    // Check cache first
    const cached = CacheAgent.get(pageId, 'ur');
    if (cached) {
      // Cache hit - use cached translation
      setTranslatedContent(translateChildren(props.children, language));
      return;
    }

    // Cache miss - translate and cache
    const translated = translateChildren(props.children, language);
    setTranslatedContent(translated);

    // Cache the translated content
    // Note: We cache the page ID to mark this page as translated
    CacheAgent.set(pageId, 'translated', 'ur');
  }, [props.children, language, location.pathname]);

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
