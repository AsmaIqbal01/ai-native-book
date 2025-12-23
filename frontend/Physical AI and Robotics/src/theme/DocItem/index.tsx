import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import { useTranslation } from '../../contexts/TranslationContext';

/**
 * DocItem Wrapper - Applies translation context to documentation pages
 *
 * This wrapper ensures that:
 * 1. RTL direction is applied for Urdu
 * 2. Urdu fonts are applied to content
 * 3. The page re-renders when language changes
 */

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  const { language } = useTranslation();

  return (
    <div
      className={`doc-item-wrapper ${language === 'ur' ? 'urdu-active' : ''}`}
      style={{
        direction: language === 'ur' ? 'rtl' : 'ltr',
        fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit',
      }}
    >
      <DocItem {...props} />
    </div>
  );
}
