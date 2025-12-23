import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * Translation Context for multilingual support (English ⇄ Urdu)
 *
 * Integrates with Docusaurus i18n system to provide:
 * - Language switching functionality across the entire application
 * - Seamless integration with Docusaurus locale routing
 * - UI element translations (topbar, footer, chat)
 * - RTL support for Urdu
 *
 * Note: Full page content translation is handled by Docusaurus i18n
 * (separate Urdu MDX files in i18n/ur/docusaurus-plugin-content-docs/current/)
 */

export type Language = 'en' | 'ur';

interface TranslationContextType {
  language: Language;
  toggleLanguage: () => void;
  setLanguage: (lang: Language) => void;
  t: (key: string) => string;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

interface TranslationProviderProps {
  children: ReactNode;
}

// Translation dictionary - English to Urdu mappings
const translations: Record<Language, Record<string, string>> = {
  en: {
    // TopBar
    'topbar.translate': 'Translate',
    'topbar.english': 'English',
    'topbar.urdu': 'اردو',
    'topbar.github': 'GitHub',
    'topbar.login': 'Login',
    'topbar.logout': 'Logout',
    'topbar.personalize': 'Personalize Context',
    'topbar.brand': 'AI-Native Robotics Textbook',

    // Chat Interface
    'chat.title': 'AI RAG Assistant',
    'chat.connected': 'Connected',
    'chat.offline': 'Offline',
    'chat.placeholder': 'Type your message...',
    'chat.send': 'Send',
    'chat.welcome': "Hello! I'm your AI RAG assistant for the Physical AI and Robotics textbook. Ask me anything about AI-Native Development, ROS2, Digital Twins, or Vision-Language-Action models!",
    'chat.openChat': 'Open Chat',
    'chat.closeChat': 'Close Chat',
    'chat.clearHistory': 'Clear History',

    // Personalization Modal
    'personalize.title': 'Personalize Your Experience',
    'personalize.subtitle': 'Customize how the AI assistant responds to you',
    'personalize.name.label': 'Your Name',
    'personalize.name.placeholder': 'Enter your name',
    'personalize.expertise.label': 'Expertise Level',
    'personalize.expertise.beginner': 'Beginner',
    'personalize.expertise.intermediate': 'Intermediate',
    'personalize.expertise.advanced': 'Advanced',
    'personalize.interests.label': 'Areas of Interest',
    'personalize.interests.placeholder': 'e.g., ROS2, Digital Twins, VLA Models',
    'personalize.context.label': 'Additional Context',
    'personalize.context.placeholder': 'Tell us about your background or specific goals...',
    'personalize.save': 'Save Preferences',
    'personalize.cancel': 'Cancel',
    'personalize.saved': 'Preferences saved successfully!',

    // Navigation & Sidebar
    'nav.textbook': 'Textbook',
    'nav.assistant': 'AI Assistant',
    'nav.translator': 'Urdu Translator',
    'sidebar.chapter1': 'Chapter 1: Introduction to Physical AI',
    'sidebar.chapter2': 'Chapter 2: ROS2 Fundamentals',
    'sidebar.chapter3': 'Chapter 3: Simulation Environments',
    'sidebar.chapter4': 'Chapter 4: Vision-Language-Action Models',
    'sidebar.chapter5': 'Chapter 5: Advanced Topics',
    'sidebar.resources': 'Resources',

    // Footer
    'footer.learn': 'Learn',
    'footer.getStarted': 'Get Started',
    'footer.physicalAI': 'Physical AI Fundamentals',
    'footer.ros2Basics': 'ROS2 Basics',
    'footer.community': 'Community',
    'footer.github': 'GitHub',
    'footer.discord': 'Discord',
    'footer.twitter': 'Twitter',
    'footer.more': 'More',
    'footer.resources': 'Resources',
    'footer.copyright': 'Copyright © {year} AI-Native Robotics Book. Built with Docusaurus.',

    // Content sections (common MDX elements)
    'content.learningObjectives': 'Learning Objectives',
    'content.byEndOfSection': 'By the end of this section, you will:',
    'content.prerequisites': 'Prerequisites',
    'content.summary': 'Summary',
    'content.keyTakeaways': 'Key Takeaways',
    'content.nextSteps': 'Next Steps',
    'content.readMore': 'Read More',
    'content.tableOfContents': 'Table of Contents',

    // Common
    'common.close': 'Close',
    'common.open': 'Open',
    'common.loading': 'Loading...',
    'common.error': 'Error',
    'common.success': 'Success',
    'common.warning': 'Warning',
    'common.info': 'Info',
    'common.note': 'Note',
    'common.tip': 'Tip',
    'common.caution': 'Caution',
    'common.danger': 'Danger',
  },
  ur: {
    // TopBar
    'topbar.translate': 'ترجمہ کریں',
    'topbar.english': 'English',
    'topbar.urdu': 'اردو',
    'topbar.github': 'گٹ ہب',
    'topbar.login': 'لاگ ان',
    'topbar.logout': 'لاگ آؤٹ',
    'topbar.personalize': 'ذاتی نوعیت بنائیں',
    'topbar.brand': 'AI-Native روبوٹکس کتاب',

    // Chat Interface
    'chat.title': 'AI RAG اسسٹنٹ',
    'chat.connected': 'منسلک',
    'chat.offline': 'آف لائن',
    'chat.placeholder': 'اپنا پیغام ٹائپ کریں...',
    'chat.send': 'بھیجیں',
    'chat.welcome': 'ہیلو! میں فزیکل AI اور روبوٹکس کی کتاب کے لیے آپ کا AI RAG اسسٹنٹ ہوں۔ AI-Native Development، ROS2، Digital Twins، یا Vision-Language-Action ماڈلز کے بارے میں کچھ بھی پوچھیں!',
    'chat.openChat': 'چیٹ کھولیں',
    'chat.closeChat': 'چیٹ بند کریں',
    'chat.clearHistory': 'سابقہ صاف کریں',

    // Personalization Modal
    'personalize.title': 'اپنا تجربہ ذاتی بنائیں',
    'personalize.subtitle': 'AI اسسٹنٹ آپ کو کیسے جواب دیتا ہے اسے تبدیل کریں',
    'personalize.name.label': 'آپ کا نام',
    'personalize.name.placeholder': 'اپنا نام درج کریں',
    'personalize.expertise.label': 'مہارت کی سطح',
    'personalize.expertise.beginner': 'ابتدائی',
    'personalize.expertise.intermediate': 'درمیانی',
    'personalize.expertise.advanced': 'اعلیٰ',
    'personalize.interests.label': 'دلچسپی کے شعبے',
    'personalize.interests.placeholder': 'مثال: ROS2، Digital Twins، VLA Models',
    'personalize.context.label': 'اضافی سیاق و سباق',
    'personalize.context.placeholder': 'ہمیں اپنے پس منظر یا مخصوص اہداف کے بارے میں بتائیں...',
    'personalize.save': 'ترجیحات محفوظ کریں',
    'personalize.cancel': 'منسوخ کریں',
    'personalize.saved': 'ترجیحات کامیابی سے محفوظ ہوگئیں!',

    // Navigation & Sidebar
    'nav.textbook': 'کتاب',
    'nav.assistant': 'AI اسسٹنٹ',
    'nav.translator': 'اردو مترجم',
    'sidebar.chapter1': 'باب 1: فزیکل AI کا تعارف',
    'sidebar.chapter2': 'باب 2: ROS2 کی بنیادی باتیں',
    'sidebar.chapter3': 'باب 3: نقلی ماحول',
    'sidebar.chapter4': 'باب 4: Vision-Language-Action ماڈلز',
    'sidebar.chapter5': 'باب 5: جدید موضوعات',
    'sidebar.resources': 'وسائل',

    // Footer
    'footer.learn': 'سیکھیں',
    'footer.getStarted': 'شروع کریں',
    'footer.physicalAI': 'فزیکل AI کی بنیادی باتیں',
    'footer.ros2Basics': 'ROS2 کی بنیادی باتیں',
    'footer.community': 'کمیونٹی',
    'footer.github': 'گٹ ہب',
    'footer.discord': 'ڈسکارڈ',
    'footer.twitter': 'ٹویٹر',
    'footer.more': 'مزید',
    'footer.resources': 'وسائل',
    'footer.copyright': 'کاپی رائٹ © {year} AI-Native روبوٹکس کتاب۔ Docusaurus کے ساتھ تعمیر شدہ۔',

    // Content sections (common MDX elements)
    'content.learningObjectives': 'سیکھنے کے مقاصد',
    'content.byEndOfSection': 'اس حصے کے اختتام تک، آپ:',
    'content.prerequisites': 'پیشگی ضروریات',
    'content.summary': 'خلاصہ',
    'content.keyTakeaways': 'کلیدی نکات',
    'content.nextSteps': 'اگلے مراحل',
    'content.readMore': 'مزید پڑھیں',
    'content.tableOfContents': 'فہرست',

    // Common
    'common.close': 'بند کریں',
    'common.open': 'کھولیں',
    'common.loading': 'لوڈ ہو رہا ہے...',
    'common.error': 'خرابی',
    'common.success': 'کامیابی',
    'common.warning': 'انتباہ',
    'common.info': 'معلومات',
    'common.note': 'نوٹ',
    'common.tip': 'مشورہ',
    'common.caution': 'احتیاط',
    'common.danger': 'خطرہ',
  },
};

export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  // Get current Docusaurus locale and config
  const { i18n, siteConfig } = useDocusaurusContext();
  const location = useLocation();
  const currentLocale = (i18n.currentLocale as Language) || 'en';
  const baseUrl = siteConfig.baseUrl;

  const [language, setLanguageState] = useState<Language>(currentLocale);

  // Sync with Docusaurus locale changes
  useEffect(() => {
    setLanguageState(currentLocale);
  }, [currentLocale]);

  // Apply global styling based on language
  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Update HTML lang attribute for accessibility
    document.documentElement.lang = language;

    // Set text direction for Urdu (RTL)
    document.documentElement.dir = language === 'ur' ? 'rtl' : 'ltr';

    // Apply Urdu font globally when language is Urdu
    if (language === 'ur') {
      document.documentElement.classList.add('urdu-active');
      document.documentElement.style.setProperty('--global-font-family', "'Noto Nastaliq Urdu', serif");
    } else {
      document.documentElement.classList.remove('urdu-active');
      document.documentElement.style.removeProperty('--global-font-family');
    }
  }, [language]);

  const toggleLanguage = () => {
    const newLocale = language === 'en' ? 'ur' : 'en';

    // Navigate to the new locale using Docusaurus routing
    if (typeof window !== 'undefined') {
      const currentPath = location.pathname;

      // Normalize baseUrl (ensure it starts and ends with /)
      const normalizedBaseUrl = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;

      // Remove baseUrl prefix to get the path relative to the site root
      let pathWithoutBase = currentPath;
      if (currentPath.startsWith(normalizedBaseUrl.slice(0, -1))) {
        pathWithoutBase = currentPath.substring(normalizedBaseUrl.length - 1);
      }

      // IMPORTANT: Since we only have one Urdu page (introduction.mdx),
      // always redirect to /ur/docs/introduction when switching to Urdu
      if (newLocale === 'ur') {
        const finalPath = normalizedBaseUrl.slice(0, -1) + '/ur/docs/introduction';
        console.log('Translation navigation (to Urdu):', { currentPath, finalPath });
        window.location.href = finalPath;
        return;
      }

      // When switching from Urdu to English
      if (newLocale === 'en') {
        // Check if we're on the Urdu introduction page
        const isUrIntro = pathWithoutBase === '/ur/docs/introduction' ||
                          pathWithoutBase === '/ur/docs/introduction/';

        if (isUrIntro) {
          // Redirect to English introduction page
          const finalPath = normalizedBaseUrl.slice(0, -1) + '/docs/introduction';
          console.log('Translation navigation (Urdu intro to English intro):', { currentPath, finalPath });
          window.location.href = finalPath;
          return;
        }

        // For any other Urdu page, redirect to English homepage
        if (pathWithoutBase.startsWith('/ur')) {
          const finalPath = normalizedBaseUrl.slice(0, -1) + '/';
          console.log('Translation navigation (Urdu to English homepage):', { currentPath, finalPath });
          window.location.href = finalPath;
          return;
        }
      }

      // Fallback: Try to navigate to the equivalent page in the new locale
      let newPathWithoutBase: string;
      if (newLocale === 'ur') {
        // Add /ur prefix
        newPathWithoutBase = pathWithoutBase.startsWith('/ur/')
          ? pathWithoutBase
          : pathWithoutBase.startsWith('/ur')
          ? pathWithoutBase
          : `/ur${pathWithoutBase}`;
      } else {
        // Remove /ur prefix
        newPathWithoutBase = pathWithoutBase.replace(/^\/ur(\/|$)/, '/');
      }

      // Ensure path starts with /
      if (!newPathWithoutBase.startsWith('/')) {
        newPathWithoutBase = '/' + newPathWithoutBase;
      }

      // Combine baseUrl with new path
      const finalPath = normalizedBaseUrl.slice(0, -1) + newPathWithoutBase;

      console.log('Translation navigation (fallback):', { currentPath, baseUrl, pathWithoutBase, newPathWithoutBase, finalPath });
      window.location.href = finalPath;
    }
  };

  const setLanguage = (lang: Language) => {
    if (lang !== language) {
      // Navigate to the new locale using Docusaurus routing
      if (typeof window !== 'undefined') {
        const currentPath = location.pathname;

        // Normalize baseUrl (ensure it starts and ends with /)
        const normalizedBaseUrl = baseUrl.endsWith('/') ? baseUrl : `${baseUrl}/`;

        // Remove baseUrl prefix to get the path relative to the site root
        let pathWithoutBase = currentPath;
        if (currentPath.startsWith(normalizedBaseUrl.slice(0, -1))) {
          pathWithoutBase = currentPath.substring(normalizedBaseUrl.length - 1);
        }

        // IMPORTANT: Since we only have one Urdu page (introduction.mdx),
        // always redirect to /ur/docs/introduction when switching to Urdu
        if (lang === 'ur') {
          const finalPath = normalizedBaseUrl.slice(0, -1) + '/ur/docs/introduction';
          console.log('Translation navigation (to Urdu):', { currentPath, finalPath });
          window.location.href = finalPath;
          return;
        }

        // When switching from Urdu to English
        if (lang === 'en') {
          // Check if we're on the Urdu introduction page
          const isUrIntro = pathWithoutBase === '/ur/docs/introduction' ||
                            pathWithoutBase === '/ur/docs/introduction/';

          if (isUrIntro) {
            // Redirect to English introduction page
            const finalPath = normalizedBaseUrl.slice(0, -1) + '/docs/introduction';
            console.log('Translation navigation (Urdu intro to English intro):', { currentPath, finalPath });
            window.location.href = finalPath;
            return;
          }

          // For any other Urdu page, redirect to English homepage
          if (pathWithoutBase.startsWith('/ur')) {
            const finalPath = normalizedBaseUrl.slice(0, -1) + '/';
            console.log('Translation navigation (Urdu to English homepage):', { currentPath, finalPath });
            window.location.href = finalPath;
            return;
          }
        }

        // Fallback: Try to navigate to the equivalent page in the new locale
        let newPathWithoutBase: string;
        if (lang === 'ur') {
          // Add /ur prefix
          newPathWithoutBase = pathWithoutBase.startsWith('/ur/')
            ? pathWithoutBase
            : pathWithoutBase.startsWith('/ur')
            ? pathWithoutBase
            : `/ur${pathWithoutBase}`;
        } else {
          // Remove /ur prefix
          newPathWithoutBase = pathWithoutBase.replace(/^\/ur(\/|$)/, '/');
        }

        // Ensure path starts with /
        if (!newPathWithoutBase.startsWith('/')) {
          newPathWithoutBase = '/' + newPathWithoutBase;
        }

        // Combine baseUrl with new path
        const finalPath = normalizedBaseUrl.slice(0, -1) + newPathWithoutBase;

        console.log('Translation navigation (fallback):', { currentPath, baseUrl, pathWithoutBase, newPathWithoutBase, finalPath });
        window.location.href = finalPath;
      }
    }
  };

  // Translation function
  const t = (key: string): string => {
    return translations[language][key] || key;
  };

  return (
    <TranslationContext.Provider value={{ language, toggleLanguage, setLanguage, t }}>
      {children}
    </TranslationContext.Provider>
  );
};

export const useTranslation = (): TranslationContextType => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
};
