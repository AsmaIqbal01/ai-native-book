/**
 * UITextMapperAgent - Maps UI text to Urdu translations
 * Wraps existing translation dictionaries from TranslationContext
 */

import { Language } from '../contexts/TranslationContext';

// Import translation dictionaries (these exist in TranslationContext)
const translations: Record<Language, Record<string, string>> = {
  en: {},
  ur: {
    // Sidebar translations
    'Introduction': 'تعارف',
    'Chapter 1': 'باب 1',
    'Chapter 2': 'باب 2',
    'Chapter 3': 'باب 3',
    'Chapter 4': 'باب 4',
    'Chapter 5': 'باب 5',
    'Resources': 'وسائل',
    'Introduction to Physical AI': 'فزیکل AI کا تعارف',
    'ROS2 Fundamentals': 'ROS2 کی بنیادی باتیں',
    'Simulation Environments': 'نقلی ماحول',
    'Vision-Language-Action Models': 'Vision-Language-Action ماڈلز',
    'Advanced Topics': 'جدید موضوعات',

    // Footer translations
    'Learn': 'سیکھیں',
    'Get Started': 'شروع کریں',
    'Physical AI Fundamentals': 'فزیکل AI کی بنیادی باتیں',
    'ROS2 Basics': 'ROS2 کی بنیادی باتیں',
    'Community': 'کمیونٹی',
    'GitHub': 'گٹ ہب',
    'Discord': 'ڈسکارڈ',
    'Twitter': 'ٹویٹر',
    'More': 'مزید',
    'Copyright': 'کاپی رائٹ',
    'Built with Docusaurus': 'Docusaurus کے ساتھ تعمیر شدہ',

    // Common UI elements
    'Previous': 'پچھلا',
    'Next': 'اگلا',
    'Edit this page': 'یہ صفحہ ترمیم کریں',
    'Docs': 'دستاویزات',
    'On this page': 'اس صفحے پر',
    'Back to top': 'سب سے اوپر واپس',
  },
};

class UITextMapperAgent {
  private static instance: UITextMapperAgent;

  private constructor() {}

  static getInstance(): UITextMapperAgent {
    if (!UITextMapperAgent.instance) {
      UITextMapperAgent.instance = new UITextMapperAgent();
    }
    return UITextMapperAgent.instance;
  }

  /**
   * Translate UI text from English to target language
   */
  translate(text: string, targetLang: Language): string {
    if (targetLang === 'en') return text;
    return translations[targetLang][text] || text;
  }

  /**
   * Check if translation exists for given text
   */
  hasTranslation(text: string, lang: Language): boolean {
    return text in translations[lang];
  }

  /**
   * Add custom translation at runtime
   */
  addTranslation(original: string, translation: string, lang: Language): void {
    if (!translations[lang]) {
      translations[lang] = {};
    }
    translations[lang][original] = translation;
  }
}

export default UITextMapperAgent.getInstance();
