/**
 * LanguageStateAgent - Lightweight wrapper for language state management
 * Wraps TranslationContext to provide agent-style interface
 */

import { Language } from '../contexts/TranslationContext';

class LanguageStateAgent {
  private static instance: LanguageStateAgent;
  private listeners: Set<(lang: Language) => void> = new Set();

  private constructor() {}

  static getInstance(): LanguageStateAgent {
    if (!LanguageStateAgent.instance) {
      LanguageStateAgent.instance = new LanguageStateAgent();
    }
    return LanguageStateAgent.instance;
  }

  /**
   * Get current language from localStorage or default to 'en'
   */
  getCurrentLanguage(): Language {
    if (typeof window === 'undefined') return 'en';
    const stored = localStorage.getItem('preferred-language');
    return (stored === 'ur' ? 'ur' : 'en') as Language;
  }

  /**
   * Set language and persist to localStorage
   */
  setLanguage(lang: Language): void {
    if (typeof window === 'undefined') return;
    localStorage.setItem('preferred-language', lang);
    this.notifyListeners(lang);
  }

  /**
   * Subscribe to language changes
   */
  subscribe(callback: (lang: Language) => void): () => void {
    this.listeners.add(callback);
    return () => this.listeners.delete(callback);
  }

  /**
   * Notify all listeners of language change
   */
  private notifyListeners(lang: Language): void {
    this.listeners.forEach(callback => callback(lang));
  }
}

export default LanguageStateAgent.getInstance();
