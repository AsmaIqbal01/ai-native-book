/**
 * CacheAgent - Translation cache management
 * Wraps localStorage + TranslationService cache
 */

import { Language } from '../contexts/TranslationContext';
import TranslationService from '../services/TranslationService';

interface CacheEntry {
  content: string;
  timestamp: number;
  language: Language;
}

class CacheAgent {
  private static instance: CacheAgent;
  private memoryCache: Map<string, CacheEntry> = new Map();
  private readonly CACHE_PREFIX = 'urdu_translation_';
  private readonly TTL = 7 * 24 * 60 * 60 * 1000; // 7 days

  private constructor() {
    this.loadFromLocalStorage();
  }

  static getInstance(): CacheAgent {
    if (!CacheAgent.instance) {
      CacheAgent.instance = new CacheAgent();
    }
    return CacheAgent.instance;
  }

  /**
   * Get cached translation for a page
   */
  get(pageId: string, lang: Language): string | null {
    const key = `${pageId}_${lang}`;

    // Check memory cache first
    const memEntry = this.memoryCache.get(key);
    if (memEntry && !this.isExpired(memEntry)) {
      return memEntry.content;
    }

    // Check localStorage
    if (typeof window !== 'undefined') {
      const stored = localStorage.getItem(`${this.CACHE_PREFIX}${key}`);
      if (stored) {
        try {
          const entry: CacheEntry = JSON.parse(stored);
          if (!this.isExpired(entry)) {
            this.memoryCache.set(key, entry);
            return entry.content;
          }
        } catch (e) {
          // Invalid cache entry, remove it
          localStorage.removeItem(`${this.CACHE_PREFIX}${key}`);
        }
      }
    }

    return null;
  }

  /**
   * Store translated content
   */
  set(pageId: string, content: string, lang: Language): void {
    const key = `${pageId}_${lang}`;
    const entry: CacheEntry = {
      content,
      timestamp: Date.now(),
      language: lang,
    };

    // Store in memory
    this.memoryCache.set(key, entry);

    // Store in localStorage
    if (typeof window !== 'undefined') {
      try {
        localStorage.setItem(`${this.CACHE_PREFIX}${key}`, JSON.stringify(entry));
      } catch (e) {
        // localStorage full, clear old entries
        this.clearExpired();
      }
    }

    // Also add to TranslationService cache for reuse
    TranslationService.addTranslation(pageId, content, lang);
  }

  /**
   * Clear expired cache entries
   */
  clearExpired(): void {
    if (typeof window === 'undefined') return;

    const keys = Object.keys(localStorage);
    keys.forEach(key => {
      if (key.startsWith(this.CACHE_PREFIX)) {
        const stored = localStorage.getItem(key);
        if (stored) {
          try {
            const entry: CacheEntry = JSON.parse(stored);
            if (this.isExpired(entry)) {
              localStorage.removeItem(key);
            }
          } catch (e) {
            localStorage.removeItem(key);
          }
        }
      }
    });
  }

  /**
   * Clear all cache
   */
  clearAll(): void {
    this.memoryCache.clear();
    if (typeof window !== 'undefined') {
      const keys = Object.keys(localStorage);
      keys.forEach(key => {
        if (key.startsWith(this.CACHE_PREFIX)) {
          localStorage.removeItem(key);
        }
      });
    }
  }

  /**
   * Check if cache entry is expired
   */
  private isExpired(entry: CacheEntry): boolean {
    return Date.now() - entry.timestamp > this.TTL;
  }

  /**
   * Load cache from localStorage to memory
   */
  private loadFromLocalStorage(): void {
    if (typeof window === 'undefined') return;

    const keys = Object.keys(localStorage);
    keys.forEach(key => {
      if (key.startsWith(this.CACHE_PREFIX)) {
        const stored = localStorage.getItem(key);
        if (stored) {
          try {
            const entry: CacheEntry = JSON.parse(stored);
            if (!this.isExpired(entry)) {
              const cacheKey = key.replace(this.CACHE_PREFIX, '');
              this.memoryCache.set(cacheKey, entry);
            }
          } catch (e) {
            // Invalid entry, skip
          }
        }
      }
    });
  }

  /**
   * Get cache statistics
   */
  getStats(): { memorySize: number; localStorageSize: number } {
    let localStorageSize = 0;
    if (typeof window !== 'undefined') {
      const keys = Object.keys(localStorage);
      keys.forEach(key => {
        if (key.startsWith(this.CACHE_PREFIX)) {
          localStorageSize++;
        }
      });
    }

    return {
      memorySize: this.memoryCache.size,
      localStorageSize,
    };
  }
}

export default CacheAgent.getInstance();
