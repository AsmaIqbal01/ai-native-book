import React, { useEffect, useState } from 'react';
import { ThemeProvider } from '../contexts/ThemeContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { AuthProvider } from '../contexts/AuthContext';
import { UserPreferencesProvider } from '../contexts/UserPreferencesContext';
import { ChatbotProvider } from '../contexts/ChatbotContext';
import FloatingChatWidget from '../components/FloatingChatWidget';
import TopBar from '../components/TopBar';
import TranslationService from '../services/TranslationService';
import { contentTranslations } from '../translations/contentTranslations';

/**
 * Root Theme Component
 *
 * This component wraps the entire Docusaurus application.
 * It provides:
 * - Global context providers (Theme, Translation, Auth, UserPreferences, Chatbot)
 * - Viewport-fixed floating chat widget accessible on all pages
 * - Agent-controlled chatbot state management
 *
 * Note: This is a Docusaurus "swizzled" theme component that replaces the default Root.
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

interface RootProps {
  children: React.ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
  // Only render FloatingChatWidget on client side to avoid SSR issues
  const [isBrowser, setIsBrowser] = useState(false);

  useEffect(() => {
    setIsBrowser(true);

    // Initialize TranslationService with content translations
    TranslationService.addTranslations(contentTranslations, 'ur');

    console.log('🤖 Root.tsx loaded! Browser:', true);
    console.log('🌐 TranslationService initialized with', TranslationService.getCacheSize(), 'translations');
    console.log('🤖 ChatbotProvider initialized with global state');
    console.log('🤖 FloatingChatWidget will render with viewport-fixed positioning');
  }, []);

  return (
    <ThemeProvider>
      <TranslationProvider>
        <AuthProvider>
          <UserPreferencesProvider>
            <ChatbotProvider>
              {/* Top navigation bar - appears on all pages (client-side only) */}
              {isBrowser && <TopBar />}

              {/* Main Docusaurus content */}
              {children}

              {/* Viewport-fixed chatbot widget - appears on all pages (client-side only) */}
              {/* State is globally managed by ChatbotContext, persists across routes */}
              {isBrowser && (
                <>
                  <FloatingChatWidget />
                  {console.log('🤖 FloatingChatWidget rendered with global state!')}
                </>
              )}
            </ChatbotProvider>
          </UserPreferencesProvider>
        </AuthProvider>
      </TranslationProvider>
    </ThemeProvider>
  );
}
