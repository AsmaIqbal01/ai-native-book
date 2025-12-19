import React, { useEffect, useState } from 'react';
import { ThemeProvider } from '../contexts/ThemeContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { AuthProvider } from '../contexts/AuthContext';
import { UserPreferencesProvider } from '../contexts/UserPreferencesContext';
import FloatingChatWidget from '../components/FloatingChatWidget';
import TopBar from '../components/TopBar';

/**
 * Root Theme Component
 *
 * This component wraps the entire Docusaurus application.
 * It provides:
 * - Global context providers (Theme, Translation, Auth, UserPreferences)
 * - Floating chat widget accessible on all pages
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
    console.log('🤖 Root.tsx loaded! Browser:', true);
    console.log('🤖 FloatingChatWidget will render');
  }, []);

  return (
    <ThemeProvider>
      <TranslationProvider>
        <AuthProvider>
          <UserPreferencesProvider>
            {/* Top navigation bar - appears on all pages (client-side only) */}
            {isBrowser && <TopBar />}

            {/* Main Docusaurus content */}
            {children}

            {/* Floating chat widget - appears on all pages (client-side only) */}
            {isBrowser && (
              <>
                <FloatingChatWidget />
                {console.log('🤖 FloatingChatWidget rendered!')}
              </>
            )}
          </UserPreferencesProvider>
        </AuthProvider>
      </TranslationProvider>
    </ThemeProvider>
  );
}
