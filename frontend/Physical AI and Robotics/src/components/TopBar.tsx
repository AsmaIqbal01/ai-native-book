import React, { useState } from 'react';
import { useTranslation } from '../contexts/TranslationContext';
import { useAuth } from '../contexts/AuthContext';
import { useTheme } from '../contexts/ThemeContext';
import PersonalizeContextModal from './PersonalizeContextModal';
import Link from '@docusaurus/Link';

/**
 * TopBar Component
 *
 * Main navigation bar with:
 * - Language translation toggle (EN ⇄ UR)
 * - GitHub repository link
 * - Login/Logout functionality
 * - Personalize Context (visible only when logged in)
 *
 * Features:
 * - Fully responsive (mobile & desktop)
 * - Accessibility compliant (ARIA labels, keyboard navigation)
 * - Theme-aware (supports dark mode)
 * - Smooth animations and hover effects
 */

interface TopBarProps {
  className?: string;
}

const TopBar: React.FC<TopBarProps> = ({ className = '' }) => {
  const { language, toggleLanguage, t } = useTranslation();
  const { isLoggedIn, user, logout } = useAuth();
  const { darkMode, toggleDarkMode } = useTheme();
  const [isPersonalizeModalOpen, setIsPersonalizeModalOpen] = useState(false);
  const [showLoginForm, setShowLoginForm] = useState(false);

  const handleGitHubClick = () => {
    window.open('https://github.com/AsmaIqbal01/ai-native-book', '_blank', 'noopener,noreferrer');
  };

  const handleLoginClick = () => {
    if (isLoggedIn) {
      logout();
    } else {
      setShowLoginForm(true);
    }
  };

  const handlePersonalizeClick = () => {
    if (isLoggedIn) {
      setIsPersonalizeModalOpen(true);
    }
  };

  return (
    <>
      <div
        className={`w-full bg-white dark:bg-gray-800 border-b border-gray-200 dark:border-gray-700 shadow-sm ${className}`}
        role="banner"
      >
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex items-center justify-between h-16">
            {/* Left Section - Brand Title with Spacing */}
            <div className="flex items-center gap-6">
              <Link
                to="/"
                className="flex items-center gap-6 no-underline hover:no-underline group"
                aria-label="Home - AI-Native Robotics Textbook"
              >
                <span
                  className="text-xl sm:text-2xl font-bold text-gray-900 dark:text-white tracking-tight transition-colors duration-200 group-hover:text-blue-600 dark:group-hover:text-blue-400"
                  style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                >
                  {language === 'en' ? 'AI-Native Robotics' : 'AI-Native روبوٹکس'}
                </span>
                <span
                  className="text-xl sm:text-2xl font-bold text-blue-600 dark:text-blue-400 tracking-tight transition-colors duration-200 group-hover:text-blue-700 dark:group-hover:text-blue-300"
                  style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                >
                  {language === 'en' ? 'Textbook' : 'کتاب'}
                </span>
              </Link>
            </div>

            {/* Right Section - Action Buttons */}
            <div className="flex items-center gap-3 sm:gap-4">
              {/* Language Toggle Button */}
              <button
                onClick={toggleLanguage}
                className="inline-flex items-center gap-2 px-3 py-2 sm:px-4 sm:py-2 rounded-lg font-medium text-gray-700 dark:text-gray-200 bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600 transition-all duration-200 transform hover:scale-105 active:scale-95"
                aria-label={`Switch to ${language === 'en' ? 'Urdu' : 'English'}`}
                title={`Switch to ${language === 'en' ? 'Urdu' : 'English'}`}
              >
                <svg
                  className="w-4 h-4 sm:w-5 sm:h-5"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                  aria-hidden="true"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129"
                  />
                </svg>
                <span className="text-sm font-bold">{language === 'en' ? 'EN' : 'اردو'}</span>
              </button>

              {/* GitHub Button */}
              <button
                onClick={handleGitHubClick}
                className="inline-flex items-center gap-2 px-3 py-2 sm:px-4 sm:py-2 rounded-lg font-medium text-gray-700 dark:text-gray-200 bg-gray-100 dark:bg-gray-700 hover:bg-gray-800 dark:hover:bg-gray-600 hover:text-white transition-all duration-200 transform hover:scale-105 active:scale-95"
                aria-label="Open GitHub repository"
                title={t('topbar.github')}
              >
                <svg
                  className="w-4 h-4 sm:w-5 sm:h-5"
                  fill="currentColor"
                  viewBox="0 0 24 24"
                  aria-hidden="true"
                >
                  <path
                    fillRule="evenodd"
                    d="M12 2C6.477 2 2 6.484 2 12.017c0 4.425 2.865 8.18 6.839 9.504.5.092.682-.217.682-.483 0-.237-.008-.868-.013-1.703-2.782.605-3.369-1.343-3.369-1.343-.454-1.158-1.11-1.466-1.11-1.466-.908-.62.069-.608.069-.608 1.003.07 1.531 1.032 1.531 1.032.892 1.53 2.341 1.088 2.91.832.092-.647.35-1.088.636-1.338-2.22-.253-4.555-1.113-4.555-4.951 0-1.093.39-1.988 1.029-2.688-.103-.253-.446-1.272.098-2.65 0 0 .84-.27 2.75 1.026A9.564 9.564 0 0112 6.844c.85.004 1.705.115 2.504.337 1.909-1.296 2.747-1.027 2.747-1.027.546 1.379.202 2.398.1 2.651.64.7 1.028 1.595 1.028 2.688 0 3.848-2.339 4.695-4.566 4.943.359.309.678.92.678 1.855 0 1.338-.012 2.419-.012 2.747 0 .268.18.58.688.482A10.019 10.019 0 0022 12.017C22 6.484 17.522 2 12 2z"
                    clipRule="evenodd"
                  />
                </svg>
                <span className="hidden md:inline text-sm font-medium">{t('topbar.github')}</span>
              </button>

              {/* Personalize Context Button (Only visible when logged in) */}
              {isLoggedIn && (
                <button
                  onClick={handlePersonalizeClick}
                  className="inline-flex items-center gap-2 px-3 py-2 sm:px-4 sm:py-2 rounded-lg font-medium text-white bg-purple-500 hover:bg-purple-600 transition-all duration-200 transform hover:scale-105 active:scale-95 shadow-md hover:shadow-lg"
                  aria-label="Personalize chatbot context"
                  title={t('topbar.personalize')}
                >
                  <svg
                    className="w-4 h-4 sm:w-5 sm:h-5"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                    aria-hidden="true"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4"
                    />
                  </svg>
                  <span
                    className="hidden lg:inline text-sm font-medium"
                    style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                  >
                    {t('topbar.personalize')}
                  </span>
                </button>
              )}

              {/* Login/Logout Button */}
              <button
                onClick={handleLoginClick}
                className={`inline-flex items-center gap-2 px-3 py-2 sm:px-4 sm:py-2 rounded-lg font-medium transition-all duration-200 transform hover:scale-105 active:scale-95 ${
                  isLoggedIn
                    ? 'text-red-600 dark:text-red-400 bg-red-50 dark:bg-red-900/20 hover:bg-red-100 dark:hover:bg-red-900/30'
                    : 'text-white bg-blue-500 hover:bg-blue-600 shadow-md hover:shadow-lg'
                }`}
                aria-label={isLoggedIn ? 'Logout' : 'Login'}
                title={isLoggedIn ? t('topbar.logout') : t('topbar.login')}
              >
                {isLoggedIn ? (
                  <>
                    <svg
                      className="w-4 h-4 sm:w-5 sm:h-5"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                      aria-hidden="true"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M17 16l4-4m0 0l-4-4m4 4H7m6 4v1a3 3 0 01-3 3H6a3 3 0 01-3-3V7a3 3 0 013-3h4a3 3 0 013 3v1"
                      />
                    </svg>
                    <span
                      className="hidden sm:inline text-sm font-medium"
                      style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                    >
                      {t('topbar.logout')}
                    </span>
                  </>
                ) : (
                  <>
                    <svg
                      className="w-4 h-4 sm:w-5 sm:h-5"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                      aria-hidden="true"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M11 16l-4-4m0 0l4-4m-4 4h14m-5 4v1a3 3 0 01-3 3H6a3 3 0 01-3-3V7a3 3 0 013-3h7a3 3 0 013 3v1"
                      />
                    </svg>
                    <span
                      className="hidden sm:inline text-sm font-medium"
                      style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                    >
                      {t('topbar.login')}
                    </span>
                  </>
                )}
              </button>

              {/* User Avatar (when logged in) */}
              {isLoggedIn && user && (
                <div
                  className="hidden sm:flex items-center justify-center w-10 h-10 rounded-full bg-gradient-to-br from-blue-400 to-purple-500 text-white font-bold text-sm shadow-md"
                  title={user.name}
                  aria-label={`Logged in as ${user.name}`}
                >
                  {user.name.charAt(0).toUpperCase()}
                </div>
              )}

              {/* Dark Mode Toggle Button */}
              <button
                onClick={toggleDarkMode}
                className="inline-flex items-center justify-center w-10 h-10 rounded-lg font-medium text-gray-700 dark:text-gray-200 bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600 transition-all duration-200 transform hover:scale-105 active:scale-95"
                aria-label={darkMode ? 'Switch to light mode' : 'Switch to dark mode'}
                title={darkMode ? 'Switch to light mode' : 'Switch to dark mode'}
              >
                {darkMode ? (
                  <svg
                    className="w-5 h-5"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                    aria-hidden="true"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M12 3v1m0 16v1m9-9h-1M4 12H3m15.364 6.364l-.707-.707M6.343 6.343l-.707-.707m12.728 0l-.707.707M6.343 17.657l-.707.707M16 12a4 4 0 11-8 0 4 4 0 018 0z"
                    />
                  </svg>
                ) : (
                  <svg
                    className="w-5 h-5"
                    fill="none"
                    stroke="currentColor"
                    viewBox="0 0 24 24"
                    aria-hidden="true"
                  >
                    <path
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      strokeWidth={2}
                      d="M20.354 15.354A9 9 0 018.646 3.646 9.003 9.003 0 0012 21a9.003 9.003 0 008.354-5.646z"
                    />
                  </svg>
                )}
              </button>
            </div>
          </div>
        </div>

        {/* User Info Banner (Mobile - when logged in) */}
        {isLoggedIn && user && (
          <div className="sm:hidden px-4 py-2 bg-blue-50 dark:bg-blue-900/20 border-t border-blue-100 dark:border-blue-800">
            <div className="flex items-center gap-2 text-sm text-blue-800 dark:text-blue-200">
              <div className="flex items-center justify-center w-6 h-6 rounded-full bg-gradient-to-br from-blue-400 to-purple-500 text-white font-bold text-xs">
                {user.name.charAt(0).toUpperCase()}
              </div>
              <span className="font-medium">Welcome, {user.name}</span>
            </div>
          </div>
        )}
      </div>

      {/* Personalize Context Modal */}
      <PersonalizeContextModal
        isOpen={isPersonalizeModalOpen}
        onClose={() => setIsPersonalizeModalOpen(false)}
      />

      {/* Simple Login Form Modal (Stub) */}
      {showLoginForm && <LoginFormStub onClose={() => setShowLoginForm(false)} />}
    </>
  );
};

/**
 * LoginFormStub - Simple login form for demonstration
 * Replace with actual authentication implementation
 */
interface LoginFormStubProps {
  onClose: () => void;
}

const LoginFormStub: React.FC<LoginFormStubProps> = ({ onClose }) => {
  const { login } = useAuth();
  const { t, language } = useTranslation();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    try {
      await login(email, password);
      onClose();
    } catch (error) {
      console.error('Login failed:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      <div
        className="fixed inset-0 bg-black bg-opacity-50 z-40"
        onClick={onClose}
        aria-hidden="true"
      />
      <div
        className="fixed inset-0 z-50 flex items-center justify-center p-4"
        role="dialog"
        aria-modal="true"
      >
        <div className="bg-white dark:bg-gray-800 rounded-2xl shadow-2xl max-w-md w-full p-6">
          <h2
            className="text-2xl font-bold text-gray-900 dark:text-white mb-4"
            style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
          >
            {t('topbar.login')}
          </h2>
          <form onSubmit={handleSubmit} className="space-y-4">
            <input
              type="email"
              placeholder="Email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className="w-full px-4 py-3 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-white"
              required
            />
            <input
              type="password"
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="w-full px-4 py-3 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-white"
              required
            />
            <div className="flex gap-3">
              <button
                type="button"
                onClick={onClose}
                className="flex-1 px-4 py-3 rounded-lg font-medium text-gray-700 dark:text-gray-300 bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600"
              >
                Cancel
              </button>
              <button
                type="submit"
                disabled={loading}
                className="flex-1 px-4 py-3 rounded-lg font-medium text-white bg-blue-500 hover:bg-blue-600 disabled:opacity-50"
              >
                {loading ? 'Logging in...' : 'Login'}
              </button>
            </div>
          </form>
        </div>
      </div>
    </>
  );
};

export default TopBar;
