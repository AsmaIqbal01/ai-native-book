import React, { useState, useEffect } from 'react';
import { useUserPreferences, ExpertiseLevel } from '../contexts/UserPreferencesContext';
import { useTranslation } from '../contexts/TranslationContext';

/**
 * PersonalizeContextModal Component
 *
 * Modal for users to customize their chatbot experience by setting:
 * - Name
 * - Expertise level
 * - Areas of interest
 * - Additional context/background
 *
 * Features:
 * - Animated slide-in/fade-in modal
 * - Form validation
 * - localStorage persistence via UserPreferencesContext
 * - Responsive design
 * - Accessibility (ARIA labels, keyboard navigation)
 */

interface PersonalizeContextModalProps {
  isOpen: boolean;
  onClose: () => void;
}

const PersonalizeContextModal: React.FC<PersonalizeContextModalProps> = ({ isOpen, onClose }) => {
  const { preferences, updatePreferences } = useUserPreferences();
  const { t, language } = useTranslation();

  // Local form state
  const [formData, setFormData] = useState({
    name: preferences.name,
    expertiseLevel: preferences.expertiseLevel,
    interests: preferences.interests.join(', '),
    additionalContext: preferences.additionalContext,
  });

  const [showSuccess, setShowSuccess] = useState(false);

  // Sync form with preferences when modal opens
  useEffect(() => {
    if (isOpen) {
      setFormData({
        name: preferences.name,
        expertiseLevel: preferences.expertiseLevel,
        interests: preferences.interests.join(', '),
        additionalContext: preferences.additionalContext,
      });
      setShowSuccess(false);
    }
  }, [isOpen, preferences]);

  // Close modal on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };
    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [isOpen, onClose]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    // Parse interests from comma-separated string
    const interestsArray = formData.interests
      .split(',')
      .map(item => item.trim())
      .filter(item => item !== '');

    updatePreferences({
      name: formData.name,
      expertiseLevel: formData.expertiseLevel,
      interests: interestsArray,
      additionalContext: formData.additionalContext,
    });

    setShowSuccess(true);
    setTimeout(() => {
      setShowSuccess(false);
      onClose();
    }, 1500);
  };

  if (!isOpen) return null;

  return (
    <>
      {/* Backdrop */}
      <div
        className="fixed inset-0 bg-black bg-opacity-50 z-40 transition-opacity duration-300"
        onClick={onClose}
        aria-hidden="true"
      />

      {/* Modal */}
      <div
        className="fixed inset-0 z-50 flex items-center justify-center p-4 overflow-y-auto"
        role="dialog"
        aria-modal="true"
        aria-labelledby="personalize-modal-title"
      >
        <div
          className={`bg-white dark:bg-gray-800 rounded-2xl shadow-2xl max-w-2xl w-full transform transition-all duration-300 ${
            isOpen ? 'scale-100 opacity-100' : 'scale-95 opacity-0'
          } ${language === 'ur' ? 'text-right' : 'text-left'}`}
          onClick={(e) => e.stopPropagation()}
        >
          {/* Header */}
          <div className="px-6 py-5 border-b border-gray-200 dark:border-gray-700">
            <div className="flex items-center justify-between">
              <div>
                <h2
                  id="personalize-modal-title"
                  className="text-2xl font-bold text-gray-900 dark:text-white"
                  style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                >
                  {t('personalize.title')}
                </h2>
                <p
                  className="mt-1 text-sm text-gray-600 dark:text-gray-400"
                  style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                >
                  {t('personalize.subtitle')}
                </p>
              </div>
              <button
                onClick={onClose}
                className="p-2 rounded-full hover:bg-gray-100 dark:hover:bg-gray-700 transition-colors"
                aria-label={t('common.close')}
              >
                <svg
                  className="w-6 h-6 text-gray-500 dark:text-gray-400"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M6 18L18 6M6 6l12 12"
                  />
                </svg>
              </button>
            </div>
          </div>

          {/* Form */}
          <form onSubmit={handleSubmit} className="px-6 py-5 space-y-5">
            {/* Name Input */}
            <div>
              <label
                htmlFor="name"
                className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              >
                {t('personalize.name.label')}
              </label>
              <input
                type="text"
                id="name"
                value={formData.name}
                onChange={(e) => setFormData({ ...formData, name: e.target.value })}
                placeholder={t('personalize.name.placeholder')}
                className="w-full px-4 py-3 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              />
            </div>

            {/* Expertise Level */}
            <div>
              <label
                htmlFor="expertise"
                className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              >
                {t('personalize.expertise.label')}
              </label>
              <div className="grid grid-cols-3 gap-3">
                {(['beginner', 'intermediate', 'advanced'] as ExpertiseLevel[]).map((level) => (
                  <button
                    key={level}
                    type="button"
                    onClick={() => setFormData({ ...formData, expertiseLevel: level })}
                    className={`px-4 py-3 rounded-lg font-medium transition-all ${
                      formData.expertiseLevel === level
                        ? 'bg-blue-500 text-white shadow-lg scale-105'
                        : 'bg-gray-100 dark:bg-gray-700 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'
                    }`}
                    style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
                  >
                    {t(`personalize.expertise.${level}`)}
                  </button>
                ))}
              </div>
            </div>

            {/* Interests */}
            <div>
              <label
                htmlFor="interests"
                className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              >
                {t('personalize.interests.label')}
              </label>
              <input
                type="text"
                id="interests"
                value={formData.interests}
                onChange={(e) => setFormData({ ...formData, interests: e.target.value })}
                placeholder={t('personalize.interests.placeholder')}
                className="w-full px-4 py-3 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              />
              <p className="mt-1 text-xs text-gray-500 dark:text-gray-400">
                Separate multiple interests with commas
              </p>
            </div>

            {/* Additional Context */}
            <div>
              <label
                htmlFor="context"
                className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              >
                {t('personalize.context.label')}
              </label>
              <textarea
                id="context"
                rows={4}
                value={formData.additionalContext}
                onChange={(e) => setFormData({ ...formData, additionalContext: e.target.value })}
                placeholder={t('personalize.context.placeholder')}
                className="w-full px-4 py-3 border border-gray-300 dark:border-gray-600 rounded-lg bg-white dark:bg-gray-700 text-gray-900 dark:text-white placeholder-gray-400 focus:ring-2 focus:ring-blue-500 focus:border-transparent transition-all resize-none"
                style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
              />
            </div>

            {/* Success Message */}
            {showSuccess && (
              <div
                className="flex items-center gap-2 px-4 py-3 bg-green-50 dark:bg-green-900/20 border border-green-200 dark:border-green-800 rounded-lg text-green-800 dark:text-green-200 animate-fade-in"
                role="alert"
              >
                <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
                  <path
                    fillRule="evenodd"
                    d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z"
                    clipRule="evenodd"
                  />
                </svg>
                <span style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}>
                  {t('personalize.saved')}
                </span>
              </div>
            )}
          </form>

          {/* Footer */}
          <div className="px-6 py-4 bg-gray-50 dark:bg-gray-900 border-t border-gray-200 dark:border-gray-700 rounded-b-2xl flex gap-3 justify-end">
            <button
              type="button"
              onClick={onClose}
              className="px-5 py-2.5 rounded-lg font-medium text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-800 border border-gray-300 dark:border-gray-600 hover:bg-gray-50 dark:hover:bg-gray-700 transition-colors"
              style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
            >
              {t('personalize.cancel')}
            </button>
            <button
              type="submit"
              onClick={handleSubmit}
              className="px-5 py-2.5 rounded-lg font-medium text-white bg-blue-500 hover:bg-blue-600 shadow-lg hover:shadow-xl transition-all"
              style={{ fontFamily: language === 'ur' ? "'Noto Nastaliq Urdu', serif" : 'inherit' }}
            >
              {t('personalize.save')}
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default PersonalizeContextModal;
