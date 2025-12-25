import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

/**
 * User Preferences Context for personalization
 *
 * Manages user-specific preferences for chatbot interactions,
 * including expertise level, interests, and custom context.
 */

export type ExpertiseLevel = 'beginner' | 'intermediate' | 'advanced';

export interface UserPreferences {
  name: string;
  expertiseLevel: ExpertiseLevel;
  interests: string[];
  additionalContext: string;
}

interface UserPreferencesContextType {
  preferences: UserPreferences;
  updatePreferences: (prefs: Partial<UserPreferences>) => void;
  resetPreferences: () => void;
  hasPreferences: boolean;
}

const defaultPreferences: UserPreferences = {
  name: '',
  expertiseLevel: 'beginner',
  interests: [],
  additionalContext: '',
};

const UserPreferencesContext = createContext<UserPreferencesContextType | undefined>(undefined);

interface UserPreferencesProviderProps {
  children: ReactNode;
}

export const UserPreferencesProvider: React.FC<UserPreferencesProviderProps> = ({ children }) => {
  const [preferences, setPreferences] = useState<UserPreferences>(() => {
    // Only access localStorage on client side
    if (typeof window === 'undefined') return defaultPreferences;
    // Load saved preferences from localStorage
    const savedPrefs = localStorage.getItem('user-context-preferences');
    return savedPrefs ? JSON.parse(savedPrefs) : defaultPreferences;
  });

  // Check if user has set any preferences
  const hasPreferences = preferences.name !== '' ||
                         preferences.interests.length > 0 ||
                         preferences.additionalContext !== '';

  // Persist preferences
  useEffect(() => {
    if (typeof window === 'undefined') return;
    localStorage.setItem('user-context-preferences', JSON.stringify(preferences));
  }, [preferences]);

  const updatePreferences = (prefs: Partial<UserPreferences>) => {
    setPreferences(prev => ({
      ...prev,
      ...prefs,
    }));
  };

  const resetPreferences = () => {
    setPreferences(defaultPreferences);
    if (typeof window !== 'undefined') {
      localStorage.removeItem('user-context-preferences');
    }
  };

  return (
    <UserPreferencesContext.Provider
      value={{ preferences, updatePreferences, resetPreferences, hasPreferences }}
    >
      {children}
    </UserPreferencesContext.Provider>
  );
};

export const useUserPreferences = (): UserPreferencesContextType => {
  const context = useContext(UserPreferencesContext);
  if (!context) {
    throw new Error('useUserPreferences must be used within a UserPreferencesProvider');
  }
  return context;
};
