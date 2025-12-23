import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

/**
 * Authentication Context for user login state management
 *
 * Provides login/logout functionality and user session management
 * with localStorage persistence (stub implementation for now).
 */

interface User {
  id: string;
  name: string;
  email: string;
}

interface AuthContextType {
  isLoggedIn: boolean;
  user: User | null;
  login: (email: string, password: string) => Promise<void>;
  logout: () => void;
  updateUser: (userData: Partial<User>) => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(() => {
    // Only access localStorage on client side
    if (typeof window === 'undefined') return null;
    // Load saved user from localStorage
    const savedUser = localStorage.getItem('app-user');
    return savedUser ? JSON.parse(savedUser) : null;
  });

  const [isLoggedIn, setIsLoggedIn] = useState<boolean>(() => {
    if (typeof window === 'undefined') return false;
    return !!localStorage.getItem('app-user');
  });

  // Persist user data
  useEffect(() => {
    if (typeof window === 'undefined') return;
    if (user) {
      localStorage.setItem('app-user', JSON.stringify(user));
    } else {
      localStorage.removeItem('app-user');
    }
  }, [user]);

  // Stub login function - replace with actual API call
  const login = async (email: string, password: string): Promise<void> => {
    // Simulate API call
    return new Promise((resolve) => {
      setTimeout(() => {
        const mockUser: User = {
          id: '1',
          name: email.split('@')[0],
          email: email,
        };
        setUser(mockUser);
        setIsLoggedIn(true);
        resolve();
      }, 500);
    });
  };

  const logout = () => {
    setUser(null);
    setIsLoggedIn(false);
    if (typeof window !== 'undefined') {
      localStorage.removeItem('app-user');
      localStorage.removeItem('user-context-preferences');
    }
  };

  const updateUser = (userData: Partial<User>) => {
    if (user) {
      const updatedUser = { ...user, ...userData };
      setUser(updatedUser);
    }
  };

  return (
    <AuthContext.Provider value={{ isLoggedIn, user, login, logout, updateUser }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
