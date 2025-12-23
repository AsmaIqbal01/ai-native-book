/**
 * useUIAgent - React Hook for Multi-Agent UI State Management
 *
 * Provides seamless integration between the UIStateAgent and React components.
 * Ensures components re-render when relevant state changes occur.
 */

import { useEffect, useState, useCallback } from 'react';
import { uiStateAgent, UIState, UIAction } from './UIStateAgent';

/**
 * Hook to access UI state and dispatch actions
 */
export function useUIAgent() {
  const [state, setState] = useState<UIState>(uiStateAgent.getState());

  useEffect(() => {
    // Subscribe to state changes
    const unsubscribe = uiStateAgent.subscribe(newState => {
      setState(newState);
    });

    // Cleanup subscription on unmount
    return unsubscribe;
  }, []);

  const dispatch = useCallback((action: UIAction) => {
    uiStateAgent.dispatch(action);
  }, []);

  return {
    state,
    dispatch,
    agent: uiStateAgent,
  };
}

/**
 * Hook for chatbot-specific state and actions
 */
export function useChatbotAgent() {
  const { state, dispatch, agent } = useUIAgent();

  const toggleChatbot = useCallback(() => {
    dispatch({ type: 'CHATBOT_TOGGLE' });
    agent.logInteraction('Chatbot', 'toggle');
  }, [dispatch, agent]);

  const openChatbot = useCallback(() => {
    dispatch({ type: 'CHATBOT_OPEN' });
    agent.logInteraction('Chatbot', 'open');
  }, [dispatch, agent]);

  const closeChatbot = useCallback(() => {
    dispatch({ type: 'CHATBOT_CLOSE' });
    agent.logInteraction('Chatbot', 'close');
  }, [dispatch, agent]);

  const minimizeChatbot = useCallback(() => {
    dispatch({ type: 'CHATBOT_MINIMIZE' });
    agent.logInteraction('Chatbot', 'minimize');
  }, [dispatch, agent]);

  const incrementUnread = useCallback(() => {
    dispatch({ type: 'CHATBOT_INCREMENT_UNREAD' });
  }, [dispatch]);

  const resetUnread = useCallback(() => {
    dispatch({ type: 'CHATBOT_RESET_UNREAD' });
  }, [dispatch]);

  return {
    chatbotState: state.chatbot,
    toggleChatbot,
    openChatbot,
    closeChatbot,
    minimizeChatbot,
    incrementUnread,
    resetUnread,
    shouldAutoOpen: agent.shouldAutoOpenChatbot(),
    optimalPosition: agent.getOptimalChatbotPosition(),
  };
}

/**
 * Hook for theme-specific state and actions
 */
export function useThemeAgent() {
  const { state, dispatch, agent } = useUIAgent();

  const toggleTheme = useCallback(() => {
    dispatch({ type: 'THEME_TOGGLE' });
    agent.logInteraction('Theme', 'toggle', { mode: state.theme.mode });
  }, [dispatch, agent, state.theme.mode]);

  return {
    themeState: state.theme,
    toggleTheme,
  };
}

/**
 * Hook for user-specific state and actions
 */
export function useUserAgent() {
  const { state, dispatch, agent } = useUIAgent();

  const login = useCallback((userData: any) => {
    dispatch({ type: 'USER_LOGIN', payload: userData });
    agent.logInteraction('User', 'login');
  }, [dispatch, agent]);

  const logout = useCallback(() => {
    dispatch({ type: 'USER_LOGOUT' });
    agent.logInteraction('User', 'logout');
  }, [dispatch, agent]);

  const updatePreferences = useCallback((preferences: Record<string, any>) => {
    dispatch({ type: 'PREFERENCES_UPDATE', payload: preferences });
    agent.logInteraction('User', 'update_preferences');
  }, [dispatch, agent]);

  return {
    userState: state.user,
    login,
    logout,
    updatePreferences,
  };
}
