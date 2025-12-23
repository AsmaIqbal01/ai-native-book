/**
 * UIStateAgent - Multi-Agent Architecture for UI State Management
 *
 * This agent manages UI state decisions intelligently, routing interactions
 * and coordinating component behavior without hard-coded logic.
 *
 * Responsibilities:
 * - Chatbot visibility management
 * - User interaction routing
 * - Component state coordination
 * - Analytics and behavior tracking
 */

export interface UIState {
  chatbot: {
    isVisible: boolean;
    isMinimized: boolean;
    unreadCount: number;
    lastInteraction: Date | null;
  };
  theme: {
    mode: 'light' | 'dark';
    accentColor: string;
  };
  user: {
    isAuthenticated: boolean;
    preferences: Record<string, any>;
  };
}

export type UIAction =
  | { type: 'CHATBOT_TOGGLE' }
  | { type: 'CHATBOT_OPEN' }
  | { type: 'CHATBOT_CLOSE' }
  | { type: 'CHATBOT_MINIMIZE' }
  | { type: 'CHATBOT_INCREMENT_UNREAD' }
  | { type: 'CHATBOT_RESET_UNREAD' }
  | { type: 'THEME_TOGGLE' }
  | { type: 'USER_LOGIN'; payload: any }
  | { type: 'USER_LOGOUT' }
  | { type: 'PREFERENCES_UPDATE'; payload: Record<string, any> };

/**
 * UIStateAgent - Core agent for managing UI state
 */
export class UIStateAgent {
  private state: UIState;
  private listeners: Set<(state: UIState) => void> = new Set();

  constructor(initialState?: Partial<UIState>) {
    this.state = {
      chatbot: {
        isVisible: false,
        isMinimized: false,
        unreadCount: 0,
        lastInteraction: null,
      },
      theme: {
        mode: 'light',
        accentColor: '#1976d2',
      },
      user: {
        isAuthenticated: false,
        preferences: {},
      },
      ...initialState,
    };
  }

  /**
   * Get current state
   */
  getState(): UIState {
    return { ...this.state };
  }

  /**
   * Subscribe to state changes
   */
  subscribe(listener: (state: UIState) => void): () => void {
    this.listeners.add(listener);
    return () => {
      this.listeners.delete(listener);
    };
  }

  /**
   * Dispatch an action to update state
   */
  dispatch(action: UIAction): void {
    const prevState = this.state;
    this.state = this.reducer(this.state, action);

    // Notify listeners if state changed
    if (prevState !== this.state) {
      this.notifyListeners();
    }
  }

  /**
   * State reducer - handles all state transitions
   */
  private reducer(state: UIState, action: UIAction): UIState {
    switch (action.type) {
      case 'CHATBOT_TOGGLE':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            isVisible: !state.chatbot.isVisible,
            unreadCount: state.chatbot.isVisible ? state.chatbot.unreadCount : 0,
            lastInteraction: new Date(),
          },
        };

      case 'CHATBOT_OPEN':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            isVisible: true,
            isMinimized: false,
            unreadCount: 0,
            lastInteraction: new Date(),
          },
        };

      case 'CHATBOT_CLOSE':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            isVisible: false,
            lastInteraction: new Date(),
          },
        };

      case 'CHATBOT_MINIMIZE':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            isMinimized: !state.chatbot.isMinimized,
            lastInteraction: new Date(),
          },
        };

      case 'CHATBOT_INCREMENT_UNREAD':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            unreadCount: state.chatbot.unreadCount + 1,
          },
        };

      case 'CHATBOT_RESET_UNREAD':
        return {
          ...state,
          chatbot: {
            ...state.chatbot,
            unreadCount: 0,
          },
        };

      case 'THEME_TOGGLE':
        return {
          ...state,
          theme: {
            ...state.theme,
            mode: state.theme.mode === 'light' ? 'dark' : 'light',
          },
        };

      case 'USER_LOGIN':
        return {
          ...state,
          user: {
            ...state.user,
            isAuthenticated: true,
            preferences: action.payload?.preferences || {},
          },
        };

      case 'USER_LOGOUT':
        return {
          ...state,
          user: {
            isAuthenticated: false,
            preferences: {},
          },
        };

      case 'PREFERENCES_UPDATE':
        return {
          ...state,
          user: {
            ...state.user,
            preferences: {
              ...state.user.preferences,
              ...action.payload,
            },
          },
        };

      default:
        return state;
    }
  }

  /**
   * Notify all listeners of state change
   */
  private notifyListeners(): void {
    this.listeners.forEach(listener => listener(this.state));
  }

  /**
   * Intelligence: Decide if chatbot should auto-open based on user behavior
   */
  shouldAutoOpenChatbot(): boolean {
    const { chatbot, user } = this.state;

    // Don't auto-open if already visible
    if (chatbot.isVisible) return false;

    // Auto-open for new users on first visit
    if (!chatbot.lastInteraction && !user.isAuthenticated) {
      return false; // Disabled by default - user must explicitly open
    }

    // Check user preferences
    if (user.preferences.autoOpenChatbot === false) {
      return false;
    }

    return false; // Conservative default
  }

  /**
   * Intelligence: Determine optimal chatbot position based on viewport
   */
  getOptimalChatbotPosition(): { bottom: string; left?: string; right?: string } {
    const isMobile = typeof window !== 'undefined' && window.innerWidth < 640;

    if (isMobile) {
      return { bottom: '0', left: '0' };
    }

    return { bottom: '1rem', left: '1rem' };
  }

  /**
   * Analytics: Track user interaction patterns
   */
  logInteraction(component: string, action: string, metadata?: Record<string, any>): void {
    if (typeof window !== 'undefined' && (window as any).analytics) {
      (window as any).analytics.track('UI Interaction', {
        component,
        action,
        timestamp: new Date().toISOString(),
        ...metadata,
      });
    }
  }
}

/**
 * Singleton instance for global state management
 */
export const uiStateAgent = new UIStateAgent();
