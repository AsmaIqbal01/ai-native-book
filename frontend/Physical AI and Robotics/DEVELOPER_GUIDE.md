# Developer Guide - Multi-Agent UI Architecture

## Quick Start

### Using the Agent System

#### 1. Import the hooks:
```typescript
import { useChatbotAgent, useThemeAgent, useUserAgent } from '@/agents';
```

#### 2. Use in your components:
```typescript
function MyComponent() {
  const { chatbotState, openChatbot, closeChatbot } = useChatbotAgent();

  return (
    <button onClick={openChatbot}>
      Chat {chatbotState.unreadCount > 0 && `(${chatbotState.unreadCount})`}
    </button>
  );
}
```

---

## Agent Hooks Reference

### `useChatbotAgent()`
Manages chatbot visibility and interaction state.

**Returns:**
- `chatbotState` - Current chatbot state
  - `isVisible: boolean` - Whether chatbot is open
  - `isMinimized: boolean` - Whether chatbot is minimized
  - `unreadCount: number` - Number of unread messages
  - `lastInteraction: Date | null` - Last interaction timestamp
- `toggleChatbot()` - Toggle chatbot visibility
- `openChatbot()` - Open chatbot
- `closeChatbot()` - Close chatbot
- `minimizeChatbot()` - Minimize/restore chatbot
- `incrementUnread()` - Increment unread counter
- `resetUnread()` - Reset unread counter to 0
- `shouldAutoOpen: boolean` - Whether chatbot should auto-open
- `optimalPosition: object` - Optimal chatbot position

**Example:**
```typescript
const { chatbotState, toggleChatbot } = useChatbotAgent();

if (chatbotState.unreadCount > 0) {
  // Show notification
}
```

---

### `useThemeAgent()`
Manages theme preferences (light/dark mode).

**Returns:**
- `themeState` - Current theme state
  - `mode: 'light' | 'dark'` - Current theme mode
  - `accentColor: string` - Current accent color
- `toggleTheme()` - Toggle between light and dark mode

**Example:**
```typescript
const { themeState, toggleTheme } = useThemeAgent();

return (
  <button onClick={toggleTheme}>
    {themeState.mode === 'light' ? '🌙 Dark' : '☀️ Light'}
  </button>
);
```

---

### `useUserAgent()`
Manages user authentication and preferences.

**Returns:**
- `userState` - Current user state
  - `isAuthenticated: boolean` - Whether user is logged in
  - `preferences: Record<string, any>` - User preferences
- `login(userData)` - Log in user
- `logout()` - Log out user
- `updatePreferences(prefs)` - Update user preferences

**Example:**
```typescript
const { userState, login, logout } = useUserAgent();

if (userState.isAuthenticated) {
  return <button onClick={logout}>Logout</button>;
} else {
  return <button onClick={() => login({ email, password })}>Login</button>;
}
```

---

## Design System Reference

### CSS Custom Properties

#### Colors (Light Mode):
```css
--ifm-color-primary: #0066ff;        /* Electric Blue */
--accent-electric-cyan: #00e5ff;     /* Cyan Accent */
--accent-neon-purple: #a855f7;       /* Purple Accent */
--accent-laser-green: #10b981;       /* Green Accent */
--accent-plasma-orange: #ff6600;     /* Orange Accent */
```

#### Colors (Dark Mode):
```css
--ifm-color-primary: #00ffff;        /* Cyan */
--accent-neon-purple: #c084fc;       /* Vibrant Purple */
--ifm-background-color: #050a0f;     /* Deep Space */
```

#### Gradients:
```css
--gradient-primary: linear-gradient(135deg, #0066ff 0%, #a855f7 100%);
--gradient-success: linear-gradient(135deg, #10b981 0%, #00e5ff 100%);
--gradient-warning: linear-gradient(135deg, #ff6600 0%, #facc15 100%);
--gradient-danger: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
```

#### Glow Effects:
```css
--glow-primary: 0 0 20px rgba(0, 102, 255, 0.4);
--glow-accent: 0 0 30px rgba(168, 85, 247, 0.5);
--glow-hover: 0 0 40px rgba(0, 229, 255, 0.6);
```

### Using Design Tokens in Components:

```typescript
// Inline styles with gradients
<button
  style={{
    background: 'var(--gradient-primary)',
    boxShadow: 'var(--glow-primary)',
  }}
>
  Click Me
</button>

// Tailwind + custom properties
<div className="bg-primary hover:shadow-[var(--glow-hover)]">
  Content
</div>
```

---

## Component Patterns

### Button with Gradient:
```typescript
<button
  className="px-6 py-3 rounded-lg font-bold text-white transition-all duration-300 hover:scale-105"
  style={{
    background: 'var(--gradient-primary)',
    boxShadow: 'var(--glow-primary)',
  }}
>
  Action Button
</button>
```

### Card with Hover Effect:
```typescript
<div
  className="p-6 rounded-xl transition-all duration-400 hover:-translate-y-2"
  style={{
    border: '1px solid rgba(168, 85, 247, 0.2)',
    boxShadow: '0 8px 32px rgba(0, 0, 0, 0.12)',
  }}
>
  Card Content
</div>
```

### Input with Focus Glow:
```typescript
<input
  className="px-4 py-3 rounded-lg border-2 focus:outline-none transition-all"
  style={{
    borderColor: 'rgba(0, 102, 255, 0.3)',
  }}
  onFocus={(e) => {
    e.target.style.boxShadow = 'var(--glow-primary)';
  }}
/>
```

---

## Extending the Agent System

### Creating a New Agent:

1. **Define the state interface:**
```typescript
// src/agents/NotificationAgent.ts
export interface NotificationState {
  notifications: Notification[];
  unreadCount: number;
}

export type NotificationAction =
  | { type: 'ADD_NOTIFICATION'; payload: Notification }
  | { type: 'MARK_AS_READ'; payload: string }
  | { type: 'CLEAR_ALL' };
```

2. **Create the agent class:**
```typescript
export class NotificationAgent {
  private state: NotificationState;
  private listeners: Set<(state: NotificationState) => void> = new Set();

  constructor() {
    this.state = {
      notifications: [],
      unreadCount: 0,
    };
  }

  getState(): NotificationState {
    return { ...this.state };
  }

  subscribe(listener: (state: NotificationState) => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  dispatch(action: NotificationAction): void {
    this.state = this.reducer(this.state, action);
    this.notifyListeners();
  }

  private reducer(state: NotificationState, action: NotificationAction): NotificationState {
    switch (action.type) {
      case 'ADD_NOTIFICATION':
        return {
          ...state,
          notifications: [...state.notifications, action.payload],
          unreadCount: state.unreadCount + 1,
        };
      // ... other cases
      default:
        return state;
    }
  }

  private notifyListeners(): void {
    this.listeners.forEach(listener => listener(this.state));
  }
}

export const notificationAgent = new NotificationAgent();
```

3. **Create the React hook:**
```typescript
// src/agents/useNotificationAgent.ts
import { useEffect, useState, useCallback } from 'react';
import { notificationAgent } from './NotificationAgent';

export function useNotificationAgent() {
  const [state, setState] = useState(notificationAgent.getState());

  useEffect(() => {
    return notificationAgent.subscribe(setState);
  }, []);

  const addNotification = useCallback((notification: Notification) => {
    notificationAgent.dispatch({
      type: 'ADD_NOTIFICATION',
      payload: notification,
    });
  }, []);

  const markAsRead = useCallback((id: string) => {
    notificationAgent.dispatch({
      type: 'MARK_AS_READ',
      payload: id,
    });
  }, []);

  return {
    notifications: state.notifications,
    unreadCount: state.unreadCount,
    addNotification,
    markAsRead,
  };
}
```

4. **Export from index:**
```typescript
// src/agents/index.ts
export { NotificationAgent, notificationAgent } from './NotificationAgent';
export { useNotificationAgent } from './useNotificationAgent';
```

---

## Best Practices

### State Management:
1. **Keep agents focused** - Each agent should manage a single domain
2. **Avoid circular dependencies** - Agents should not depend on each other directly
3. **Use callbacks for side effects** - Don't put async logic in reducers
4. **Normalize state** - Keep state flat and normalized when possible

### Styling:
1. **Use CSS custom properties** - Always use design tokens for colors and effects
2. **Follow the gradient pattern** - Use defined gradients for consistency
3. **Add hover states** - All interactive elements should have hover feedback
4. **Respect accessibility** - Always include focus states and ARIA labels
5. **Test both themes** - Verify appearance in light and dark modes

### Performance:
1. **Memoize callbacks** - Use `useCallback` for event handlers
2. **Avoid inline functions** - Define handlers outside JSX when possible
3. **Use CSS transforms** - Prefer `transform` over `top/left` for animations
4. **Debounce expensive operations** - Add debouncing for heavy computations

### Accessibility:
1. **Keyboard navigation** - All interactive elements must be keyboard-accessible
2. **ARIA labels** - Provide descriptive labels for screen readers
3. **Focus indicators** - Maintain visible focus states
4. **Color contrast** - Ensure WCAG AA compliance for all text

---

## Troubleshooting

### Agent not updating:
```typescript
// Make sure you're using the hook, not calling the agent directly
❌ const state = uiStateAgent.getState(); // Won't update
✅ const { state } = useUIAgent(); // Will update
```

### Styles not applying:
```css
/* Check CSS variable exists */
background: var(--gradient-primary); /* ✅ Defined */
background: var(--gradient-custom); /* ❌ Not defined */
```

### Dark mode not working:
```typescript
// Ensure you're checking the theme context
const { darkMode } = useTheme(); // From existing ThemeContext
const { themeState } = useThemeAgent(); // From new agent system
```

---

## Migration Guide

### From old state management to agents:

**Before:**
```typescript
const [isOpen, setIsOpen] = useState(false);

<button onClick={() => setIsOpen(!isOpen)}>
  Toggle
</button>
```

**After:**
```typescript
const { chatbotState, toggleChatbot } = useChatbotAgent();

<button onClick={toggleChatbot}>
  Toggle
</button>
```

### Benefits:
- ✅ Centralized state
- ✅ Shared across components
- ✅ Intelligence built-in
- ✅ Analytics tracking
- ✅ Easier testing

---

## Testing

### Testing components with agents:

```typescript
import { render, screen } from '@testing-library/react';
import { uiStateAgent } from '@/agents';

test('chatbot opens when button clicked', () => {
  render(<ChatButton />);

  const button = screen.getByRole('button');
  button.click();

  expect(uiStateAgent.getState().chatbot.isVisible).toBe(true);
});
```

### Mocking agents in tests:

```typescript
jest.mock('@/agents', () => ({
  useChatbotAgent: () => ({
    chatbotState: { isVisible: false, unreadCount: 0 },
    openChatbot: jest.fn(),
    closeChatbot: jest.fn(),
  }),
}));
```

---

## Resources

- **Design System**: See `UI_REFACTOR_SUMMARY.md` for complete design documentation
- **Agent Architecture**: Review `src/agents/UIStateAgent.ts` for implementation details
- **CSS Variables**: Check `src/css/custom.css` for all available design tokens
- **Component Examples**: See `src/components/FloatingChatWidget.tsx` for patterns

---

**Last Updated**: 2025-12-20
**Maintainer**: UI/UX Engineering Team
**Version**: 2.0.0
