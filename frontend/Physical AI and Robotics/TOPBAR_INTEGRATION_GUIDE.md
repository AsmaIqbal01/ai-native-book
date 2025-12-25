# TopBar Component - Integration Guide

## Overview

The TopBar component is a comprehensive navigation bar with multilingual support, authentication, and personalization features for your RAG Chatbot frontend.

## Features

✅ **Language Translation** - Switch between English and Urdu with RTL support
✅ **GitHub Integration** - Direct link to repository
✅ **Authentication** - Login/Logout functionality with user session
✅ **Personalization** - Context customization modal (visible only when logged in)
✅ **Responsive Design** - Mobile and desktop optimized
✅ **Dark Mode Support** - Integrates with existing theme context
✅ **Accessibility** - ARIA labels, keyboard navigation

---

## File Structure

```
src/
├── components/
│   ├── TopBar.tsx                    # Main TopBar component
│   ├── PersonalizeContextModal.tsx   # Personalization modal
│   └── index.ts                      # Component exports
├── contexts/
│   ├── TranslationContext.tsx        # Language management
│   ├── AuthContext.tsx               # Authentication state
│   ├── UserPreferencesContext.tsx    # User preferences storage
│   └── index.ts                      # Context exports
```

---

## Installation & Setup

### Step 1: Wrap Your App with Context Providers

Update your root `App.tsx` or main entry point:

```typescript
import React from 'react';
import {
  ThemeProvider,
  TranslationProvider,
  AuthProvider,
  UserPreferencesProvider,
} from './contexts';
import ChatApp from './components/ChatApp';
import TopBar from './components/TopBar';

const App: React.FC = () => {
  return (
    <ThemeProvider>
      <TranslationProvider>
        <AuthProvider>
          <UserPreferencesProvider>
            <div className="h-screen bg-gray-50 dark:bg-gray-900 flex flex-col">
              {/* TopBar at the top */}
              <TopBar />

              {/* Main content area */}
              <div className="flex-1 overflow-hidden">
                <ChatApp />
              </div>
            </div>
          </UserPreferencesProvider>
        </AuthProvider>
      </TranslationProvider>
    </ThemeProvider>
  );
};

export default App;
```

### Step 2: Update Chat Page (if using Docusaurus)

For `src/pages/chat.tsx`:

```typescript
import React from 'react';
import {
  ThemeProvider,
  TranslationProvider,
  AuthProvider,
  UserPreferencesProvider,
} from '../contexts';
import TopBar from '../components/TopBar';
import ChatApp from '../components/ChatApp';

export default function Chat(): JSX.Element {
  return (
    <ThemeProvider>
      <TranslationProvider>
        <AuthProvider>
          <UserPreferencesProvider>
            <div className="h-screen bg-gray-50 dark:bg-gray-900 flex flex-col">
              <TopBar />
              <div className="flex-1">
                <ChatApp />
              </div>
            </div>
          </UserPreferencesProvider>
        </AuthProvider>
      </TranslationProvider>
    </ThemeProvider>
  );
}
```

---

## Component Usage

### Basic Usage

```typescript
import TopBar from './components/TopBar';

function MyPage() {
  return (
    <div>
      <TopBar />
      {/* Your page content */}
    </div>
  );
}
```

### With Custom Styling

```typescript
<TopBar className="shadow-lg" />
```

---

## Context Hooks

### Translation Hook

```typescript
import { useTranslation } from './contexts/TranslationContext';

function MyComponent() {
  const { language, toggleLanguage, t } = useTranslation();

  return (
    <div>
      <p>{t('chat.welcome')}</p>
      <button onClick={toggleLanguage}>
        Switch to {language === 'en' ? 'Urdu' : 'English'}
      </button>
    </div>
  );
}
```

### Authentication Hook

```typescript
import { useAuth } from './contexts/AuthContext';

function MyComponent() {
  const { isLoggedIn, user, login, logout } = useAuth();

  return (
    <div>
      {isLoggedIn ? (
        <div>
          <p>Welcome, {user?.name}</p>
          <button onClick={logout}>Logout</button>
        </div>
      ) : (
        <button onClick={() => login('user@example.com', 'password')}>
          Login
        </button>
      )}
    </div>
  );
}
```

### User Preferences Hook

```typescript
import { useUserPreferences } from './contexts/UserPreferencesContext';

function MyComponent() {
  const { preferences, updatePreferences, hasPreferences } = useUserPreferences();

  return (
    <div>
      {hasPreferences && (
        <p>Expertise: {preferences.expertiseLevel}</p>
      )}
      <button
        onClick={() =>
          updatePreferences({
            expertiseLevel: 'advanced',
            interests: ['ROS2', 'Digital Twins'],
          })
        }
      >
        Update Preferences
      </button>
    </div>
  );
}
```

---

## Customizing Translations

Edit `src/contexts/TranslationContext.tsx` to add or modify translations:

```typescript
const translations: Record<Language, Record<string, string>> = {
  en: {
    'my.custom.key': 'My Custom Text',
    // ... other translations
  },
  ur: {
    'my.custom.key': 'میری حسب ضرورت متن',
    // ... other translations
  },
};
```

Then use in components:

```typescript
const { t } = useTranslation();
<p>{t('my.custom.key')}</p>
```

---

## Updating ChatApp to Use Translation

Update `src/components/ChatApp.tsx` to support multilingual interface:

```typescript
import { useTranslation } from '../contexts/TranslationContext';
import { useUserPreferences } from '../contexts/UserPreferencesContext';

const ChatApp: React.FC = () => {
  const { t, language } = useTranslation();
  const { preferences } = useUserPreferences();

  // Use translations for UI text
  const welcomeMessage = {
    content: t('chat.welcome'),
    role: 'assistant',
    // ...
  };

  // Personalize chatbot queries with user context
  const queryBackend = async (userMessage: string): Promise<string> => {
    const contextualMessage = preferences.name
      ? `User context: ${preferences.name}, ${preferences.expertiseLevel} level, interested in ${preferences.interests.join(', ')}. ${preferences.additionalContext}\n\nQuestion: ${userMessage}`
      : userMessage;

    // Send contextualMessage to backend...
  };
};
```

---

## Styling & Theming

### Urdu Font Support

The component automatically applies Urdu font (`Noto Nastaliq Urdu`) when language is set to Urdu. To add the font:

**Option 1: Google Fonts (Recommended)**

Add to `src/css/custom.css`:

```css
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;500;600;700&display=swap');
```

**Option 2: Self-hosted**

Download the font and add to your CSS:

```css
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/NotoNastaliqUrdu-Regular.ttf') format('truetype');
}
```

### Dark Mode

The TopBar automatically supports dark mode through the `ThemeContext`. No additional configuration needed.

---

## API Integration

### Authentication (Production)

Replace the stub login function in `src/contexts/AuthContext.tsx`:

```typescript
const login = async (email: string, password: string): Promise<void> => {
  const response = await fetch(`${API_BASE_URL}/auth/login`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password }),
  });

  if (!response.ok) {
    throw new Error('Login failed');
  }

  const data = await response.json();
  const user: User = {
    id: data.user.id,
    name: data.user.name,
    email: data.user.email,
  };

  setUser(user);
  setIsLoggedIn(true);

  // Store auth token
  localStorage.setItem('auth-token', data.token);
};
```

### Sending User Preferences to Backend

When querying the RAG backend, include user context:

```typescript
const response = await queryRAG({
  question: userMessage,
  top_k: 5,
  user_context: {
    name: preferences.name,
    expertise_level: preferences.expertiseLevel,
    interests: preferences.interests,
    additional_context: preferences.additionalContext,
  },
});
```

---

## Troubleshooting

### Issue: Translations not working

**Solution:** Ensure all components are wrapped with `TranslationProvider`:

```typescript
<TranslationProvider>
  <YourComponent />
</TranslationProvider>
```

### Issue: Personalize button not visible

**Solution:** Check that user is logged in:

```typescript
const { isLoggedIn } = useAuth();
console.log('Is logged in:', isLoggedIn);
```

### Issue: RTL text direction not applying

**Solution:** The `TranslationContext` automatically sets `dir="rtl"` on `<html>` for Urdu. Verify the provider is at root level.

### Issue: Styles not loading

**Solution:** Ensure Tailwind CSS is properly configured in `tailwind.config.js` and imported in your entry CSS.

---

## Mobile Responsiveness

The TopBar is fully responsive:

- **Desktop:** Shows all buttons with labels
- **Tablet:** Hides some labels, keeps icons
- **Mobile:** Stacks user info below TopBar, collapses to icon-only buttons

Test responsive behavior:

```bash
npm start
# Then resize browser window or use DevTools device toolbar
```

---

## Accessibility Features

- ✅ ARIA labels on all buttons
- ✅ Keyboard navigation support
- ✅ Focus indicators
- ✅ Screen reader friendly
- ✅ Semantic HTML
- ✅ Color contrast compliant

---

## Performance Optimization

### Lazy Loading Modal

```typescript
import { lazy, Suspense } from 'react';

const PersonalizeContextModal = lazy(() => import('./PersonalizeContextModal'));

// In component
<Suspense fallback={<div>Loading...</div>}>
  <PersonalizeContextModal isOpen={isOpen} onClose={onClose} />
</Suspense>
```

### Memoization

```typescript
import { memo } from 'react';

const TopBar = memo(({ className }: TopBarProps) => {
  // Component code...
});
```

---

## Testing

### Unit Tests Example (Jest + React Testing Library)

```typescript
import { render, screen, fireEvent } from '@testing-library/react';
import { TranslationProvider } from '../contexts/TranslationContext';
import TopBar from './TopBar';

test('toggles language when translate button is clicked', () => {
  render(
    <TranslationProvider>
      <TopBar />
    </TranslationProvider>
  );

  const translateBtn = screen.getByLabelText(/switch to urdu/i);
  fireEvent.click(translateBtn);

  expect(screen.getByText('اردو')).toBeInTheDocument();
});
```

---

## Advanced Customization

### Custom Buttons

Add your own buttons to the TopBar:

```typescript
// In TopBar.tsx, add to the right section
<button
  onClick={handleCustomAction}
  className="inline-flex items-center gap-2 px-4 py-2 rounded-lg..."
  aria-label="Custom Action"
>
  <svg>...</svg>
  <span>Custom</span>
</button>
```

### Theming

Override Tailwind classes in `tailwind.config.js`:

```javascript
module.exports = {
  theme: {
    extend: {
      colors: {
        primary: {
          500: '#your-color',
        },
      },
    },
  },
};
```

---

## Support & Contribution

- **GitHub:** https://github.com/AsmaIqbal01/ai-native-book
- **Issues:** Report bugs or request features in GitHub Issues
- **Documentation:** Update this guide when adding new features

---

## License

MIT License - Feel free to use and modify for your projects.
