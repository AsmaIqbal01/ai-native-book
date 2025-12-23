# Translation System Implementation Guide

## 🌐 Overview

This document describes the comprehensive translation system implemented for the AI-Native Robotics Textbook. The system enables full-page, seamless translation between English and Urdu.

## 📋 Architecture

### Multi-Layered Translation System (Hybrid Approach)

```
┌─────────────────────────────────────────────────────────────┐
│                    User Interface Layer                      │
│  (TopBar, Footer, Buttons, Navigation, Chat, etc.)          │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                  TranslationContext                          │
│  • Global language state (EN ⇄ UR)                          │
│  • localStorage persistence                                  │
│  • HTML lang & dir attributes                               │
│  • Global font application                                  │
└───────────────────────┬─────────────────────────────────────┘
                        │
        ┌───────────────┴───────────────┐
        ▼                               ▼
┌──────────────────┐         ┌──────────────────────┐
│  Static UI       │         │  Dynamic Content     │
│  Translations    │         │  Translation         │
│  (Dictionary)    │         │  (TranslationService)│
└──────────────────┘         └──────────────────────┘
```

## 🎯 Key Components

### 1. **TranslationContext** (`src/contexts/TranslationContext.tsx`)
- **Purpose**: Global language state management
- **Features**:
  - Language toggle (EN ⇄ UR)
  - localStorage persistence
  - RTL/LTR direction switching
  - Global Urdu font application
  - Translation dictionary for UI elements

**Usage**:
```tsx
import { useTranslation } from '@/contexts/TranslationContext';

function MyComponent() {
  const { language, toggleLanguage, t } = useTranslation();
  return <button onClick={toggleLanguage}>{t('common.close')}</button>;
}
```

### 2. **TranslationService** (`src/services/TranslationService.ts`)
- **Purpose**: Core translation engine with caching
- **Features**:
  - Singleton pattern for global access
  - Translation caching for performance
  - Bulk translation support
  - Export/import cache for persistence

**Usage**:
```tsx
import TranslationService from '@/services/TranslationService';

// Add translations
TranslationService.addTranslation('Hello', 'ہیلو', 'ur');

// Get translation
const translated = TranslationService.getTranslation('Hello', 'ur');
```

### 3. **Content Translations** (`src/translations/contentTranslations.ts`)
- **Purpose**: Comprehensive Urdu translations for book content
- **Contains**:
  - Chapter titles
  - Section headings
  - Common MDX phrases
  - Technical terms
  - Button/action text
  - 100+ translation mappings

### 4. **MDXTranslator Components** (`src/components/MDXTranslator.tsx`)

Three specialized components for translating MDX content:

#### a. `TranslatableText`
Wraps text content for translation:
```tsx
<TranslatableText text="Learning Objectives" as="h2" />
```

#### b. `TranslatableHeading`
Specialized for headings with SEO support:
```tsx
<TranslatableHeading level={1} id="intro">
  Introduction to Physical AI
</TranslatableHeading>
```

#### c. `MDXTranslator`
Wraps entire MDX sections:
```tsx
<MDXTranslator>
  <h1>Chapter Title</h1>
  <p>Content here...</p>
</MDXTranslator>
```

### 5. **useContentTranslation Hook** (`src/hooks/useContentTranslation.ts`)

Simplified hook for content translation:
```tsx
import useContentTranslation from '@/hooks/useContentTranslation';

function MyPage() {
  const tc = useContentTranslation();
  return (
    <div>
      <h1>{tc('Learning Objectives')}</h1>
      <p>{tc('This section covers...')}</p>
    </div>
  );
}
```

### 6. **Custom Footer** (`src/theme/Footer/index.tsx`)
- Fully translated footer
- Replaces default Docusaurus footer
- All links and text translated

## 🎨 Styling & Fonts

### Global Urdu Font Application

When Urdu is active, the system:
1. Adds `urdu-active` class to `<html>`
2. Applies `Noto Nastaliq Urdu` font globally
3. Sets `direction: rtl` and `text-align: right`
4. Preserves monospace fonts for code blocks

**CSS** (`src/css/custom.css`):
```css
html.urdu-active * {
  font-family: 'Noto Nastaliq Urdu', serif !important;
}

/* Preserve code fonts */
html.urdu-active code,
html.urdu-active pre {
  font-family: 'Courier New', Courier, monospace !important;
}
```

### RTL Layout Support
- Navbar items reversed
- Sidebar spacing adjusted
- Menu arrows flipped
- Text alignment right-aligned

## 🔧 Implementation Steps

### Step 1: Add Translations

Add new translations to `src/translations/contentTranslations.ts`:
```tsx
export const contentTranslations: Record<string, string> = {
  'Your English Text': 'آپ کا اردو متن',
  // ... more translations
};
```

### Step 2: Use in Components

**Option A: Use Hook**
```tsx
const tc = useContentTranslation();
<h1>{tc('Your English Text')}</h1>
```

**Option B: Use Component**
```tsx
<TranslatableText text="Your English Text" as="p" />
```

**Option C: Use Context Directly**
```tsx
const { t } = useTranslation();
<button>{t('common.close')}</button>
```

### Step 3: Initialize on App Load

Translations are automatically initialized in `src/theme/Root.tsx`:
```tsx
useEffect(() => {
  TranslationService.addTranslations(contentTranslations, 'ur');
}, []);
```

## ✅ Translation Checklist

When adding a new page or component:

- [ ] Add translations to `contentTranslations.ts`
- [ ] Wrap static text with `tc()` or `TranslatableText`
- [ ] Use `t()` for UI elements (buttons, labels)
- [ ] Test both EN and UR versions
- [ ] Verify RTL layout
- [ ] Check font rendering
- [ ] Ensure code blocks remain monospace
- [ ] Verify visual integrity (colors, spacing)

## 🧪 Testing

### Manual Testing Steps

1. **Toggle Language**
   - Click EN/اردو button in TopBar
   - Verify entire page switches language
   - Check localStorage persistence

2. **Visual Verification**
   - Colors unchanged ✓
   - Gradients unchanged ✓
   - Shadows unchanged ✓
   - Spacing unchanged ✓
   - Only text changes ✓

3. **RTL Layout**
   - Text right-aligned ✓
   - Navbar reversed ✓
   - Sidebar adjusted ✓
   - Code blocks preserved ✓

4. **Font Application**
   - All text uses Noto Nastaliq Urdu ✓
   - Code blocks use monospace ✓
   - Headings properly rendered ✓

5. **Content Translation**
   - Chapter titles translated ✓
   - Headings translated ✓
   - Paragraphs translated ✓
   - Buttons translated ✓
   - Footer translated ✓

## 📊 Translation Coverage

Current translation coverage:

| Category | Count | Status |
|----------|-------|--------|
| UI Elements | 50+ | ✅ Complete |
| Footer Links | 12+ | ✅ Complete |
| Content Headings | 30+ | ✅ Complete |
| Common Phrases | 40+ | ✅ Complete |
| Technical Terms | 20+ | ✅ Complete |
| Button Text | 15+ | ✅ Complete |

**Total Translations**: 167+ mappings

## 🚀 Future Enhancements

### Phase 2 (Optional):
1. **Backend Integration**
   - Connect to RAG backend for dynamic translation
   - AI-powered content translation
   - Real-time translation for user input

2. **Advanced Features**
   - Translation history
   - User-contributed translations
   - Auto-detection of missing translations
   - Translation analytics

3. **Performance**
   - Lazy-load translations
   - Webpack optimization
   - Translation splitting by route

## 🐛 Troubleshooting

### Issue: Font not applying
**Solution**: Check `<html>` has `urdu-active` class when language is UR

### Issue: Layout breaking in RTL
**Solution**: Verify CSS has proper RTL adjustments in `custom.css`

### Issue: Translation not showing
**Solution**: Ensure translation exists in `contentTranslations.ts` and is loaded in `Root.tsx`

### Issue: Code blocks have Urdu font
**Solution**: Check CSS exclusion rules for `code` and `pre` elements

## 📝 Adding New Languages (Future)

To add a new language (e.g., Arabic, Hindi):

1. Update `Language` type in `TranslationContext.tsx`
2. Add language translations to `translations` dictionary
3. Add content translations to `contentTranslations.ts`
4. Import appropriate Google Font
5. Add CSS rules for new language
6. Update language toggle button

## 📚 Resources

- [Noto Nastaliq Urdu Font](https://fonts.google.com/noto/specimen/Noto+Nastaliq+Urdu)
- [RTL Best Practices](https://rtlstyling.com/posts/rtl-styling)
- [Docusaurus i18n](https://docusaurus.io/docs/i18n/introduction)
- [React Context API](https://react.dev/reference/react/useContext)

---

**Author**: AI-Native Development Team
**Last Updated**: December 2025
**Version**: 1.0.0
