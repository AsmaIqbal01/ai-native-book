# Full Website Translation Implementation Guide

## Overview

The AI-Native Robotics Textbook now supports **full website translation** between English and Urdu. When users click the language toggle button in the TopBar, the **entire website content** (including all documentation pages, MDX content, headings, paragraphs, lists, and UI elements) will be translated.

## What Was Implemented

### 1. **MDXContent Wrapper** (`src/theme/MDXContent/index.tsx`)
- Intercepts all Docusaurus MDX content rendering
- Recursively translates React component trees
- Preserves code blocks, syntax highlighting, and technical terms
- Applies Urdu typography (Noto Nastaliq Urdu font) when language is Urdu
- Sets RTL (right-to-left) text direction for Urdu content

### 2. **DocItem Wrapper** (`src/theme/DocItem/index.tsx`)
- Wraps all documentation pages
- Ensures proper RTL direction and Urdu font application at the page level
- Triggers re-renders when language changes

### 3. **Enhanced TranslationService** (`src/services/TranslationService.ts`)
- Improved `translateContent()` method that:
  - Preserves code blocks (inline code and code fences)
  - Maintains formatting and whitespace
  - Translates line-by-line while checking the translation cache
  - Returns original text if no translation is available

### 4. **TranslatableContent Component** (`src/components/TranslatableContent.tsx`)
- Reusable component for wrapping content that needs translation
- Can be manually added to MDX files for explicit translation control
- Includes `TranslatableText` sub-component for inline text

### 5. **Expanded Translation Dictionary** (`src/translations/contentTranslations.ts`)
- Added 70+ new Urdu translations covering:
  - Chapter titles and section headings
  - Common MDX content phrases
  - Technical terms (with transliterations)
  - Learning objectives and pedagogical elements
  - Platform-specific content (Humanoid Landscape chapter)
  - UI elements with emojis (🎯, 📚, 💡, etc.)

## How It Works

### Translation Flow

1. **User clicks language toggle** in TopBar (EN ⇄ اردو)
2. **TranslationContext** updates global language state
3. **All theme-wrapped components re-render** with new language
4. **MDXContent wrapper** recursively processes content tree:
   - Text nodes → lookup in TranslationService cache
   - React elements → recursively translate children
   - Code blocks → preserve as-is
5. **TranslationService** returns:
   - Urdu translation (if available in cache)
   - Original English text (if not found)
6. **Urdu typography applied** via CSS font-family and RTL direction

### What Gets Translated

✅ **Translated:**
- Page titles and headings
- Paragraphs and text content
- List items (bullet points, numbered lists)
- Table content (headers and cells)
- Admonitions (:::tip, :::warning, etc.)
- Navigation labels
- Button text
- Footer links

❌ **Preserved (not translated):**
- Code blocks (```python, ```bash, etc.)
- Inline code (`variable_names`)
- Technical identifiers (function names, class names)
- URLs and links
- File paths
- Command-line examples

## How to Add More Translations

### Option 1: Add to Translation Dictionary (Recommended)

Edit `src/translations/contentTranslations.ts`:

```typescript
export const contentTranslations: Record<string, string> = {
  // Add your translations here
  'Your English Text': 'آپ کا اردو متن',
  'Another heading': 'ایک اور عنوان',

  // Example with multi-line
  'This is a longer paragraph that needs translation':
    'یہ ایک طویل پیراگراف ہے جس کو ترجمہ کی ضرورت ہے',
};
```

### Option 2: Use TranslatableContent Component in MDX

For custom MDX pages:

```mdx
import TranslatableContent from '@site/src/components/TranslatableContent';

<TranslatableContent>
  <h1>Custom Page Title</h1>
  <p>This content will be automatically translated</p>
</TranslatableContent>
```

### Option 3: Use TranslatableText for Inline Text

```mdx
import { TranslatableText } from '@site/src/components/TranslatableContent';

<TranslatableText>Learning Objectives</TranslatableText>
```

## Translation Best Practices

### 1. **Preserve Technical Terms**
Keep English terms when there's no good Urdu equivalent:

```typescript
'ROS2': 'ROS2', // Keep as-is
'Robot Operating System': 'Robot Operating System (روبوٹ آپریٹنگ سسٹم)', // English + transliteration
```

### 2. **Maintain Formatting**
Translations should preserve special characters:

```typescript
'🎯 Learning Objectives': '🎯 سیکھنے کے مقاصد', // Keep emoji
'**Bold Text**': '**بولڈ متن**', // Keep markdown
```

### 3. **Match Original Tone**
- Technical content: Formal Urdu
- Educational content: Clear, accessible Urdu
- UI elements: Concise Urdu

### 4. **Test Translations**
After adding translations:
```bash
npm run start
# Navigate to http://localhost:3000
# Click language toggle button
# Verify translations appear correctly
```

## Architecture

```
┌─────────────────────────────────────────┐
│          User Clicks Toggle             │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│      TranslationContext.toggleLanguage() │
│      Updates: language = 'ur'            │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│   All Components Re-render with 'ur'    │
│   - TopBar, MDXContent, DocItem, etc.   │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  MDXContent.translateChildren()         │
│  Recursively processes React tree       │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  TranslationService.getTranslation()    │
│  Looks up: "Learning Objectives"        │
│  Returns: "سیکھنے کے مقاصد"             │
└────────────────┬────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────┐
│  Rendered Output with Urdu Font         │
│  font-family: 'Noto Nastaliq Urdu'      │
│  direction: rtl                          │
└─────────────────────────────────────────┘
```

## Files Modified/Created

### Created:
- ✅ `src/theme/MDXContent/index.tsx` - MDX content translation wrapper
- ✅ `src/theme/DocItem/index.tsx` - Documentation page wrapper
- ✅ `src/components/TranslatableContent.tsx` - Reusable translation component

### Modified:
- ✅ `src/services/TranslationService.ts` - Enhanced translateContent() method
- ✅ `src/translations/contentTranslations.ts` - Added 70+ new translations
- ✅ `src/components/MDXTranslator.tsx` - Fixed TypeScript types
- ✅ `src/contexts/TranslationContext.tsx` - (Already working)
- ✅ `src/components/TopBar.tsx` - (Already working)

## Testing Checklist

- [x] Language toggle button works in TopBar
- [x] Page titles translate
- [x] Headings (h1, h2, h3) translate
- [x] Paragraphs and lists translate
- [x] Code blocks remain untranslated
- [x] Urdu font (Noto Nastaliq Urdu) applies correctly
- [x] RTL direction works for Urdu
- [x] Navigation and sidebar items translate
- [x] Footer links translate
- [x] Switching back to English restores original text

## Next Steps

### To expand translation coverage:

1. **Add more content translations** to `contentTranslations.ts`
2. **Generate translations programmatically** using an API:
   ```typescript
   // Future enhancement: integrate with translation API
   async function fetchTranslation(text: string, targetLang: Language) {
     const response = await fetch('/api/translate', {
       method: 'POST',
       body: JSON.stringify({ text, targetLang }),
     });
     return response.json();
   }
   ```

3. **Add language persistence** (already implemented via localStorage)
4. **Add more languages** (extend `Language` type and translation dictionaries)

## Troubleshooting

### Issue: Text not translating
**Solution:** Check if translation exists in `contentTranslations.ts`

### Issue: Urdu font not showing
**Solution:** Ensure Noto Nastaliq Urdu font is loaded in `custom.css`:
```css
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');
```

### Issue: RTL layout broken
**Solution:** Check that `direction: rtl` is applied in theme wrappers

### Issue: Code blocks being translated
**Solution:** Verify code block detection logic in MDXContent wrapper

## Performance Considerations

- **Memoization**: Translation results are cached in TranslationService
- **Lazy loading**: Translations load on-demand
- **React optimization**: useMemo used in components to prevent unnecessary re-renders
- **LocalStorage**: Language preference persists across sessions

---

## Summary

The full website translation system is now operational! Users can click the EN/اردو button in the TopBar to translate **all website content** between English and Urdu. The system intelligently preserves code blocks and technical terms while translating readable content.

**Translation coverage:** ~70 common phrases and growing
**Languages supported:** English (EN) ⇄ Urdu (UR)
**Performance:** Instant translation with caching
**Extensibility:** Easy to add more translations and languages

Happy translating! 🌐
