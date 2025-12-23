# Docusaurus i18n Translation Setup Guide

## ✅ What's Been Configured

### 1. Docusaurus i18n Configuration
- **File**: `docusaurus.config.ts`
- **Locales**: English (en) and Urdu (ur)
- **RTL Support**: Enabled for Urdu
- **Locale Routing**: Automatic (`/` for English, `/ur/` for Urdu)

### 2. Translation Files Created
```
i18n/
└── ur/
    ├── code.json                              # UI strings (buttons, labels, etc.)
    └── docusaurus-theme-classic/
        ├── navbar.json                        # Navbar translations
        └── footer.json                        # Footer translations
```

### 3. Integration with Custom Components
- **TranslationContext** now syncs with Docusaurus i18n
- **TopBar** language toggle navigates between `/` and `/ur/`
- **Dark mode** fixed to use `data-theme` attribute
- **RTL layout** automatically applied for Urdu

## 🚀 How to Translate Content

### Step 1: Build the Translation Template
Run this command to generate translation files for your docs:

```bash
npm run write-translations -- --locale ur
```

This creates translation JSON files for all your docs in:
```
i18n/ur/docusaurus-plugin-content-docs/current/
```

### Step 2: Translate MDX Files
For full content translation, copy your MDX files to the Urdu directory:

```bash
# Manual approach: Copy docs to Urdu folder
mkdir -p "i18n/ur/docusaurus-plugin-content-docs/current"
cp -r docs/* "i18n/ur/docusaurus-plugin-content-docs/current/"
```

Then translate the copied MDX files in `i18n/ur/docusaurus-plugin-content-docs/current/`.

### Example Translation Workflow

**English file**: `docs/chapter1/introduction.mdx`
```mdx
---
title: Introduction to Physical AI
description: Learn the basics of Physical AI
---

# Introduction to Physical AI

Physical AI combines artificial intelligence with robotics...
```

**Urdu file**: `i18n/ur/docusaurus-plugin-content-docs/current/chapter1/introduction.mdx`
```mdx
---
title: فزیکل AI کا تعارف
description: فزیکل AI کی بنیادی باتیں سیکھیں
---

# فزیکل AI کا تعارف

فزیکل AI مصنوعی ذہانت کو روبوٹکس کے ساتھ ملاتی ہے...
```

## 🔧 Development & Build Commands

### Start Development Server (English)
```bash
npm start
```
Access at: `http://localhost:3000`

### Start Development Server (Urdu)
```bash
npm run start -- --locale ur
```
Access at: `http://localhost:3000/ur/`

### Build for Production (All Locales)
```bash
npm run build
```

### Build for Specific Locale
```bash
npm run build -- --locale ur
```

## 📁 Directory Structure

```
frontend/Physical AI and Robotics/
├── docs/                                      # English content (default)
│   ├── chapter1/
│   │   ├── introduction.mdx
│   │   └── physical-ai.mdx
│   ├── chapter2/
│   └── resources/
├── i18n/
│   └── ur/                                    # Urdu translations
│       ├── code.json                          # UI translations
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/                       # Urdu docs (copy of docs/)
│       │       ├── chapter1/
│       │       │   ├── introduction.mdx       # Translated to Urdu
│       │       │   └── physical-ai.mdx        # Translated to Urdu
│       │       ├── chapter2/
│       │       └── resources/
│       └── docusaurus-theme-classic/
│           ├── navbar.json
│           └── footer.json
├── src/
│   ├── contexts/
│   │   └── TranslationContext.tsx             # Syncs with Docusaurus i18n
│   └── components/
│       └── TopBar.tsx                         # Language switcher
└── docusaurus.config.ts                       # i18n configuration
```

## 🎯 Translation Priorities

### Already Translated (via code.json)
✅ UI elements (buttons, labels, navigation)
✅ Footer links
✅ Error messages
✅ Search placeholders

### Need Translation (MDX files)
- [ ] Homepage (`src/pages/index.tsx`)
- [ ] Chapter 1: Introduction to Physical AI
- [ ] Chapter 2: ROS2 Fundamentals
- [ ] Chapter 3: Simulation Environments
- [ ] Resources pages
- [ ] Installation guides

## 🔄 How Language Switching Works

1. **User clicks language toggle** in TopBar
2. **TranslationContext** detects locale change
3. **Browser navigates** to `/ur/` (or `/` for English)
4. **Docusaurus serves** translated content from `i18n/ur/`
5. **RTL layout** automatically applies for Urdu
6. **Urdu font** (Noto Nastaliq Urdu) loads globally

## 📝 Translation Best Practices

### 1. Preserve MDX Structure
Keep all MDX components and frontmatter:
```mdx
---
title: Translated Title
sidebar_position: 1
---

import MyComponent from '@site/src/components/MyComponent';

<MyComponent />
```

### 2. Don't Translate Code Blocks
```python
# Keep code in English - it's universal
def hello_world():
    print("Hello, World!")
```

### 3. Translate Comments in Code
```python
# اردو تبصرہ: یہ فنکشن ہیلو ورلڈ پرنٹ کرتا ہے
def hello_world():
    print("Hello, World!")
```

### 4. Handle Links Carefully
- Internal links: `/docs/chapter1/intro` → automatically localized to `/ur/docs/chapter1/intro`
- External links: Keep unchanged

## 🐛 Troubleshooting

### Dark Mode Not Working?
✅ Fixed! Updated `ThemeContext.tsx` to use `data-theme` attribute instead of `dark` class.

### Translation Not Showing?
1. Check file exists in `i18n/ur/docusaurus-plugin-content-docs/current/`
2. Verify frontmatter is correct
3. Clear cache: `npm run clear && npm start -- --locale ur`

### RTL Layout Issues?
- Verify `direction: 'rtl'` in `docusaurus.config.ts`
- Check `custom.css` for RTL-specific styles (lines 421-487)

## 📚 Resources

- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [Urdu Typography Guide](https://fonts.google.com/noto/specimen/Noto+Nastaliq+Urdu)
- [RTL Best Practices](https://rtlstyling.com/)

## 🎉 Next Steps

1. **Run translation template generator**:
   ```bash
   npm run write-translations -- --locale ur
   ```

2. **Copy and translate docs**:
   ```bash
   cp -r docs/* "i18n/ur/docusaurus-plugin-content-docs/current/"
   ```

3. **Start translating MDX files** in `i18n/ur/docusaurus-plugin-content-docs/current/`

4. **Test locally**:
   ```bash
   npm run start -- --locale ur
   ```

5. **Build and deploy**:
   ```bash
   npm run build
   ```

Happy translating! 🚀
