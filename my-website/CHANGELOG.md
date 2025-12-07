# Changelog - AI-Native Robotics Website

All notable changes to the AI-Native Robotics textbook website.

## [0.1.0] - 2025-12-07

### Added - Complete Website Rebrand

#### Configuration Updates
- **Docusaurus Config** (`docusaurus.config.ts`)
  - Changed site title to "AI-Native Robotics"
  - Updated tagline to "Building the Future of Physical AI - From ROS2 to Digital Twins"
  - Configured GitHub Pages deployment (asmaiqbal01.github.io/ai-native-book/)
  - Added custom organization and project names
  - Enhanced blog configuration with "Robotics Lab Notes" branding
  - Added syntax highlighting for Python, Bash, YAML, XML, C++, CMake
  - Configured Algolia search (requires API keys)
  - Updated edit URLs to point to correct GitHub repository
  - Added last update time and author display

#### Package Configuration
- **package.json**
  - Renamed project to "ai-native-robotics-textbook"
  - Updated version to 0.1.0
  - Added project description
  - Added author information (Asma Iqbal)
  - Changed license to MIT

#### Navigation & Structure
- **Sidebar** (`sidebars.ts`)
  - Created structured navigation with 5 main chapters
  - Added emoji icons for visual hierarchy
  - Organized content by:
    - 📖 Chapter 1: Physical AI (4 topics)
    - 🧠 Chapter 2: ROS2 (5 topics)
    - 🌐 Chapter 3: Digital Twin (4 topics)
    - 🚀 Chapter 4: NVIDIA Isaac (4 topics)
    - 👁️ Chapter 5: VLA (4 topics)
    - 🛠️ Resources (3 topics)

- **Navbar**
  - Renamed "Tutorial" to "Textbook"
  - Added "Get Started" link
  - Renamed "Blog" to "Lab Notes"
  - Updated GitHub link to correct repository

- **Footer**
  - Restructured into three sections: Learn, Resources, Community
  - Added direct links to chapter content
  - Updated GitHub repository links
  - Changed copyright to Asma Iqbal

#### Design & Styling
- **Custom CSS** (`src/css/custom.css`)
  - **Light Mode Theme**: Tech Blue (#1976d2)
  - **Dark Mode Theme**: Cyberpunk Neon with Electric Cyan (#00e5ff)
  - Added robotics-specific accent colors (orange, green, purple, red)
  - Created custom components:
    - `.chapter-badge` (beginner/intermediate/advanced levels)
    - `.learning-objectives` (highlighted key takeaways)
    - `.terminal` (console/command line styling)
    - `.ros-node-diagram` (ROS2 visualization)
    - `.code-example-header` (code block headers)
  - Enhanced typography with gradient hero title
  - Added hover effects for buttons and cards
  - Improved accessibility with focus outlines
  - Added responsive design breakpoints
  - Included print-friendly styles

#### Homepage Updates
- **Homepage** (`src/pages/index.tsx`)
  - Updated page title to "AI-Native Robotics Textbook"
  - Added comprehensive meta description
  - Changed primary CTA to "Start Learning 🤖"
  - Added secondary button "Explore Physical AI"

- **Features Section** (`src/components/HomepageFeatures/index.tsx`)
  - **Feature 1**: Physical AI Fundamentals
    - Description: Embodied intelligence from digital to physical
  - **Feature 2**: Hands-On with ROS2 & Gazebo
    - Description: Build, simulate, deploy with complete code examples
  - **Feature 3**: AI-Powered Robotics
    - Description: NVIDIA Isaac, VLA models, intelligent humanoid robots

#### Documentation
- **README.md**
  - Comprehensive documentation for the website
  - Detailed content structure diagram
  - Technology stack information
  - Instructions for adding new content
  - Contribution guidelines
  - Links to live website and resources

- **CHANGELOG.md** (this file)
  - Complete change history
  - Detailed list of all modifications

### Technical Details

#### File Structure
```
my-website/
├── docusaurus.config.ts    ✅ Updated
├── package.json             ✅ Updated
├── sidebars.ts              ✅ Rewritten
├── README.md                ✅ Rewritten
├── CHANGELOG.md             ✅ New
├── src/
│   ├── pages/
│   │   └── index.tsx        ✅ Updated
│   ├── components/
│   │   └── HomepageFeatures/
│   │       └── index.tsx    ✅ Updated
│   └── css/
│       └── custom.css       ✅ Rewritten
└── docs/                    📝 Ready for content
```

#### Theme Colors

**Light Mode**
- Primary: #1976d2 (Tech Blue)
- Accent Orange: #ff6f00
- Accent Green: #00c853
- Accent Purple: #6a1b9a
- Accent Red: #d32f2f

**Dark Mode**
- Primary: #00e5ff (Electric Cyan)
- Background: #0a1929 (Deep Blue-Black)
- Surface: #1a2332 (Slightly Lighter)
- Accent Orange: #ff9100
- Accent Green: #00e676
- Accent Purple: #ab47bc
- Accent Red: #ff5252

#### Browser Support
- Modern browsers (Chrome, Firefox, Safari, Edge)
- Mobile responsive
- Dark mode automatic detection
- Print-friendly layout

### Next Steps

1. **Content Creation**: Add markdown files for all 25 chapter topics
2. **Assets**: Add robotics-related images and diagrams
3. **Blog Posts**: Create initial "Lab Notes" blog content
4. **Search**: Configure Algolia search with API keys
5. **Deployment**: Set up GitHub Actions for automated deployment
6. **Testing**: Verify all links and navigation paths
7. **SEO**: Add meta tags and optimize for search engines

### Breaking Changes
- None (initial release)

### Dependencies
- Docusaurus: 3.9.2
- React: 19.0.0
- TypeScript: ~5.6.2
- Node.js: >= 20.0

### Known Issues
- Search requires Algolia configuration (API keys needed)
- Some documentation pages are placeholders (content pending)
- Blog section needs initial posts

---

**Author**: Asma Iqbal
**Repository**: https://github.com/AsmaIqbal01/ai-native-book
**Website**: https://asmaiqbal01.github.io/ai-native-book/
