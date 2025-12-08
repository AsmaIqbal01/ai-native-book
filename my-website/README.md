# AI-Native Robotics Textbook Website

An interactive educational website built with Docusaurus for the AI-Native Robotics textbook.

## 📚 About

This website hosts the complete **AI-Native Robotics** textbook, covering:

- **Chapter 1**: Introduction to Physical AI
- **Chapter 2**: The Robotic Nervous System (ROS2)
- **Chapter 3**: Digital Twin (Gazebo & Unity)
- **Chapter 4**: The AI-Robot Brain (NVIDIA Isaac™)
- **Chapter 5**: Vision-Language-Action (VLA)

## 🚀 Quick Start

### Prerequisites

- Node.js >= 20.0
- npm or yarn

### Installation

```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without needing to restart the server.

### Build

```bash
npm run build
```

This command generates static content into the `build` directory that can be served using any static hosting service.

### Deployment

#### Using GitHub Pages

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub Pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## 📖 Content Structure

```
docs/
├── intro.md                    # Getting Started
├── chapter1/                   # Physical AI
│   ├── physical-ai.md
│   ├── digital-to-physical.md
│   ├── humanoid-landscape.md
│   └── sensor-systems.md
├── chapter2/                   # ROS2
│   ├── ros2-intro.md
│   ├── ros2-architecture.md
│   ├── ros2-nodes-topics.md
│   ├── ros2-services-actions.md
│   └── ros2-workspace.md
├── chapter3/                   # Digital Twin
│   ├── digital-twin-intro.md
│   ├── gazebo-basics.md
│   ├── urdf-models.md
│   └── unity-simulation.md
├── chapter4/                   # NVIDIA Isaac
│   ├── isaac-intro.md
│   ├── isaac-sim.md
│   ├── isaac-ros.md
│   └── navigation-perception.md
├── chapter5/                   # VLA
│   ├── vla-intro.md
│   ├── vision-models.md
│   ├── language-models.md
│   └── action-integration.md
└── resources/                  # Additional Resources
    ├── setup-guide.md
    ├── troubleshooting.md
    └── references.md
```

## 🎨 Customization

### Theme

The website uses a custom robotics-themed color palette:

- **Light Mode**: Tech Blue theme
- **Dark Mode**: Cyberpunk Neon theme with electric cyan accents

### Navigation

- **Textbook**: Main documentation organized by chapters
- **Get Started**: Quick introduction
- **Lab Notes**: Blog posts and updates
- **GitHub**: Source code repository

## 🛠️ Technology Stack

- **Framework**: [Docusaurus 3.9.2](https://docusaurus.io/)
- **UI Library**: React 19
- **Language**: TypeScript
- **CSS**: Custom CSS with Infima framework
- **Deployment**: GitHub Pages
- **AI Agents**: Claude Agent SDK (for content management automation)

## 📝 Adding Content

### Creating a New Page

1. Add a new markdown file in the appropriate chapter directory:
   ```bash
   touch docs/chapter1/new-topic.md
   ```

2. Add frontmatter to the file:
   ```markdown
   ---
   sidebar_position: 5
   title: New Topic Title
   ---

   # New Topic

   Your content here...
   ```

3. The page will automatically appear in the sidebar based on its position.

### Adding Code Examples

Use syntax highlighting for code blocks:

````markdown
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```
````

### Adding Admonitions

Use special callout boxes for important information:

```markdown
:::note
This is a note
:::

:::tip
This is a helpful tip
:::

:::warning
This is a warning
:::

:::danger
This is dangerous
:::

:::info
Additional information
:::
```

### Adding Images

```markdown
![Robot Diagram](./images/robot-diagram.png)
```

## 🎓 Features

- **📱 Responsive Design**: Works on desktop, tablet, and mobile
- **🌓 Dark Mode**: Automatic dark/light theme switching
- **🔍 Search**: Full-text search (requires Algolia configuration)
- **📊 Syntax Highlighting**: Support for Python, C++, YAML, XML, Bash
- **🔖 Version Control**: Edit links to GitHub for every page
- **🏷️ Tags**: Organize content with tags
- **📖 Reading Time**: Estimated reading time for each post
- **🎯 Learning Objectives**: Highlight key takeaways
- **💻 Code Tabs**: Multiple language examples

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

MIT License - see the [LICENSE](LICENSE) file for details.

## 👥 Author

**Asma Iqbal**

- GitHub: [@AsmaIqbal01](https://github.com/AsmaIqbal01)

## 🔗 Links

- **Live Website**: https://asmaiqbal01.github.io/ai-native-book/
- **GitHub Repository**: https://github.com/AsmaIqbal01/ai-native-book
- **Issue Tracker**: https://github.com/AsmaIqbal01/ai-native-book/issues
- **Discussions**: https://github.com/AsmaIqbal01/ai-native-book/discussions

## 📚 Additional Resources

- [Docusaurus Documentation](https://docusaurus.io/)
- [ROS2 Documentation](https://docs.ros.org/)
- [NVIDIA Isaac Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [Gazebo Documentation](https://gazebosim.org/)

---

Built with ❤️ using [Docusaurus](https://docusaurus.io/)
