# MCP (Model Context Protocol) Structure

This directory contains structured JSON files that organize the AI-Native Robotics textbook content for use with AI assistants, tooling, and documentation generators.

## Overview

The MCP structure provides machine-readable organization of:
- Chapter/module metadata and learning objectives
- Topic breakdowns with content paths
- Code examples and exercises
- Prerequisites and technical requirements
- Cross-references between chapters

## Files

- **`index.json`** - Master index with book metadata and chapter listing
- **`chapter1.json`** - Introduction to Physical AI
- **`chapter2.json`** - The Robotic Nervous System (ROS 2) / Module 1
- **`chapter3.json`** - The Digital Twin (Gazebo & Unity) / Module 2
- **`chapter4.json`** - The AI-Robot Brain (NVIDIA Isaac) / Module 3
- **`chapter5.json`** - Vision-Language-Action (VLA) / Module 4

## Structure

Each chapter JSON file contains:

```json
{
  "chapter": 1,
  "title": "Chapter Title",
  "slug": "folder-name",
  "status": "specification_complete | implementation_in_progress",
  "metadata": {
    "version": "0.1.0",
    "lastUpdated": "2025-12-07",
    "estimatedReadingTime": "X minutes",
    "handsOnTime": "X hours"
  },
  "learningObjectives": ["..."],
  "topics": [
    {
      "id": "1.1",
      "title": "Topic Title",
      "contentPath": "path/to/content.md",
      "concepts": ["..."],
      "examples": ["path/to/example.py"],
      "exercises": ["Exercise description"]
    }
  ],
  "prerequisites": ["..."],
  "technicalRequirements": {...},
  "resources": {...}
}
```

## Usage

### For AI Assistants (Claude MCP)

Load chapter files to understand textbook structure:

```python
import json

# Load master index
with open('mcp/index.json') as f:
    book = json.load(f)

# Load specific chapter
with open('mcp/chapter2.json') as f:
    ros2_module = json.load(f)

# Access topics
for topic in ros2_module['topics']:
    print(f"{topic['id']}: {topic['title']}")
```

### For Documentation Generators

Generate navigation, table of contents, or interactive learning paths:

```javascript
// Example: Generate chapter navigation
const index = require('./mcp/index.json');
const chapters = index.chapters.map(ch => ({
  title: ch.title,
  file: ch.file,
  status: ch.status
}));
```

### For Custom Tooling

Build learning management systems, progress trackers, or content validators:

```python
# Example: Check prerequisites
def get_chapter_prerequisites(chapter_num):
    with open(f'mcp/chapter{chapter_num}.json') as f:
        chapter = json.load(f)
    return chapter.get('prerequisites', [])
```

## Integration with GitHub MCP Server

To use this structure with Claude Code's GitHub MCP integration:

1. **Install GitHub MCP server** (if not already installed)
2. **Configure Claude Desktop** to use GitHub MCP
3. **Access chapter content** via MCP tools

See [GitHub MCP Documentation](https://github.com/modelcontextprotocol/servers/tree/main/src/github) for setup instructions.

## Maintenance

When updating textbook content:

1. Update the corresponding chapter JSON file
2. Update `index.json` if chapter metadata changes
3. Increment version numbers as appropriate
4. Update `lastUpdated` timestamp

## Schema

All JSON files follow a consistent schema:
- **Required fields**: `chapter`, `title`, `slug`, `status`, `metadata`, `learningObjectives`, `topics`, `resources`
- **Optional fields**: `module`, `prerequisites`, `technicalRequirements`, `previousChapter`, `nextChapter`

## Version

- **MCP Version**: 1.0
- **Book Version**: 0.1.0
- **Last Updated**: 2025-12-07

## Related Documentation

- [Main README](../README.md) - Project overview
- [SETUP.md](../SETUP.md) - Installation instructions
- [Specification files](../specs/) - Detailed feature specs
- [Examples](../examples/) - Code examples referenced in chapters
