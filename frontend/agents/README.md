# AI-Native Book Pipeline

An autonomous multi-agent system for creating, refining, translating, and publishing textbook chapters using MCP servers.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Pipeline Orchestrator                     │
└─────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
     ┌────────────────┐              ┌────────────────┐
     │  Context7 MCP  │              │  GitHub MCP    │
     │  (Content)     │              │  (Publishing)  │
     └────────────────┘              └────────────────┘
              │                               │
    ┌─────────┴─────────┐          ┌─────────┴─────────┐
    ▼         ▼         ▼          ▼         ▼         ▼
 Planner  Writer  Refiner      Commit    Push     PR
```

## 📦 MCP Servers

### Connected Servers

1. **Context7** - Content generation and editing
   - Type: HTTP
   - URL: https://mcp.context7.com/mcp
   - Status: ✓ Connected

2. **GitHub** - Repository management
   - Type: stdio
   - Command: `npx -y @iflow-mcp/server-github`
   - Status: ✓ Connected

## 🤖 Agents

### Content Pipeline

1. **content_planner**
   - Creates chapter outlines and learning objectives
   - Uses: Context7 MCP

2. **chapter_writer**
   - Writes full chapter content with examples
   - Uses: Context7 MCP

3. **chapter_refiner**
   - Improves clarity, formatting, and flow
   - Uses: Context7 MCP

4. **citation_checker**
   - Validates and adds academic citations
   - Uses: Context7 MCP

5. **mcp_formatter**
   - Converts chapters to MCP JSON format
   - Uses: Context7 MCP

### Multi-Modal Agents

6. **text_to_voice_agent**
   - Generates audio versions of chapters
   - Uses: Context7 MCP
   - Optional: Activated with `audio_enabled: true`

7. **translator_agent**
   - Translates chapters to other languages
   - Uses: Context7 MCP
   - Optional: Activated with `target_language: <lang>`

### Publishing Agent

8. **github_publisher**
   - Commits and pushes all outputs to GitHub
   - Uses: GitHub MCP

### Orchestrator

9. **orchestrator**
   - Coordinates all agents
   - Manages workflow state and error recovery
   - Uses: Both Context7 and GitHub MCP

## 🔄 Workflows

### 1. Full Chapter Pipeline
Complete end-to-end chapter creation:

```yaml
Steps:
  1. Plan Chapter → content_planner
  2. Write Chapter → chapter_writer
  3. Refine Chapter → chapter_refiner
  4. Validate Citations → citation_checker
  5. Generate MCP JSON → mcp_formatter
  6. Generate Audio (optional) → text_to_voice_agent
  7. Translate (optional) → translator_agent
  8. Publish to GitHub → github_publisher
```

### 2. Quick Edit Pipeline
For editing existing chapters:

```yaml
Steps:
  1. Refine Chapter → chapter_refiner
  2. Validate Citations → citation_checker
  3. Publish to GitHub → github_publisher
```

### 3. Translation Only Pipeline
Add new language version:

```yaml
Steps:
  1. Translate Chapter → translator_agent
  2. Publish to GitHub → github_publisher
```

### 4. Audio Generation Pipeline
Generate audio for existing chapter:

```yaml
Steps:
  1. Generate Audio → text_to_voice_agent
  2. Publish to GitHub → github_publisher
```

## 🚀 Usage

### Prerequisites

```bash
# Install dependencies
npm install yaml

# Ensure MCP servers are connected
claude mcp list
```

### Running the Pipeline

#### Full Chapter Creation

```javascript
const { PipelineOrchestrator } = require('./orchestrator.js');

const orchestrator = new PipelineOrchestrator('./pipeline-config.yaml');
await orchestrator.loadConfig();

const results = await orchestrator.executeFullChapterPipeline(
  'Introduction to ROS2',
  {
    audio_enabled: true,
    target_language: 'urdu'
  }
);
```

#### Quick Edit

```javascript
const results = await orchestrator.executeQuickEditPipeline(
  './docs/introduction-to-ros2.md'
);
```

#### Translation Only

```javascript
const results = await orchestrator.executeTranslationPipeline(
  './docs/introduction-to-ros2.md',
  { target_language: 'spanish' }
);
```

### Command Line Usage

```bash
# Full chapter pipeline
node agents/orchestrator.js

# Custom topic
node agents/orchestrator.js --topic "Digital Twin in Robotics" \
  --audio --language urdu
```

## 📁 Directory Structure

```
ai-native-book/
├── agents/
│   ├── pipeline-config.yaml    # Pipeline configuration
│   ├── orchestrator.js          # Main orchestrator
│   └── README.md                # This file
├── docs/                        # Generated chapters
├── mcp/                         # MCP JSON files
├── audio/                       # Generated audio files
├── translations/                # Translated versions
│   ├── urdu/
│   ├── spanish/
│   └── ...
└── logs/                        # Pipeline execution logs
```

## ⚙️ Configuration

Edit `pipeline-config.yaml` to customize:

- MCP server endpoints
- Agent behavior and tools
- Workflow steps and retry logic
- Error handling strategies
- Output directories

## 🔧 Error Handling

The pipeline includes robust error handling:

- **Automatic Retries**: Each step can retry up to 3 times
- **Retry Delay**: 5-second delay between retries
- **Fallback Strategy**: `log_and_continue` or `fail_fast`
- **State Management**: Tracks completed and failed steps

## 📊 Monitoring

The orchestrator logs:

- ✓ Completed steps
- ✗ Failed steps with error messages
- 🔄 Retry attempts
- ⏭️ Skipped conditional steps
- 📍 Current step execution

## 🎯 Next Steps

To integrate with actual MCP servers:

1. **Context7 Integration**
   ```javascript
   // In orchestrator.js, replace TODO comments with:
   const response = await mcpClient.callTool('context7', 'generate-content', {
     prompt: topic,
     format: 'markdown'
   });
   ```

2. **GitHub Integration**
   ```javascript
   // Replace GitHub publisher TODO with:
   await mcpClient.callTool('github', 'create-commit', {
     files: filesToCommit,
     message: `Add chapter: ${topic}`,
     branch: 'main'
   });
   ```

3. **Add MCP Client Library**
   ```bash
   npm install @modelcontextprotocol/sdk
   ```

## 📝 Example Output

```json
{
  "topic": "Digital Twin in Robotics",
  "startTime": "2025-12-07T14:30:00.000Z",
  "steps": [
    { "name": "Plan Chapter", "status": "completed", "agent": "content_planner" },
    { "name": "Write Chapter", "status": "completed", "agent": "chapter_writer" },
    { "name": "Refine Chapter", "status": "completed", "agent": "chapter_refiner" },
    { "name": "Validate Citations", "status": "completed", "agent": "citation_checker" },
    { "name": "Generate Audio", "status": "completed", "agent": "text_to_voice_agent" },
    { "name": "Translate Chapter", "status": "completed", "agent": "translator_agent" },
    { "name": "Publish to GitHub", "status": "completed", "agent": "github_publisher" }
  ],
  "outputs": {
    "github_publisher": {
      "committed": 4,
      "branch": "main",
      "url": "https://github.com/AsmaIqbal01/ai-native-book"
    }
  },
  "duration": 45.3
}
```

## 🤝 Contributing

To add new agents:

1. Define agent in `pipeline-config.yaml`
2. Implement execution method in `orchestrator.js`
3. Add to appropriate workflow
4. Update this README

## 📄 License

MIT
