# Quick Start Guide

## 🎯 Goal

Autonomous textbook creation pipeline using Context7 MCP (content) and GitHub MCP (publishing).

## ✅ Prerequisites Checklist

- [x] Context7 MCP server connected
- [x] GitHub MCP server connected
- [ ] Node.js installed (v14+)
- [ ] Dependencies installed

## 🚀 5-Minute Setup

### Step 1: Verify MCP Connections

```bash
claude mcp list
```

Expected output:
```
✓ context7: Connected
✓ github: Connected
```

### Step 2: Install Dependencies

```bash
cd agents
npm install
```

### Step 3: Test the Pipeline

```bash
node orchestrator.js
```

This will run a demo pipeline for "Digital Twin in Robotics" with:
- Audio generation enabled
- Urdu translation enabled

## 📋 Creating Your First Chapter

### Option 1: Using the Orchestrator Directly

```javascript
const { PipelineOrchestrator } = require('./orchestrator.js');
const path = require('path');

async function createChapter() {
  const orchestrator = new PipelineOrchestrator(
    path.join(__dirname, 'pipeline-config.yaml')
  );

  await orchestrator.loadConfig();

  const results = await orchestrator.executeFullChapterPipeline(
    'Introduction to ROS2 Navigation',
    {
      audio_enabled: false,      // Set to true for audio
      target_language: null      // Set to 'urdu' for translation
    }
  );

  console.log('Chapter created!', results);
}

createChapter();
```

### Option 2: Command Line

```bash
# Basic chapter creation
node orchestrator.js --topic "SLAM Algorithms in Robotics"

# With audio
node orchestrator.js --topic "Computer Vision for Robots" --audio

# With translation
node orchestrator.js --topic "Path Planning" --language urdu

# Full pipeline
node orchestrator.js --topic "Humanoid Robot Control" --audio --language urdu
```

## 🔄 Available Workflows

### 1. Full Chapter Pipeline (New Content)

```javascript
await orchestrator.executeFullChapterPipeline(topic, options);
```

Creates everything from scratch:
- ✍️ Chapter outline
- 📝 Full content
- ✨ Refinement
- 📚 Citations
- 🎵 Audio (optional)
- 🌍 Translation (optional)
- 🚀 GitHub publish

### 2. Quick Edit (Existing Content)

```javascript
await orchestrator.executeQuickEditPipeline(filePath);
```

For minor updates:
- ✨ Refine content
- 📚 Update citations
- 🚀 GitHub publish

### 3. Translation Only

```javascript
await orchestrator.executeTranslationPipeline(filePath, { target_language: 'urdu' });
```

### 4. Audio Generation Only

```javascript
await orchestrator.executeAudioPipeline(filePath, { voice_options: 'female' });
```

## 📂 Output Structure

After running the pipeline, you'll get:

```
ai-native-book/
├── docs/
│   └── digital-twin-in-robotics.md      # Chapter content
├── mcp/
│   └── digital-twin-in-robotics.json    # MCP format
├── audio/
│   └── digital-twin-in-robotics.mp3     # Audio version (if enabled)
├── translations/
│   └── urdu/
│       └── digital-twin-in-robotics.md  # Translation (if enabled)
└── logs/
    └── pipeline-results.json             # Execution log
```

## 🎨 Customizing the Pipeline

### Add a New Agent

**1. Define in `pipeline-config.yaml`:**

```yaml
agents:
  diagram_generator:
    description: "Generates technical diagrams for chapters"
    uses:
      - context7
    inputs:
      - chapter_content
    outputs:
      - diagram_files
```

**2. Implement in `orchestrator.js`:**

```javascript
async executeDiagramGenerator(chapterData, agent) {
  console.log(`      → Generating diagrams...`);
  // Call Context7 MCP to generate diagrams
  return {
    filePath: path.join(this.config.shared_resources.assets_path, 'diagrams.svg')
  };
}
```

**3. Add to workflow:**

```yaml
workflows:
  full_chapter_pipeline:
    steps:
      # ... existing steps ...
      - name: "Generate Diagrams"
        agent: diagram_generator
        retry: 1
```

### Modify MCP Server Endpoints

Edit `pipeline-config.yaml`:

```yaml
mcp_servers:
  context7:
    type: http
    url: "https://your-custom-endpoint.com/mcp"

  github-mcp:
    type: stdio
    command: npx
    args: ["-y", "@iflow-mcp/server-github"]
```

### Change Error Handling

```yaml
error_handling:
  max_retries: 5              # Increase retries
  retry_delay_seconds: 10     # Longer delay
  fallback_strategy: "fail_fast"  # Stop on first error
```

## 🔍 Monitoring Pipeline Execution

### Real-time Console Output

```
🚀 Starting Full Chapter Pipeline for: "Digital Twin in Robotics"

📍 Step: Plan Chapter
   Agent: content_planner
   → Planning chapter structure for: Digital Twin in Robotics
   ✓ Completed

📍 Step: Write Chapter
   Agent: chapter_writer
   → Writing chapter content...
   ✓ Completed

📍 Step: Generate Audio
   Agent: text_to_voice_agent
   → Generating audio...
   ✓ Completed

✅ Pipeline completed in 45.30s
```

### Check Results

```javascript
// Load results from logs
const results = require('../logs/pipeline-results.json');

console.log(`Topic: ${results.topic}`);
console.log(`Duration: ${results.duration}s`);
console.log(`Steps completed: ${results.steps.filter(s => s.status === 'completed').length}`);
```

## 🐛 Troubleshooting

### MCP Server Not Connected

```bash
# Check connection
claude mcp list

# Restart MCP servers
claude mcp remove github -s local
claude mcp add --transport stdio github \
  --env GITHUB_PERSONAL_ACCESS_TOKEN=your_token \
  -- npx -y @iflow-mcp/server-github
```

### Pipeline Fails at Specific Step

Check the error in results:

```javascript
const failedSteps = results.steps.filter(s => s.status === 'failed');
console.log('Failed steps:', failedSteps);
```

Increase retries for that step in `pipeline-config.yaml`.

### Audio Generation Not Working

Ensure audio is enabled:

```javascript
{
  audio_enabled: true  // Must be true
}
```

### GitHub Publishing Fails

Verify GitHub token:

```bash
# Test token
curl -H "Authorization: token YOUR_TOKEN" https://api.github.com/user
```

## 📊 Example Complete Run

```bash
$ node orchestrator.js

✓ Loaded pipeline config: AI-Native Book

🚀 Starting Full Chapter Pipeline for: "Digital Twin in Robotics"

📍 Step: Plan Chapter
   Agent: content_planner
   → Planning chapter structure for: Digital Twin in Robotics
   ✓ Completed

📍 Step: Write Chapter
   Agent: chapter_writer
   → Writing chapter content...
   ✓ Completed

📍 Step: Refine Chapter
   Agent: chapter_refiner
   → Refining chapter...
   ✓ Completed

📍 Step: Validate Citations
   Agent: citation_checker
   → Validating citations...
   ✓ Completed

📍 Step: Generate MCP JSON
   Agent: mcp_formatter
   → Generating MCP JSON...
   ✓ Completed

📍 Step: Generate Audio
   Agent: text_to_voice_agent
   → Generating audio...
   ✓ Completed

📍 Step: Translate Chapter
   Agent: translator_agent
   → Translating to urdu...
   ✓ Completed

📍 Step: Publish to GitHub
   Agent: github_publisher
   → Publishing to GitHub...
   ✓ Completed

✅ Pipeline completed in 45.30s

💾 Results saved to: ../logs/pipeline-results.json
```

## 🎓 Next Steps

1. **Integrate Real MCP Calls**: Replace TODO comments with actual MCP SDK calls
2. **Add More Agents**: Create specialized agents for your needs
3. **Build UI**: Create a web interface for the pipeline
4. **Add Tests**: Write unit tests for each agent
5. **Deploy**: Set up automated pipelines on GitHub Actions

## 📚 Additional Resources

- [MCP Documentation](https://modelcontextprotocol.io/docs)
- [Context7 API](https://context7.com/docs)
- [GitHub MCP Server](https://github.com/iflow-mcp/server-github)
- [Pipeline Configuration Reference](./README.md)

## 💡 Tips

- Start with simple workflows and gradually add complexity
- Test each agent independently before running full pipeline
- Use conditional steps to save time when features aren't needed
- Monitor token usage with Context7 MCP
- Set up GitHub Actions to automate regular content updates

---

**Ready to create autonomous textbooks! 🚀**
