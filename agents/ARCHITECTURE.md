# Pipeline Architecture

## 🏗️ System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                       │
│                     AI-Native Book Pipeline                          │
│                    Autonomous Textbook Creation                      │
│                                                                       │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
        ┌───────────────────┐           ┌───────────────────┐
        │   Context7 MCP    │           │   GitHub MCP      │
        │   (Content Gen)   │           │   (Publishing)    │
        └───────────────────┘           └───────────────────┘
                    │                               │
        ┌───────────┴───────────┐       ┌───────────┴───────────┐
        │                       │       │                       │
        ▼                       ▼       ▼                       ▼
    ┌────────┐            ┌────────┐  ┌────────┐         ┌────────┐
    │ Planner│            │ Writer │  │ Commit │         │  Push  │
    └────────┘            └────────┘  └────────┘         └────────┘
        │                       │          │                   │
        ▼                       ▼          ▼                   ▼
    ┌────────┐            ┌────────┐  ┌─────────────────────────┐
    │Refiner │            │Citation│  │   Pull Request          │
    └────────┘            │Checker │  └─────────────────────────┘
        │                       │
        ▼                       ▼
    ┌────────┐            ┌────────┐
    │  MCP   │            │ Audio  │
    │ Format │            │  Gen   │
    └────────┘            └────────┘
                               │
                               ▼
                          ┌────────┐
                          │Translate│
                          └────────┘
```

## 🔄 Data Flow

### Input → Processing → Output

```
┌──────────────────────────────────────────────────────────────────┐
│ INPUT                                                             │
├──────────────────────────────────────────────────────────────────┤
│ • Topic: "Digital Twin in Robotics"                              │
│ • Options: { audio_enabled: true, target_language: 'urdu' }     │
└──────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│ PROCESSING PIPELINE                                               │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  1. content_planner                                               │
│     └─→ Chapter outline with learning objectives                 │
│                                                                   │
│  2. chapter_writer                                                │
│     └─→ Full markdown chapter (5000+ words)                      │
│                                                                   │
│  3. chapter_refiner                                               │
│     └─→ Improved clarity, formatting, examples                   │
│                                                                   │
│  4. citation_checker                                              │
│     └─→ Validated academic references                            │
│                                                                   │
│  5. mcp_formatter                                                 │
│     └─→ MCP JSON structure                                       │
│                                                                   │
│  6. text_to_voice_agent [CONDITIONAL: audio_enabled=true]        │
│     └─→ MP3 audio file (45 min duration)                         │
│                                                                   │
│  7. translator_agent [CONDITIONAL: target_language='urdu']       │
│     └─→ Urdu translation maintaining formatting                  │
│                                                                   │
│  8. github_publisher                                              │
│     └─→ Commit + Push + PR                                       │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────┐
│ OUTPUT                                                            │
├──────────────────────────────────────────────────────────────────┤
│ • docs/digital-twin-in-robotics.md                               │
│ • mcp/digital-twin-in-robotics.json                              │
│ • audio/digital-twin-in-robotics.mp3                             │
│ • translations/urdu/digital-twin-in-robotics.md                  │
│ • GitHub PR: https://github.com/.../pull/123                     │
└──────────────────────────────────────────────────────────────────┘
```

## 🎯 Agent Responsibilities

### Content Creation Agents

#### 1. content_planner
```yaml
Purpose: Strategic planning
Inputs:
  - Topic string
  - Textbook structure context
Outputs:
  - Chapter outline
  - Learning objectives
  - Section breakdown
MCP Server: Context7
Tools Used:
  - Library documentation lookup
  - Content structure generation
```

#### 2. chapter_writer
```yaml
Purpose: Content generation
Inputs:
  - Chapter plan
Outputs:
  - Full markdown chapter
  - Code examples
  - Exercises
MCP Server: Context7
Tools Used:
  - Code example generation
  - Technical writing assistance
```

#### 3. chapter_refiner
```yaml
Purpose: Quality improvement
Inputs:
  - Chapter draft
Outputs:
  - Refined chapter
  - Improved diagrams
  - Better formatting
MCP Server: Context7
Tools Used:
  - Style checking
  - Clarity enhancement
```

#### 4. citation_checker
```yaml
Purpose: Academic validation
Inputs:
  - Final chapter
Outputs:
  - Validated citations
  - Reference list
  - Bibliography
MCP Server: Context7
Tools Used:
  - Citation validation
  - Reference formatting
```

### Data Transformation Agents

#### 5. mcp_formatter
```yaml
Purpose: Structure conversion
Inputs:
  - Validated chapter
Outputs:
  - MCP JSON file
Format:
  chapter: 1
  title: "..."
  learningObjectives: []
  topics: []
  resources: {}
MCP Server: Context7
```

### Multi-Modal Agents

#### 6. text_to_voice_agent
```yaml
Purpose: Audio generation
Inputs:
  - Final chapter text
  - Voice options (optional)
Outputs:
  - MP3/WAV audio file
  - Duration metadata
Triggers: audio_enabled = true
MCP Server: Context7
Tools Used:
  - Text-to-speech
  - Audio processing
```

#### 7. translator_agent
```yaml
Purpose: Language translation
Inputs:
  - Final chapter
  - Target language
Outputs:
  - Translated markdown
  - Preserved formatting
Triggers: target_language != null
MCP Server: Context7
Tools Used:
  - Translation models
  - Context preservation
```

### Publishing Agent

#### 8. github_publisher
```yaml
Purpose: Version control & deployment
Inputs:
  - All generated files
Outputs:
  - Git commits
  - Pushed changes
  - Pull requests
MCP Server: GitHub
Tools Used:
  - create-commit
  - push-to-remote
  - create-pull-request
Actions:
  1. Stage files
  2. Create commit
  3. Push to branch
  4. Open PR
```

### Orchestrator Agent

#### 9. orchestrator
```yaml
Purpose: Workflow coordination
Responsibilities:
  - Execute pipeline steps in order
  - Handle conditional execution
  - Manage error recovery
  - Track state and progress
  - Aggregate outputs
Uses: Both Context7 and GitHub MCP
Features:
  - Automatic retries
  - State persistence
  - Progress reporting
  - Error logging
```

## 🔐 Error Handling Architecture

```
┌──────────────────────────────────────────────────┐
│              Error Detection                      │
└──────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────┐
│         Is retry count < max_retries?            │
│                                                   │
│  YES ──→ Wait retry_delay_seconds ──→ Retry     │
│                                                   │
│  NO  ──→ Check fallback_strategy                 │
│            │                                      │
│            ├─→ log_and_continue: Log & proceed   │
│            └─→ fail_fast: Stop pipeline          │
└──────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────┐
│          Record in pipeline results              │
│          • Step name                             │
│          • Status: failed                        │
│          • Attempts made                         │
│          • Error message                         │
└──────────────────────────────────────────────────┘
```

## 📦 Component Interaction

### MCP Server Communication

```javascript
// Context7 MCP - Content Generation
{
  server: 'context7',
  method: 'callTool',
  tool: 'generate-content',
  params: {
    prompt: 'Write chapter about...',
    format: 'markdown',
    context: {...}
  }
}

// GitHub MCP - Publishing
{
  server: 'github',
  method: 'callTool',
  tool: 'create-commit',
  params: {
    files: ['docs/chapter.md'],
    message: 'Add chapter: Digital Twin',
    branch: 'main'
  }
}
```

### State Management

```javascript
state = {
  currentStep: 'chapter_writer',
  completedSteps: [
    'content_planner'
  ],
  failedSteps: [],
  outputs: {
    content_planner: {
      filePath: './docs/chapter-plan.md',
      outline: {...}
    }
  }
}
```

## 🚀 Workflow Execution Models

### Sequential Execution (Default)

```
Step 1 → Complete → Step 2 → Complete → Step 3 → ...
```

Benefits:
- Predictable order
- Each step depends on previous
- Easy to debug

### Conditional Execution

```
Step 1 → Complete → Step 2 → Complete
                               │
                ┌──────────────┴──────────────┐
                ▼                             ▼
         audio_enabled?                target_language?
                │                             │
         YES ─→ Audio Gen              YES ─→ Translate
         NO  ─→ Skip                   NO  ─→ Skip
                │                             │
                └──────────────┬──────────────┘
                               ▼
                          Publish to GitHub
```

Benefits:
- Efficient resource usage
- User-controlled features
- Faster for simple cases

### Parallel Execution (Future Enhancement)

```
        ┌─→ Step 2A ─┐
Step 1 ─┼─→ Step 2B ─┼─→ Step 3
        └─→ Step 2C ─┘
```

Benefits:
- Faster execution
- Better resource utilization
- Independent task processing

## 📊 Monitoring & Observability

### Metrics Collected

```yaml
Pipeline Metrics:
  - Total execution time
  - Per-step duration
  - Retry count per step
  - Success/failure rate
  - Token usage (Context7)
  - API calls count

Agent Metrics:
  - Invocation count
  - Average duration
  - Error rate
  - Output size

Resource Metrics:
  - Files created
  - Files modified
  - Storage used
  - MCP server calls
```

### Logging Levels

```
DEBUG:   All tool calls, state changes
INFO:    Pipeline steps, completions
WARNING: Retries, recoverable errors
ERROR:   Failures, unrecoverable issues
```

## 🔄 Retry Strategy

```
Attempt 1: Immediate execution
    ↓ (fails)
Wait 5s
    ↓
Attempt 2: Retry
    ↓ (fails)
Wait 5s
    ↓
Attempt 3: Final retry
    ↓ (fails)

Apply fallback_strategy:
  • log_and_continue → Next step
  • fail_fast → Stop pipeline
```

## 📁 File Organization

```
ai-native-book/
├── agents/                      # Pipeline components
│   ├── pipeline-config.yaml     # Configuration
│   ├── orchestrator.js          # Main orchestrator
│   ├── package.json             # Dependencies
│   ├── README.md                # Full documentation
│   ├── QUICKSTART.md            # Quick start guide
│   └── ARCHITECTURE.md          # This file
│
├── docs/                        # Generated chapters
│   └── [chapter-name].md
│
├── mcp/                         # MCP JSON files
│   ├── index.json               # Book structure
│   └── chapter*.json            # Chapter metadata
│
├── audio/                       # Audio versions
│   └── [chapter-name].mp3
│
├── translations/                # Translated content
│   ├── urdu/
│   ├── spanish/
│   └── ...
│
└── logs/                        # Execution logs
    └── pipeline-results.json
```

## 🎛️ Configuration Schema

### Pipeline Config Structure

```yaml
version: string                  # Config version

project:
  name: string                   # Project name
  goal: string                   # Project objective

mcp_servers:
  [server_name]:
    type: http|stdio|sse         # Transport type
    url: string                  # HTTP/SSE endpoint
    command: string              # stdio command
    args: array                  # Command arguments
    env: object                  # Environment variables

shared_resources:
  [resource_name]: string        # Shared paths

agents:
  [agent_name]:
    description: string
    uses: array                  # MCP servers used
    inputs: array                # Expected inputs
    outputs: array               # Expected outputs

workflows:
  [workflow_name]:
    description: string
    steps:
      - name: string
        agent: string
        conditional: string      # Optional condition
        retry: number            # Retry count

error_handling:
  max_retries: number
  retry_delay_seconds: number
  fallback_strategy: string

monitoring:
  log_level: string
  metrics_enabled: boolean
```

## 🔮 Future Enhancements

1. **Parallel Agent Execution**: Run independent agents concurrently
2. **Agent Swarms**: Multiple agents collaborating on single task
3. **Dynamic Agent Selection**: AI chooses best agent for task
4. **Feedback Loops**: Agents review each other's work
5. **Incremental Updates**: Only regenerate changed sections
6. **A/B Testing**: Generate multiple versions for comparison
7. **Quality Scoring**: Automated quality assessment
8. **User Feedback Integration**: Incorporate reader feedback

---

**Architecture Version**: 1.0
**Last Updated**: 2025-12-07
**Author**: Asma Iqbal
