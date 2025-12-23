# Quickstart: RAG Chatbot Development

**Feature**: 001-rag-chatbot-agent-system
**Date**: 2025-12-20
**Purpose**: Get developers up and running with local development environment

---

## Prerequisites

### Backend
- Python 3.11+
- pip or uv package manager
- Qdrant instance (cloud or local Docker)
- OpenAI API key

### Frontend
- Node.js 20+
- npm or yarn

---

## Backend Setup

### 1. Environment Configuration

```bash
cd RAG-backend

# Copy environment template
cp .env.example .env

# Edit .env with your credentials
```

**Required Environment Variables**:
```bash
# .env
OPENAI_API_KEY=sk-...
COHERE_API_KEY=...  # For embeddings

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=documentation

# Optional
LOG_LEVEL=INFO
ENVIRONMENT=development
```

### 2. Install Dependencies

```bash
# Using pip
pip install -r requirements.txt

# Or using uv (faster)
uv pip install -r requirements.txt
```

### 3. Run Development Server

```bash
# Start FastAPI with hot reload
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify Backend**:
```bash
# Health check
curl http://localhost:8000/health

# Expected response:
{
  "status": "healthy",
  "timestamp": "2025-12-20T10:30:00Z",
  "qdrant_status": "connected",
  "llm_status": "available"
}
```

---

## Frontend Setup

### 1. Install Dependencies

```bash
cd "frontend/Physical AI and Robotics"

# Install packages
npm install

# Add Zustand for state management
npm install zustand

# Add testing dependencies (from research.md)
npm install --save-dev @testing-library/react @testing-library/jest-dom @testing-library/user-event jest jest-environment-jsdom @types/jest
```

### 2. Configure API Base URL

```typescript
// src/services/chatApi.ts (create if not exists)
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-space.hf.space'  // Update with your Hugging Face Space URL
  : 'http://localhost:8000'

export const chatApi = {
  async query(question: string) {
    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question })
    })

    if (!response.ok) {
      const error = await response.json()
      throw new Error(error.message)
    }

    return response.json()
  },

  async getHealth() {
    const response = await fetch(`${API_BASE_URL}/health`)
    return response.json()
  }
}
```

### 3. Run Development Server

```bash
npm start
```

Frontend will be available at `http://localhost:3000`

---

## Development Workflow

### Backend Development

#### Running Tests

```bash
cd RAG-backend

# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test file
pytest tests/unit/test_query_router.py

# Run with verbose output
pytest -v
```

#### Testing Individual Agents

```python
# tests/unit/test_query_router.py
from app.agents.query_router import QueryRouterAgent

def test_greeting_detection():
    router = QueryRouterAgent()
    result = router.route("Hello!")

    assert result.is_greeting is True
    assert result.should_retrieve is False
```

#### Manual API Testing

```bash
# Test query endpoint
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'

# Expected response:
{
  "answer": "ROS 2 is a robotics middleware framework...",
  "sources": ["intro/what-is-ros2"],
  "confidence": 0.95,
  "processing_time_ms": 1245.3
}
```

### Frontend Development

#### Running Tests

```bash
cd "frontend/Physical AI and Robotics"

# Run tests
npm test

# Run with coverage
npm test -- --coverage

# Watch mode
npm test -- --watch
```

#### Testing Chat Widget

```typescript
// tests/components/FloatingChatWidget.test.tsx
import { render, screen, fireEvent } from '@testing-library/react'
import { FloatingChatWidget } from '@site/src/components/FloatingChatWidget'

test('clears messages on chat close', () => {
  render(<FloatingChatWidget />)

  // Simulate adding messages
  // ... test logic

  // Close chat
  const closeButton = screen.getByLabelText('Close chat')
  fireEvent.click(closeButton)

  // Verify messages cleared (FR-016)
  expect(screen.queryByRole('article')).not.toBeInTheDocument()
})
```

---

## Common Development Tasks

### Task 1: Add New Agent

```python
# 1. Create agent file
# app/agents/new_agent.py

from pydantic import BaseModel
from typing import Optional

class NewAgentResponse(BaseModel):
    success: bool
    result: Optional[str] = None

class NewAgent:
    def __init__(self):
        pass

    async def process(self, input_data: str) -> NewAgentResponse:
        # Agent logic here
        return NewAgentResponse(success=True, result="processed")

# 2. Add unit tests
# tests/unit/test_new_agent.py

import pytest
from app.agents.new_agent import NewAgent

@pytest.mark.asyncio
async def test_new_agent():
    agent = NewAgent()
    result = await agent.process("test input")
    assert result.success is True

# 3. Integrate into RAGChatAgent
# app/agents/rag_chat_agent.py

from app.agents.new_agent import NewAgent

class RAGChatAgent:
    def __init__(self):
        self.new_agent = NewAgent()
        # ...
```

### Task 2: Add New Frontend Component

```typescript
// 1. Create component
// src/components/NewComponent.tsx

import React from 'react'

interface NewComponentProps {
  data: string
}

export function NewComponent({ data }: NewComponentProps) {
  return <div>{data}</div>
}

// 2. Add tests
// tests/components/NewComponent.test.tsx

import { render, screen } from '@testing-library/react'
import { NewComponent } from '@site/src/components/NewComponent'

test('renders data correctly', () => {
  render(<NewComponent data="test" />)
  expect(screen.getByText('test')).toBeInTheDocument()
})

// 3. Integrate into ChatWindow
// src/components/ChatWindow.tsx

import { NewComponent } from './NewComponent'

export function ChatWindow() {
  return (
    <div>
      <NewComponent data="example" />
    </div>
  )
}
```

### Task 3: Update System Prompt

```bash
# 1. Edit prompt file
# app/utils/prompts/answer_synthesis.txt

You are a documentation assistant for AI-Native Robotics.

CRITICAL RULES:
- ONLY use information from the provided context
- If the answer is not in the context, say: "This information is not available in the current documentation."
- Never speculate or use external knowledge
- Always cite sources

Context: {retrieved_context}

Question: {user_question}

Answer:

# 2. No code changes needed - prompt loaded at runtime
# 3. Test with updated prompt
pytest tests/integration/test_answer_synthesis.py -v
```

---

## Debugging Tips

### Backend Debugging

**Enable Detailed Logging**:
```python
# app/config.py
import structlog

structlog.configure(
    wrapper_class=structlog.make_filtering_bound_logger(logging.DEBUG)
)
```

**Debug Agent Flow**:
```python
# Add logging to agents
import structlog
logger = structlog.get_logger()

class QueryRouterAgent:
    async def route(self, query: str):
        logger.info("query_router.route", query=query)
        # ... processing
        logger.info("query_router.result", result=result)
        return result
```

**Inspect Qdrant Queries**:
```bash
# Enable Qdrant debug logging
export QDRANT_LOG_LEVEL=debug
```

### Frontend Debugging

**Debug Zustand State**:
```typescript
// src/store/chatStore.ts
import { devtools } from 'zustand/middleware'

export const useChatStore = create<ChatState>()(
  devtools(
    (set) => ({
      // ... state
    }),
    { name: 'ChatStore' }
  )
)

// Use Redux DevTools browser extension to inspect state changes
```

**Debug API Calls**:
```typescript
// src/services/chatApi.ts
export const chatApi = {
  async query(question: string) {
    console.log('[chatApi] Sending query:', question)

    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question })
    })

    console.log('[chatApi] Response status:', response.status)

    const data = await response.json()
    console.log('[chatApi] Response data:', data)

    return data
  }
}
```

---

## Performance Monitoring

### Backend Metrics

**Track Response Times**:
```python
# app/api/query.py
import time

@router.post("/query")
async def process_query(request: QueryRequest):
    start = time.time()

    # Process query
    result = await agent.process(request.question)

    elapsed_ms = (time.time() - start) * 1000
    logger.info("query.completed", time_ms=elapsed_ms)

    return {**result, "processing_time_ms": elapsed_ms}
```

**Monitor Success Criteria**:
- SC-001: <3 seconds for 95% of queries
- SC-004: Graceful Qdrant errors
- SC-008: 50 concurrent queries

### Frontend Metrics

**Track User Interactions**:
```typescript
// src/store/chatStore.ts
addMessage: (msg) => {
  const start = Date.now()

  set((state) => {
    const elapsed = Date.now() - start
    console.log(`[Metrics] addMessage took ${elapsed}ms`)

    return { messages: [...state.messages, msg] }
  })
}
```

---

## Next Steps

After completing local setup:

1. **Read** [data-model.md](./data-model.md) - Understand core entities
2. **Review** [contracts/openapi.yaml](./contracts/openapi.yaml) - API specification
3. **Check** [research.md](./research.md) - Technical decisions
4. **Implement** using [tasks.md](./tasks.md) (generated via `/sp.tasks` command)

---

## Troubleshooting

### Qdrant Connection Fails

```bash
# Check Qdrant URL
curl $QDRANT_URL/collections

# Verify API key
curl -H "api-key: $QDRANT_API_KEY" $QDRANT_URL/collections
```

### OpenAI API Errors

```bash
# Test API key
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### CORS Errors

```python
# app/main.py - Ensure localhost allowed
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Add this
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Chat Widget Not Visible

```css
/* Ensure high z-index */
.floating-chat-widget {
  position: fixed !important;
  bottom: 20px;
  left: 20px;
  z-index: 9999 !important;
}
```

---

**Ready to implement?** Run `/sp.tasks` to generate implementation tasks!
