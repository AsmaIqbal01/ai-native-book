# Research: RAG Chatbot Agent System

**Feature**: 001-rag-chatbot-agent-system
**Date**: 2025-12-20
**Purpose**: Resolve technical unknowns identified in Technical Context

## Research Questions

From Technical Context, the following items need clarification:

1. **Frontend State Management**: Zustand vs React Context
2. **Frontend Testing Framework**: Jest vs Vitest for Docusaurus
3. **Agent Testing Strategy**: Unit vs Integration approach
4. **Qdrant Mocking Strategy**: Test data management

## Q1: Frontend State Management - Zustand vs React Context

### Decision: Use Zustand

**Rationale**:
- **Simpler API**: Zustand requires less boilerplate than React Context
- **Better Performance**: No Context Provider re-render cascades
- **Built-in DevTools**: Easy debugging of state changes
- **Perfect for Ephemeral State**: Ideal for our use case (no persistence needed)
- **TypeScript Support**: First-class TypeScript integration

**Implementation Approach**:
```typescript
// store/chatStore.ts
import { create } from 'zustand'

interface Message {
  id: string
  role: 'user' | 'assistant'
  content: string
  timestamp: Date
  sources?: string[]
}

interface ChatState {
  isChatOpen: boolean
  messages: Message[]
  isLoading: boolean

  // Actions
  toggleChat: () => void
  closeChat: () => void  // FR-016: Clears messages
  addMessage: (message: Omit<Message, 'id' | 'timestamp'>) => void
  clearMessages: () => void
  setLoading: (loading: boolean) => void
}

export const useChatStore = create<ChatState>((set) => ({
  isChatOpen: false,
  messages: [],
  isLoading: false,

  toggleChat: () => set((state) => ({ isChatOpen: !state.isChatOpen })),
  closeChat: () => set({ isChatOpen: false, messages: [] }),  // Clear on close
  addMessage: (msg) => set((state) => ({
    messages: [...state.messages, {
      ...msg,
      id: crypto.randomUUID(),
      timestamp: new Date()
    }]
  })),
  clearMessages: () => set({ messages: [] }),
  setLoading: (loading) => set({ isLoading: loading })
}))
```

**Alternatives Considered**:
- **React Context**: More boilerplate, potential performance issues with frequent updates
- **Redux Toolkit**: Overkill for this simple state (ephemeral, no persistence, no complex logic)
- **Jotai/Recoil**: Less mature, smaller ecosystems

**Dependencies Required**:
```json
{
  "zustand": "^4.4.7"
}
```

---

## Q2: Frontend Testing Framework for Docusaurus

### Decision: Use @docusaurus/theme-classic Testing Setup (Built-in Jest)

**Rationale**:
- **Docusaurus Native**: Docusaurus uses Jest internally, already configured
- **React Testing Library**: Standard for React component testing
- **No Additional Config**: Works out of the box with Docusaurus structure
- **Community Support**: Well-documented testing patterns for Docusaurus plugins

**Implementation Approach**:
```json
// package.json additions
{
  "scripts": {
    "test": "jest",
    "test:watch": "jest --watch",
    "test:coverage": "jest --coverage"
  },
  "devDependencies": {
    "@testing-library/react": "^14.1.2",
    "@testing-library/jest-dom": "^6.1.5",
    "@testing-library/user-event": "^14.5.1",
    "jest": "^29.7.0",
    "jest-environment-jsdom": "^29.7.0",
    "@types/jest": "^29.5.11"
  }
}
```

```javascript
// jest.config.js
module.exports = {
  preset: '@docusaurus/theme-classic',
  testEnvironment: 'jsdom',
  moduleNameMapper: {
    '\\.(css|less|scss|sass)$': 'identity-obj-proxy',
  },
  setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],
  testPathIgnorePatterns: ['/node_modules/', '/.docusaurus/'],
}
```

**Test Example**:
```typescript
// tests/components/FloatingChatWidget.test.tsx
import { render, screen, fireEvent } from '@testing-library/react'
import { FloatingChatWidget } from '@site/src/components/FloatingChatWidget'
import { useChatStore } from '@site/src/store/chatStore'

describe('FloatingChatWidget', () => {
  it('clears messages when closed', () => {
    const { closeChat } = useChatStore.getState()
    render(<FloatingChatWidget />)

    // Add message
    useChatStore.getState().addMessage({ role: 'user', content: 'Test' })
    expect(useChatStore.getState().messages).toHaveLength(1)

    // Close widget
    closeChat()

    // FR-016: Messages should be cleared
    expect(useChatStore.getState().messages).toHaveLength(0)
  })
})
```

**Alternatives Considered**:
- **Vitest**: Faster, but less mature Docusaurus integration
- **Playwright Component Testing**: Too heavy for unit/component tests

---

## Q3: Agent Testing Strategy

### Decision: Hybrid Approach - Unit Tests for Sub-Agents, Integration Tests for Orchestration

**Rationale**:
- **Modularity**: Each sub-agent has clear, testable responsibilities
- **Mocking Boundaries**: External services (Qdrant, LLM) mocked at service layer
- **Fast Feedback**: Unit tests run quickly without external dependencies
- **Real Integration**: Integration tests validate agent coordination

**Testing Layers**:

#### Layer 1: Unit Tests (Sub-Agents)

Test individual agent logic with mocked services:

```python
# tests/unit/test_query_router.py
import pytest
from app.agents.query_router import QueryRouterAgent

@pytest.fixture
def query_router():
    return QueryRouterAgent()

def test_detects_greeting(query_router):
    result = query_router.route("Hello!")
    assert result.is_greeting is True
    assert result.should_retrieve is False
    assert "friendly" in result.response.lower()

def test_detects_empty_query(query_router):
    result = query_router.route("")
    assert result.is_valid is False
    assert result.error_message == "Please ask a question"

def test_normalizes_query(query_router):
    result = query_router.route("  What IS   ROS2?  ")
    assert result.normalized_query == "what is ros2?"
    assert result.should_retrieve is True
```

#### Layer 2: Integration Tests (Agent Orchestration)

Test RAGChatAgent with mocked external services:

```python
# tests/integration/test_rag_chat_agent.py
import pytest
from unittest.mock import AsyncMock
from app.agents.rag_chat_agent import RAGChatAgent
from app.services.qdrant import QdrantService
from app.services.llm import LLMService

@pytest.fixture
def mock_qdrant(mocker):
    mock = mocker.patch('app.services.qdrant.QdrantService')
    mock.search.return_value = [
        {"text": "ROS2 is a robotics framework", "score": 0.95, "chapter": "intro"}
    ]
    return mock

@pytest.fixture
def mock_llm(mocker):
    mock = mocker.patch('app.services.llm.LLMService')
    mock.generate.return_value = "ROS2 is a robotics middleware framework..."
    return mock

@pytest.mark.asyncio
async def test_full_query_flow(mock_qdrant, mock_llm):
    agent = RAGChatAgent()
    response = await agent.process_query("What is ROS2?")

    assert response.success is True
    assert response.answer is not None
    assert len(response.sources) > 0
    mock_qdrant.search.assert_called_once()
    mock_llm.generate.assert_called_once()
```

#### Layer 3: Contract Tests (API Endpoints)

Test FastAPI endpoints with full agent stack but mocked Qdrant:

```python
# tests/integration/test_query_endpoint.py
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_query_endpoint_success(mock_qdrant):
    response = client.post("/query", json={
        "question": "What is Physical AI?"
    })

    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert isinstance(data["sources"], list)
```

**Tools**:
- `pytest` (test runner)
- `pytest-asyncio` (async test support)
- `pytest-mock` (mocking)
- `pytest-cov` (coverage)

---

## Q4: Qdrant Mocking Strategy

### Decision: Pytest Fixture-Based Mocking with Sample Document Store

**Rationale**:
- **Deterministic**: Tests always use same data, reproducible results
- **Fast**: No network calls, instant responses
- **Flexible**: Easy to test edge cases (empty results, low scores, errors)
- **Realistic**: Sample documents mimic real chunked content

**Implementation**:

```python
# tests/fixtures/mock_qdrant.py
import pytest
from typing import List, Dict
from unittest.mock import AsyncMock

class MockQdrantClient:
    """Mock Qdrant client for testing"""

    def __init__(self):
        self.sample_docs = [
            {
                "id": "doc1-chunk1",
                "text": "ROS 2 is a robotics middleware framework designed for building robot applications.",
                "metadata": {"chapter": "intro", "section": "what-is-ros2"}
            },
            {
                "id": "doc1-chunk2",
                "text": "Physical AI combines perception, reasoning, and action in physical environments.",
                "metadata": {"chapter": "physical-ai", "section": "definition"}
            },
            {
                "id": "doc2-chunk1",
                "text": "Gazebo is a 3D robot simulator used for testing ROS 2 applications.",
                "metadata": {"chapter": "simulation", "section": "gazebo"}
            },
        ]

    async def search(self, collection_name: str, query_vector: List[float], limit: int = 5):
        """Simulate vector search - return docs sorted by mock score"""
        # Simple keyword matching for demo (real mock would use embeddings)
        results = []
        for i, doc in enumerate(self.sample_docs[:limit]):
            results.append({
                "id": doc["id"],
                "score": 0.95 - (i * 0.1),  # Decreasing scores
                "payload": {
                    "text": doc["text"],
                    **doc["metadata"]
                }
            })
        return results

    async def search_empty(self, *args, **kwargs):
        """Simulate no results found"""
        return []

    async def search_error(self, *args, **kwargs):
        """Simulate Qdrant unavailable"""
        raise Exception("Qdrant connection failed")

@pytest.fixture
def mock_qdrant_client():
    return MockQdrantClient()

@pytest.fixture
def mock_qdrant_service(mocker, mock_qdrant_client):
    """Mock QdrantService with fixture client"""
    mock = mocker.patch('app.services.qdrant.QdrantService')
    mock.return_value.client = mock_qdrant_client
    return mock
```

**Usage in Tests**:

```python
# tests/integration/test_qdrant_retrieval.py
import pytest
from app.agents.qdrant_retrieval import QdrantRetrievalAgent

@pytest.mark.asyncio
async def test_retrieval_success(mock_qdrant_service):
    agent = QdrantRetrievalAgent(qdrant_service=mock_qdrant_service)
    results = await agent.retrieve("ROS 2 tutorial")

    assert len(results) > 0
    assert results[0]["score"] > 0.9
    assert "ros" in results[0]["text"].lower()

@pytest.mark.asyncio
async def test_retrieval_empty_results(mock_qdrant_client):
    mock_qdrant_client.search = mock_qdrant_client.search_empty
    agent = QdrantRetrievalAgent(qdrant_service=mock_qdrant_client)
    results = await agent.retrieve("nonexistent topic")

    assert len(results) == 0

@pytest.mark.asyncio
async def test_retrieval_error_handling(mock_qdrant_client):
    mock_qdrant_client.search = mock_qdrant_client.search_error
    agent = QdrantRetrievalAgent(qdrant_service=mock_qdrant_client)

    with pytest.raises(Exception):
        await agent.retrieve("any question")
```

**Alternatives Considered**:
- **VCR.py**: Records real HTTP responses, but Qdrant uses gRPC (not HTTP)
- **Docker Test Container**: Slow startup, requires Docker in CI
- **In-Memory Qdrant**: Still requires Qdrant library, unnecessary complexity

---

## Best Practices Summary

### OpenAI Agents SDK Integration

**Pattern**: Use `openai-agents` SDK for agent orchestration

```python
# app/agents/rag_chat_agent.py
from openai_agents import Agent, Tool
from openai import OpenAI

class RAGChatAgent:
    def __init__(self):
        self.client = OpenAI()
        self.agent = Agent(
            name="RAG Chat Assistant",
            instructions=self.load_system_prompt(),
            tools=[
                self.query_router_tool(),
                self.qdrant_retrieval_tool(),
                self.answer_synthesis_tool()
            ]
        )

    def load_system_prompt(self) -> str:
        """FR-009: Anti-hallucination prompt"""
        with open("app/utils/prompts/answer_synthesis.txt") as f:
            return f.read()

    async def process_query(self, question: str) -> dict:
        """Main orchestration entry point"""
        response = await self.agent.run(question)
        return response
```

**Key Patterns**:
1. System prompt loaded from file (easier to iterate)
2. Tools map to sub-agents
3. Error recovery built into agent framework

### CORS Configuration for GitHub Pages

```python
# app/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Development
        "https://asmaiqbal01.github.io",  # GitHub Pages production
    ],
    allow_credentials=False,  # FR-021: No cookies
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type", "Authorization"],
)
```

### Environment-Based API URL

```typescript
// frontend/src/services/chatApi.ts
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-space.hf.space'  // Hugging Face Spaces URL
  : 'http://localhost:8000'

export const chatApi = {
  async query(question: string) {
    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ question })
    })
    return response.json()
  }
}
```

---

## Research Completion Summary

| Question | Decision | Confidence | Next Steps |
|----------|----------|------------|------------|
| State Management | Zustand | HIGH | Add to package.json |
| Testing Framework | Jest + RTL | HIGH | Configure jest.config.js |
| Agent Testing | Hybrid (Unit + Integration) | HIGH | Set up test structure |
| Qdrant Mocking | Fixture-based | HIGH | Create mock_qdrant.py |

**All NEEDS CLARIFICATION items resolved.** Ready to proceed to Phase 1: Data Model & Contracts.
