# Data Model: RAG Chatbot Agent System

**Feature**: 001-rag-chatbot-agent-system
**Date**: 2025-12-20
**Source**: Extracted from spec.md Key Entities section

---

## Core Entities

### 1. Message

**Purpose**: Represents a single chat message in the conversation

**Attributes**:
- `id`: string (UUID) - Unique message identifier
- `role`: enum('user' | 'assistant') - Who sent the message
- `content`: string - Message text content
- `timestamp`: datetime - When the message was created
- `sources`: optional string[] - Source references (chapter/section) for assistant messages

**Validation Rules**:
- `id` must be a valid UUID v4
- `role` must be either 'user' or 'assistant'
- `content` must not be empty for user messages
- `content` must not exceed 10,000 characters (FR edge case)
- `timestamp` must be in ISO 8601 format
- `sources` only present when role='assistant' and answer is grounded

**State Transitions**: N/A (immutable once created)

**Relationships**:
- Messages exist only in frontend state (ephemeral)
- Not persisted to any backend storage (FR-023)
- Cleared when chat closes (FR-016)

**Python Model** (Backend):
```python
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Literal, Optional
from uuid import UUID, uuid4

class Message(BaseModel):
    id: UUID = Field(default_factory=uuid4)
    role: Literal["user", "assistant"]
    content: str = Field(min_length=1, max_length=10000)
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    sources: Optional[list[str]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "id": "550e8400-e29b-41d4-a716-446655440000",
                "role": "assistant",
                "content": "ROS 2 is a robotics middleware framework...",
                "timestamp": "2025-12-20T10:30:00Z",
                "sources": ["intro/what-is-ros2", "basics/core-concepts"]
            }
        }
```

**TypeScript Type** (Frontend):
```typescript
export interface Message {
  id: string
  role: 'user' | 'assistant'
  content: string
  timestamp: Date
  sources?: string[]
}
```

---

### 2. QueryResult

**Purpose**: Represents retrieval results from Qdrant vector database

**Attributes**:
- `query_text`: string - Original user query (normalized)
- `retrieved_chunks`: RetrievedChunk[] - List of relevant document chunks
- `total_results`: integer - Number of chunks retrieved
- `retrieval_time_ms`: float - Time taken to retrieve (performance monitoring)

**RetrievedChunk Sub-Model**:
- `chunk_id`: string - Qdrant document ID
- `text`: string - Chunk content
- `score`: float (0.0-1.0) - Similarity score
- `metadata`: ChunkMetadata - Source information

**ChunkMetadata Sub-Model**:
- `chapter`: string - Chapter identifier
- `section`: string - Section identifier
- `page_url`: optional string - URL to source page

**Validation Rules**:
- `query_text` must match normalized form (lowercase, trimmed)
- `retrieved_chunks` ordered by score (descending)
- `score` must be between 0.0 and 1.0
- `total_results` must equal len(retrieved_chunks)
- `retrieval_time_ms` must be positive

**State Transitions**:
1. Created after Qdrant search
2. Passed to AnswerSynthesisAgent
3. Discarded after response generation (not persisted)

**Relationships**:
- Input to AnswerSynthesisAgent
- Metadata extracted to populate Message.sources

**Python Model**:
```python
from pydantic import BaseModel, Field, validator

class ChunkMetadata(BaseModel):
    chapter: str
    section: str
    page_url: Optional[str] = None

class RetrievedChunk(BaseModel):
    chunk_id: str
    text: str = Field(min_length=1)
    score: float = Field(ge=0.0, le=1.0)
    metadata: ChunkMetadata

class QueryResult(BaseModel):
    query_text: str
    retrieved_chunks: list[RetrievedChunk]
    total_results: int
    retrieval_time_ms: float = Field(gt=0)

    @validator('total_results')
    def validate_total_results(cls, v, values):
        chunks = values.get('retrieved_chunks', [])
        if v != len(chunks):
            raise ValueError('total_results must match retrieved_chunks length')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query_text": "what is ros2?",
                "retrieved_chunks": [
                    {
                        "chunk_id": "doc1-chunk3",
                        "text": "ROS 2 is a robotics middleware...",
                        "score": 0.95,
                        "metadata": {
                            "chapter": "intro",
                            "section": "what-is-ros2",
                            "page_url": "/docs/intro/what-is-ros2"
                        }
                    }
                ],
                "total_results": 1,
                "retrieval_time_ms": 145.2
            }
        }
```

---

### 3. AgentResponse

**Purpose**: Represents the output of any agent in the system

**Attributes**:
- `success`: boolean - Whether agent execution succeeded
- `agent_name`: string - Which agent produced this response
- `result_data`: optional dict - Agent-specific result data
- `error_details`: optional ErrorContext - Error information if failed
- `execution_time_ms`: float - Time taken by agent

**Validation Rules**:
- `success=false` implies `error_details` must be present
- `success=true` implies `result_data` should be present
- `agent_name` must be one of: RAGChatAgent, QueryRouterAgent, QdrantRetrievalAgent, AnswerSynthesisAgent, ErrorRecoveryAgent
- `execution_time_ms` must be positive

**State Transitions**:
1. Created by agent after processing
2. Passed to orchestrator (RAGChatAgent)
3. May trigger error recovery if success=false

**Relationships**:
- Produced by all agents
- Consumed by RAGChatAgent orchestrator
- Contains ErrorContext on failure

**Python Model**:
```python
from enum import Enum
from typing import Any

class AgentName(str, Enum):
    RAG_CHAT_AGENT = "RAGChatAgent"
    QUERY_ROUTER = "QueryRouterAgent"
    QDRANT_RETRIEVAL = "QdrantRetrievalAgent"
    ANSWER_SYNTHESIS = "AnswerSynthesisAgent"
    ERROR_RECOVERY = "ErrorRecoveryAgent"

class AgentResponse(BaseModel):
    success: bool
    agent_name: AgentName
    result_data: Optional[dict[str, Any]] = None
    error_details: Optional['ErrorContext'] = None
    execution_time_ms: float = Field(gt=0)

    @validator('error_details')
    def validate_error_details(cls, v, values):
        if not values.get('success') and v is None:
            raise ValueError('error_details required when success=false')
        return v

    @validator('result_data')
    def validate_result_data(cls, v, values):
        if values.get('success') and v is None:
            raise ValueError('result_data should be present when success=true')
        return v
```

---

### 4. ErrorContext

**Purpose**: Represents error classification and user-safe error information

**Attributes**:
- `error_type`: enum - Classification of error
- `original_error`: string - Full error message (backend only, logged)
- `user_safe_message`: string - Sanitized message for frontend
- `retry_able`: boolean - Whether user can retry
- `log_details`: dict - Additional context for debugging

**Error Types**:
- `NETWORK_ERROR`: Connection failures (Qdrant, LLM API)
- `DATABASE_ERROR`: Qdrant-specific errors
- `LLM_ERROR`: OpenAI API errors (timeout, rate limit, invalid response)
- `VALIDATION_ERROR`: Input validation failures
- `AGENT_ERROR`: Agent processing failures
- `UNKNOWN_ERROR`: Unclassified errors

**Validation Rules**:
- `error_type` must be one of defined types
- `user_safe_message` must NOT contain stack traces (FR-011, SC-005)
- `user_safe_message` must NOT contain sensitive information (API keys, env vars)
- `retry_able` should be true for transient errors (network, timeout)
- `log_details` must include timestamp and request_id

**State Transitions**:
1. Created by ErrorRecoveryAgent when exception caught
2. Included in AgentResponse
3. Logged server-side
4. user_safe_message returned to frontend

**Relationships**:
- Created by ErrorRecoveryAgent (FR-008)
- Embedded in AgentResponse
- Logged separately for debugging

**Python Model**:
```python
from enum import Enum
from typing import Any

class ErrorType(str, Enum):
    NETWORK_ERROR = "network"
    DATABASE_ERROR = "database"
    LLM_ERROR = "llm"
    VALIDATION_ERROR = "validation"
    AGENT_ERROR = "agent"
    UNKNOWN_ERROR = "unknown"

class ErrorContext(BaseModel):
    error_type: ErrorType
    original_error: str
    user_safe_message: str
    retry_able: bool
    log_details: dict[str, Any] = Field(default_factory=dict)

    @validator('user_safe_message')
    def validate_safe_message(cls, v, values):
        # Ensure no stack traces
        if 'Traceback' in v or 'File "' in v:
            raise ValueError('user_safe_message must not contain stack traces')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "error_type": "database",
                "original_error": "QdrantException: Connection timeout after 5.0s",
                "user_safe_message": "The assistant is temporarily unavailable. Please try again later.",
                "retry_able": True,
                "log_details": {
                    "request_id": "req-123",
                    "timestamp": "2025-12-20T10:30:00Z",
                    "qdrant_host": "qdrant.example.com"
                }
            }
        }
```

**TypeScript Type**:
```typescript
export type ErrorType = 'network' | 'database' | 'llm' | 'validation' | 'agent' | 'unknown'

export interface ErrorContext {
  error_type: ErrorType
  user_safe_message: string
  retry_able: boolean
}
```

---

## Entity Lifecycle Summary

```
User Query Flow:
1. Message (role=user) created in frontend
2. Sent to backend /query endpoint
3. QueryRouterAgent validates → AgentResponse
4. QdrantRetrievalAgent searches → QueryResult
5. AnswerSynthesisAgent generates → Message (role=assistant)
6. If error at any step → ErrorRecoveryAgent → ErrorContext
7. Final Message returned to frontend
8. Messages stored in Zustand until chat close
9. Chat close → all Messages cleared (FR-016)
```

## Data Flow Diagram

```
┌─────────────────────┐
│  Frontend (Zustand) │
│  messages: Message[]│
└──────────┬──────────┘
           │
           │ POST /query { question }
           ▼
┌─────────────────────────────────────────────┐
│          Backend Agent Pipeline             │
│                                             │
│  ┌────────────────┐                        │
│  │ QueryRouter    │ validates              │
│  │ AgentResponse  │                        │
│  └────────┬───────┘                        │
│           │                                 │
│           ▼                                 │
│  ┌────────────────┐                        │
│  │ QdrantRetrieval│ searches               │
│  │ QueryResult    │                        │
│  └────────┬───────┘                        │
│           │                                 │
│           ▼                                 │
│  ┌────────────────┐                        │
│  │AnswerSynthesis │ generates              │
│  │ Message        │                        │
│  └────────┬───────┘                        │
│           │                                 │
│           │  On Error: ErrorRecoveryAgent  │
│           │            ErrorContext         │
│           ▼                                 │
│  ┌────────────────┐                        │
│  │ RAGChatAgent   │ orchestrates           │
│  │ AgentResponse  │                        │
│  └────────┬───────┘                        │
└───────────┼─────────────────────────────────┘
            │
            │ { answer, sources }
            ▼
┌─────────────────────┐
│  Frontend (Zustand) │
│  addMessage()       │
└─────────────────────┘
```

---

## Storage & Persistence Rules

Per FR-021 through FR-024:

| Entity | Frontend Storage | Backend Storage | Lifecycle |
|--------|------------------|-----------------|-----------|
| Message | Zustand (in-memory) | None | Cleared on chat close |
| QueryResult | None | None | Discarded after use |
| AgentResponse | None | None | Discarded after logging |
| ErrorContext | None (only user_safe_message) | Logs only | Logged then discarded |

**NO PERSISTENCE**:
- No localStorage
- No sessionStorage
- No cookies
- No backend database for chat history
- Qdrant contains only document embeddings, NOT chat logs
