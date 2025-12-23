# Implementation Plan: RAG Chatbot with Agent Architecture and Ephemeral Sessions

**Branch**: `001-rag-chatbot-agent-system` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-agent-system/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a production-grade RAG chatbot system that provides document-grounded answers through a modular agent architecture. The system retrieves context exclusively from Qdrant vector database, processes queries through specialized sub-agents (QueryRouter, QdrantRetrieval, AnswerSynthesis, ErrorRecovery), and maintains ephemeral chat sessions that clear completely on widget close. Frontend is React/TypeScript with Zustand state management; backend is FastAPI with OpenAI Agents SDK orchestration.

## Technical Context

### Backend (Python/FastAPI)

**Language/Version**: Python 3.11
**Primary Dependencies**:
- FastAPI 0.115+ (web framework)
- OpenAI Agents SDK 0.0.19 (agent orchestration)
- Qdrant Client 1.16+ (vector database)
- Cohere 5.20+ (embeddings)
- OpenAI 1.93+ (LLM provider)
- Pydantic 2.11+ (data validation)

**Storage**:
- Qdrant Cloud/Self-hosted (vector database for document embeddings)
- AsyncPG + PostgreSQL (optional: for future analytics, not chat history)
- No persistent storage for chat sessions (ephemeral only)

**Testing**: pytest 9.0+, pytest-asyncio 1.3+, pytest-cov 7.0+

**Target Platform**:
- Development: Local (Python 3.11+)
- Production: Hugging Face Spaces (Docker container)

**Project Type**: Web backend (API service)

**Performance Goals**:
- <3 seconds response time for 95% of queries (SC-001)
- <200ms Qdrant retrieval latency
- 50 concurrent queries without degradation (SC-008)

**Constraints**:
- No chat history persistence (ephemeral sessions)
- No cookies or server-side session storage
- <100ms state cleanup on session close (SC-003)
- Zero stack traces to end users (SC-005)

**Scale/Scope**:
- MVP: Single documentation collection (~1000-5000 chunks)
- 50 concurrent users
- 4 specialized sub-agents + 1 orchestration agent
- 3 REST endpoints (health, query, chapters)

### Frontend (React/TypeScript)

**Language/Version**: TypeScript 5.6, React 18.3

**Primary Dependencies**:
- React 18.3+ (UI framework)
- Docusaurus 3.9+ (documentation platform)
- Zustand (state management - NEEDS CLARIFICATION: Currently uses React Context, need to verify Zustand integration)
- Tailwind CSS 4.1+ (styling)

**Storage**: In-memory component state only (no persistence)

**Testing**: NEEDS CLARIFICATION (Jest/Vitest not in current dependencies)

**Target Platform**:
- Development: Local dev server
- Production: GitHub Pages (static site)

**Project Type**: Web frontend (Docusaurus site with React components)

**Performance Goals**:
- <5 seconds total interaction time from open to answer (SC-006)
- <100ms chat clear on widget close (SC-003)
- Responsive across desktop/tablet/mobile

**Constraints**:
- Must integrate with existing Docusaurus site
- No localStorage/sessionStorage for chat
- High z-index for visibility
- Bottom-left fixed positioning

**Scale/Scope**:
- Single floating chat widget
- Message list component
- Input component with loading states
- Error display component

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Code Quality & Best Practices ✅

**Status**: PASS

- Python backend code will follow PEP 8 style guidelines
- TypeScript/React code will follow standard conventions
- Agent orchestration follows single-responsibility principle (4 specialized sub-agents)
- All dependencies explicitly declared with version constraints
- Complexity justified by modular architecture (better than monolithic RAG)

**Justification**: Agent-based architecture aligns with constitution's modularity principle.

### II. Testing & Validation Standards ⚠️

**Status**: NEEDS ATTENTION

**Concerns**:
- Frontend testing framework not yet established (no Jest/Vitest in dependencies)
- Agent integration tests need definition
- Qdrant mock/test strategy undefined

**Required Actions** (Phase 0 Research):
1. Research frontend testing setup for Docusaurus + React
2. Define agent testing strategy (unit vs integration)
3. Establish Qdrant mocking approach for tests

**Commitment**:
- All agent functions will include unit tests
- API endpoints will have integration tests with mocked Qdrant
- Frontend components will have component tests once framework selected

### III. User Experience & Consistency ✅

**Status**: PASS

- Chat widget follows consistent component structure
- Error messages use uniform, user-friendly language
- Loading states provide clear feedback
- Component hierarchy: Widget → Window → MessageList/Input
- Terminology consistency enforced through shared types

**Alignment**: Matches constitution's UX consistency requirements.

### IV. Performance & Accessibility ✅

**Status**: PASS with MONITORING

**Compliance**:
- API latency targets defined (<3s for 95% queries)
- LLM API cost optimization via sub-agent routing (QueryRouter filters unnecessary calls)
- Resource requirements documented (Python 3.11, Node 20+)
- Responsive design across screen sizes

**Monitoring Required**:
- Qdrant query performance (target <200ms)
- LLM token usage per query
- Concurrent request handling under load

**Alignment**: Meets performance accessibility for student hardware (no GPU required).

### Safety Requirements ✅

**Status**: PASS (N/A for this feature)

This feature does not involve robotics control, physical hardware, or safety-critical operations. Safety requirements are not applicable.

### Content Generation Rules ✅

**Status**: PASS (N/A for this feature)

This feature implements infrastructure (chatbot system), not educational content. Content generation rules apply to documentation/chapters, not the chat system itself.

### Deployment & Publication ✅

**Status**: PASS

**Compliance**:
- Backend deployable to Hugging Face Spaces via Docker
- Frontend integrates with existing GitHub Pages deployment
- Environment variables documented in .env.example
- No hardcoded secrets (per FR requirements)
- Git commits required before deployment

**Readiness**: Deployment targets clearly defined and feasible.

### Constitution Check Summary

| Principle | Status | Action Required |
|-----------|--------|-----------------|
| I. Code Quality | ✅ PASS | None |
| II. Testing | ⚠️ NEEDS ATTENTION | Phase 0: Define testing strategy |
| III. UX Consistency | ✅ PASS | None |
| IV. Performance | ✅ PASS | Monitor metrics post-implementation |
| Safety | ✅ N/A | None |
| Content Rules | ✅ N/A | None |
| Deployment | ✅ PASS | None |

**Overall Gate Status**: CONDITIONAL PASS

Proceed to Phase 0 research with requirement to resolve testing strategy unknowns.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-agent-system/
├── spec.md              # Feature specification (completed)
├── checklists/
│   └── requirements.md  # Spec quality validation (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (next step)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (API schemas)
│   ├── openapi.yaml     # REST API contract
│   └── types.ts         # TypeScript type definitions
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

**Structure Decision**: Web application (Option 2) - Backend and Frontend are separate codebases with independent deployment targets (Hugging Face Spaces and GitHub Pages).

#### Backend Structure (RAG-backend/)

```text
RAG-backend/
├── app/
│   ├── __init__.py
│   ├── main.py                  # FastAPI application entry
│   ├── config.py                # Environment configuration
│   │
│   ├── agents/                  # Agent orchestration layer
│   │   ├── __init__.py
│   │   ├── rag_chat_agent.py    # Primary orchestration agent
│   │   ├── query_router.py      # FR-005: Input validation & routing
│   │   ├── qdrant_retrieval.py  # FR-006: Vector DB retrieval
│   │   ├── answer_synthesis.py  # FR-007: LLM-based answer generation
│   │   └── error_recovery.py    # FR-008: Error classification & handling
│   │
│   ├── api/                     # REST API endpoints
│   │   ├── __init__.py
│   │   ├── health.py            # FR-001: Health check endpoint
│   │   ├── query.py             # FR-002: Query endpoint
│   │   └── chapters.py          # FR-003: Chapters metadata endpoint
│   │
│   ├── models/                  # Pydantic models & data classes
│   │   ├── __init__.py
│   │   ├── message.py           # Message entity
│   │   ├── query_result.py      # QueryResult entity
│   │   ├── agent_response.py    # AgentResponse entity
│   │   └── error_context.py     # ErrorContext entity
│   │
│   ├── services/                # Business logic services
│   │   ├── __init__.py
│   │   ├── embedding.py         # Embedding service (Cohere/OpenAI)
│   │   ├── qdrant.py            # Qdrant client wrapper
│   │   └── llm.py               # LLM service (OpenAI)
│   │
│   ├── db/                      # Database utilities
│   │   ├── __init__.py
│   │   └── qdrant_client.py     # Qdrant connection management
│   │
│   └── utils/                   # Shared utilities
│       ├── __init__.py
│       ├── logging.py           # Structured logging setup
│       └── prompts/             # System prompts
│           └── answer_synthesis.txt  # FR-009: Anti-hallucination prompt
│
├── tests/
│   ├── unit/                    # Unit tests for agents/services
│   │   ├── test_query_router.py
│   │   ├── test_qdrant_retrieval.py
│   │   ├── test_answer_synthesis.py
│   │   └── test_error_recovery.py
│   │
│   ├── integration/             # API endpoint tests
│   │   ├── test_health.py
│   │   ├── test_query.py
│   │   └── test_chapters.py
│   │
│   └── fixtures/                # Test fixtures & mocks
│       ├── mock_qdrant.py
│       └── sample_documents.py
│
├── .env.example                 # Environment variable template
├── requirements.txt             # Python dependencies
├── pyproject.toml              # Project metadata
├── Dockerfile                   # Hugging Face Spaces deployment
└── README.md                    # Backend documentation
```

#### Frontend Structure (frontend/Physical AI and Robotics/)

```text
frontend/Physical AI and Robotics/
├── src/
│   ├── components/              # React components
│   │   ├── FloatingChatWidget.tsx    # FR-013: Main widget container
│   │   ├── ChatWindow.tsx            # Chat window with open/close
│   │   ├── MessageBubble.tsx         # Individual message display
│   │   ├── ChatInput.tsx             # User input with loading state
│   │   └── SystemNotification.tsx    # FR-018: Error display
│   │
│   ├── contexts/                # React Context (or Zustand store)
│   │   └── ChatContext.tsx      # FR-015: Chat state management
│   │                            # State: isChatOpen, messages[]
│   │                            # FR-016: Clear on close logic
│   │
│   ├── services/                # API communication
│   │   └── chatApi.ts           # Backend API client
│   │                            # Endpoints: /health, /query, /chapters
│   │
│   ├── types/                   # TypeScript type definitions
│   │   ├── message.ts           # Message type
│   │   └── api.ts               # API request/response types
│   │
│   └── css/
│       └── chat-widget.css      # FR-013/014: Widget styling
│                                # z-index, positioning, animations
│
├── tests/                       # Frontend tests (framework TBD in Phase 0)
│   └── components/
│       ├── FloatingChatWidget.test.tsx
│       ├── ChatWindow.test.tsx
│       └── ChatInput.test.tsx
│
├── package.json                 # Node dependencies
├── tsconfig.json               # TypeScript configuration
├── tailwind.config.js          # Tailwind CSS config
└── docusaurus.config.ts        # Docusaurus configuration
```

**Key Integration Points**:
1. Backend API URL configured in `chatApi.ts` (environment-based)
2. CORS configuration in FastAPI `main.py` to allow GitHub Pages origin
3. Widget injected into Docusaurus via custom React component
4. State management ensures no persistence (FR-017, FR-021-024)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations require justification. The agent-based architecture is explicitly required by the feature specification (FR-004 through FR-008) and aligns with the constitution's modularity principles.
