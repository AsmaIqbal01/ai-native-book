# Multi-Agent RAG System - Test Pyramid Plan

## Overview
Test pyramid for the 5-agent RAG architecture (FR-004 to FR-008), ensuring constitutional compliance and correctness.

**Testing Framework**: pytest + pytest-asyncio + pytest-cov
**Target Coverage**: >80% for agents, 100% for critical paths
**Constitutional Requirement**: Principle II - Zero untested code in production

---

## Test Pyramid Structure

```
                    ┌─────────────────────┐
                    │  Constitutional     │  (2-3 tests)
                    │  End-to-End Tests   │  Verify principles
                    └─────────────────────┘
                            ▲
                    ┌───────────────────────┐
                    │   Integration Tests    │ (10-15 tests)
                    │  Agent Coordination    │ Real agents, mocked services
                    └───────────────────────┘
                            ▲
            ┌───────────────────────────────────┐
            │         Unit Tests                │ (40-60 tests)
            │  Individual Agent Logic           │ Isolated, fast
            └───────────────────────────────────┘
```

---

## 1. Unit Tests (40-60 tests)

### 1.1 BaseAgent Tests (`tests/unit/agents/test_base_agent.py`)
- ✅ Test abstract class contract enforcement
- ✅ Test `run()` lifecycle (validate → execute)
- ✅ Test validation failure raises ValueError
- ✅ Test generic type handling

**Mock Strategy**: No external dependencies

---

### 1.2 ErrorRecoveryAgent Tests (`tests/unit/agents/test_error_recovery_agent.py`)
**Target**: 15-20 test cases

#### Error Classification Tests
- ✅ Test ConnectionError → error_type="connection", status_code=503
- ✅ Test asyncio.TimeoutError → error_type="timeout", status_code=504
- ✅ Test ValidationError → error_type="validation", status_code=400
- ✅ Test RateLimitError (quota) → error_type="quota", status_code=429
- ✅ Test RateLimitError (rate limit) → error_type="rate_limit", status_code=429
- ✅ Test generic Exception → error_type="unknown", status_code=500

#### Retry Logic Tests
- ✅ Test connection errors should_retry=True
- ✅ Test timeout errors should_retry=True
- ✅ Test validation errors should_retry=False
- ✅ Test quota errors should_retry=False

#### User Message Sanitization
- ✅ Test no sensitive data in user_message
- ✅ Test technical_details contains full traceback
- ✅ Test user-friendly language in messages

**Mock Strategy**: Create mock exceptions with specific error strings

---

### 1.3 QueryRouterAgent Tests (`tests/unit/agents/test_query_router_agent.py`)
**Target**: 12-15 test cases

#### Input Validation
- ✅ Test empty question rejected
- ✅ Test whitespace-only question rejected
- ✅ Test valid question passes validation

#### Greeting Detection
- ✅ Test "hi" detected as greeting
- ✅ Test "hello there" detected as greeting
- ✅ Test "hey what's up" (>3 words) NOT detected as greeting
- ✅ Test "what is ROS?" NOT detected as greeting

#### Out-of-Scope Detection
- ✅ Test "what's the weather today" → out_of_scope
- ✅ Test "who won the election" → out_of_scope
- ✅ Test "explain ROS 2 navigation" → valid (in-scope)

#### Query Normalization
- ✅ Test whitespace trimming
- ✅ Test lowercasing for routing (not in output)

#### Mode Detection
- ✅ Test normal_rag mode when no selected_text
- ✅ Test selected_text_only mode when selected_text provided

**Mock Strategy**: No external dependencies

---

### 1.4 QdrantRetrievalAgent Tests (`tests/unit/agents/test_qdrant_retrieval_agent.py`)
**Target**: 10-12 test cases

#### Input Validation
- ✅ Test empty query rejected
- ✅ Test whitespace-only query rejected
- ✅ Test valid query passes validation

#### Retrieval Logic (with mocks)
- ✅ Test successful retrieval returns chunks + context
- ✅ Test chapter filter applied correctly
- ✅ Test section filter applied correctly
- ✅ Test top_k parameter respected
- ✅ Test empty results handled gracefully

#### Error Handling
- ✅ Test Qdrant connection error propagates
- ✅ Test embedding service error propagates
- ✅ Test timeout error propagates

#### Context Building
- ✅ Test build_context_normal_rag called with chunks
- ✅ Test context string is non-empty when chunks retrieved

**Mock Strategy**: Mock EmbeddingService, RetrieverService, build_context_normal_rag

---

### 1.5 AnswerSynthesisAgent Tests (`tests/unit/agents/test_answer_synthesis_agent.py`)
**Target**: 15-18 test cases

#### Input Validation
- ✅ Test empty context rejected
- ✅ Test empty query rejected
- ✅ Test valid input passes validation

#### System Prompt Loading
- ✅ Test system prompt loaded from specs/agent.system.md
- ✅ Test FileNotFoundError if prompt missing

#### Multi-LLM Failover
- ✅ Test primary provider success
- ✅ Test primary fails → fallback 1 succeeds
- ✅ Test primary + fallback 1 fail → fallback 2 succeeds
- ✅ Test all providers fail → raises Exception

#### Rate Limit Handling
- ✅ Test quota exhaustion triggers failover
- ✅ Test rate limit triggers failover
- ✅ Test non-rate-limit error triggers failover

#### User Prompt Building
- ✅ Test normal_rag mode prompt format
- ✅ Test selected_text_only mode prompt format
- ✅ Test [SELECTED TEXT ONLY] marker present in selected mode

#### Citation Generation
- ✅ Test citations include chapter, section, page
- ✅ Test chunk_text truncated to 200 chars
- ✅ Test empty chunks → empty citations

**Mock Strategy**: Mock AsyncOpenAI client, mock system prompt file read

---

### 1.6 RAGChatAgent Tests (`tests/unit/agents/test_rag_chat_agent.py`)
**Target**: 12-15 test cases

#### Input Validation
- ✅ Test empty question rejected
- ✅ Test valid input passes validation

#### Orchestration Flow (with mocked sub-agents)
- ✅ Test greeting mode returns friendly response
- ✅ Test out-of-scope returns polite rejection
- ✅ Test normal_rag flow: router → retrieval → synthesis
- ✅ Test selected_text_only flow: router → synthesis (no retrieval)

#### No-Results Handling
- ✅ Test empty chunks from Qdrant → polite "not found" message

#### Error Recovery Integration
- ✅ Test exception during routing → ErrorRecoveryAgent invoked
- ✅ Test exception during retrieval → ErrorRecoveryAgent invoked
- ✅ Test exception during synthesis → ErrorRecoveryAgent invoked
- ✅ Test HTTPException re-raised correctly

#### Metadata Propagation
- ✅ Test metadata includes chunks_retrieved, provider_used, mode

**Mock Strategy**: Mock all 4 sub-agents (QueryRouter, QdrantRetrieval, AnswerSynthesis, ErrorRecovery)

---

## 2. Integration Tests (10-15 tests)

### 2.1 Agent Coordination Tests (`tests/integration/test_agent_coordination.py`)
**Target**: 8-10 test cases

#### Real Agent Integration (mocked external services only)
- ✅ Test full pipeline with real agents + mock Qdrant + mock OpenAI
- ✅ Test greeting detected and handled without Qdrant call
- ✅ Test out-of-scope detected and handled without Qdrant call
- ✅ Test normal_rag retrieves from mock Qdrant
- ✅ Test selected_text_only bypasses Qdrant
- ✅ Test multi-LLM failover with real AnswerSynthesisAgent

#### Error Propagation
- ✅ Test Qdrant timeout → ErrorRecoveryAgent → 504 HTTP response
- ✅ Test LLM quota exhaustion → failover → success
- ✅ Test all LLMs fail → ErrorRecoveryAgent → 503 HTTP response

**Mock Strategy**: Mock Qdrant client, mock OpenAI client, use real agent instances

---

### 2.2 Query Endpoint Tests (`tests/integration/test_query_endpoint.py`)
**Target**: 5-7 test cases

#### API Contract Tests
- ✅ Test POST /query with valid request → 200 OK
- ✅ Test POST /query with missing question → 400 Bad Request
- ✅ Test POST /query with feature flag OFF → legacy path
- ✅ Test POST /query with feature flag ON → multi-agent path
- ✅ Test response schema matches QueryResponse

#### Feature Flag Toggle
- ✅ Test settings.use_multi_agent_system=False uses RAGAgent
- ✅ Test settings.use_multi_agent_system=True uses RAGChatAgent

**Mock Strategy**: Mock Qdrant, mock OpenAI, use TestClient from FastAPI

---

## 3. Constitutional Tests (2-3 tests)

### 3.1 Constitutional Compliance Tests (`tests/constitution/test_constitution_compliance.py`)
**Target**: 3-4 test cases

#### Principle I: Spec-Driven Development
- ✅ Test FR-004 to FR-008 implemented (agent count, types)
- ✅ Test data contracts match spec (OrchestratorInput, RouterResult, etc.)

#### Principle II: Zero Untested Code
- ✅ Test coverage >80% for app/agents/**
- ✅ Test all agents have unit tests

#### Principle III: Performance (Latency <3s)
- ✅ Test end-to-end query latency <3000ms (with real agents, mocked services)
- ✅ Test latency_ms in metadata

#### Principle IV: Security & Ethics
- ✅ Test no API keys in logs
- ✅ Test error messages don't leak sensitive data

**Mock Strategy**: Minimal mocking, use real agents with controlled service responses

---

## Test Execution Plan

### Phase 2.1: Unit Tests (Current Phase)
1. Create test fixtures and mocks
2. Implement ErrorRecoveryAgent tests (15-20)
3. Implement QueryRouterAgent tests (12-15)
4. Implement QdrantRetrievalAgent tests (10-12)
5. Implement AnswerSynthesisAgent tests (15-18)
6. Implement RAGChatAgent tests (12-15)

**Acceptance Criteria**: >80% coverage for agents, all tests passing

### Phase 2.2: Integration Tests
1. Create mock factories for Qdrant/OpenAI
2. Implement agent coordination tests (8-10)
3. Implement query endpoint tests (5-7)

**Acceptance Criteria**: All integration tests passing, feature flag validated

### Phase 2.3: Constitutional Tests
1. Implement constitutional compliance tests (3-4)
2. Run coverage report (pytest-cov)
3. Validate latency budgets

**Acceptance Criteria**: All principles pass, coverage >80%

---

## Test Execution Commands

```bash
# Run all tests
pytest tests/

# Run unit tests only
pytest tests/unit/

# Run integration tests only
pytest tests/integration/

# Run constitutional tests only
pytest tests/constitution/

# Run with coverage
pytest --cov=app.agents --cov-report=html tests/

# Run specific agent tests
pytest tests/unit/agents/test_rag_chat_agent.py -v
```

---

## Success Metrics

- **Unit Tests**: 40-60 tests, >80% code coverage
- **Integration Tests**: 10-15 tests, all passing
- **Constitutional Tests**: 3-4 tests, all passing
- **Total Test Count**: 55-80 tests
- **Execution Time**: <30 seconds for unit tests, <2 minutes for all tests

---

## Mock Strategies Summary

| Component | Mock Strategy |
|-----------|---------------|
| BaseAgent | No mocks (abstract class testing) |
| ErrorRecoveryAgent | Mock exceptions with specific error strings |
| QueryRouterAgent | No mocks (pure logic) |
| QdrantRetrievalAgent | Mock EmbeddingService, RetrieverService |
| AnswerSynthesisAgent | Mock AsyncOpenAI, mock file read |
| RAGChatAgent | Mock all 4 sub-agents |
| Integration Tests | Mock Qdrant client, mock OpenAI client |
| Constitutional Tests | Minimal mocking, real agents |

---

## Next Steps

1. ✅ Design test pyramid (this document)
2. Create pytest fixtures and conftest.py
3. Implement unit tests (ErrorRecovery → QueryRouter → QdrantRetrieval → AnswerSynthesis → RAGChat)
4. Implement integration tests
5. Implement constitutional tests
6. Run coverage report and validate >80%
7. Update constitution compliance score from 60% → 90%+

**Status**: Test pyramid designed ✅
**Next**: Implement unit tests for ErrorRecoveryAgent
