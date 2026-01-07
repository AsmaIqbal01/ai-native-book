# Implementation Plan: Spec 1 - Core RAG Backend Foundation

**Branch**: `refactor-deployment-parity-fix` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/refactor-deployment-parity-fix/spec.md`

## Summary

Spec 1 establishes the foundational RAG (Retrieval-Augmented Generation) backend: content ingestion → chunking → embedding → vector storage → semantic retrieval. This is the minimum viable system that enables all future specifications (agents, frontend, token optimization).

**Core Capability**: Accept textbook content, chunk and embed it, store in Qdrant, and retrieve relevant passages via HTTP API.

**Technical Approach**: FastAPI backend with Qdrant vector database, Cohere/OpenAI embeddings, and structured context formatting. No LLM integration, no agents, no frontend—just reliable ingestion and retrieval.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI 0.115+, Qdrant Client 1.7+, Cohere SDK or OpenAI SDK, Pydantic 2.0+, Uvicorn
**Storage**: Qdrant vector database (cloud or self-hosted), no relational DB for Spec 1
**Testing**: pytest, pytest-asyncio, httpx (for async FastAPI testing)
**Target Platform**: Linux server, Docker containerized deployment, localhost development
**Project Type**: Single backend API (no frontend in Spec 1)
**Performance Goals**:
- Ingest: Process 10k-50k character documents in <30 seconds
- Query: Retrieve top-5 chunks in <500ms (p95 latency)
- Embedding: <2 seconds per chunk (Cohere/OpenAI API latency)

**Constraints**:
- Qdrant connection required (no offline mode)
- Embedding API quota limits (handle rate limits gracefully)
- CORS enabled for future frontend integration
- Docker image <500MB (slim Python base image)

**Scale/Scope**:
- MVP: 5-10 textbook chapters (~100k words total)
- Chunk size: 500-1000 characters
- Vector dimensions: 768 (Cohere) or 1024 (OpenAI)
- Expected queries: <100/day during development

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Code Quality & Best Practices

| Principle | Status | Notes |
|-----------|--------|-------|
| Code MUST run without errors | ✅ PASS | All endpoints will be integration tested |
| Follow PEP 8 style guidelines | ✅ PASS | Using Black formatter, Ruff linter |
| Modular with single-responsibility functions | ✅ PASS | Separate services for ingest, retrieval, context building |
| Comments explain "why" not "what" | ✅ PASS | Document embedding API choices, chunking rationale |
| Complexity avoided unless justified | ✅ PASS | Simple pipeline: chunk → embed → store → search |
| Dependencies explicitly declared | ✅ PASS | requirements.txt with pinned versions |

### II. Testing & Validation Standards

| Principle | Status | Notes |
|-----------|--------|-------|
| Every code example run locally | ✅ PASS | All API endpoints tested via pytest |
| Verification instructions included | ✅ PASS | README.md documents test commands |
| LLM-generated code manually reviewed | ⚠️ N/A | No LLM code generation in Spec 1 |
| Integration tests provided | ✅ PASS | `/ingest` and `/query` integration tests required |
| Test commands documented | ✅ PASS | `pytest tests/` in README.md |

### III. User Experience & Consistency

| Principle | Status | Notes |
|-----------|--------|-------|
| Consistent structure and style | ✅ PASS | OpenAPI docs auto-generated (FastAPI) |
| Terminology remains uniform | ✅ PASS | Glossary: chunk, embedding, vector, retrieval |
| Technical terms defined on first use | ✅ PASS | README explains RAG, Qdrant, embeddings |
| Examples progress simple to complex | ⚠️ N/A | No multi-step examples in backend-only spec |
| Code formatting consistent | ✅ PASS | Black formatter enforces style |

### IV. Performance & Accessibility

| Principle | Status | Notes |
|-----------|--------|-------|
| Run on mid-range laptops (8GB RAM) | ✅ PASS | Lightweight FastAPI, Qdrant cloud (no local resources) |
| Resource requirements documented | ✅ PASS | Docker memory limits, API quota notes in README |
| Minimize API latency and cost | ✅ PASS | Batch embeddings if possible, cache Qdrant client |
| Fallback options provided | ✅ PASS | Support both Cohere and OpenAI embeddings |

### Constitution Violations (Must Justify)

**None identified.** Spec 1 aligns with all constitution principles.

## Project Structure

### Documentation (this feature)

```text
specs/refactor-deployment-parity-fix/
├── spec.md              # Spec 1 definition (created)
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0 output (to be generated)
├── data-model.md        # Phase 1 output (to be generated)
├── quickstart.md        # Phase 1 output (to be generated)
├── contracts/           # Phase 1 output (OpenAPI schemas)
│   ├── ingest.json
│   ├── query.json
│   └── health.json
└── tasks.md             # Phase 2 output (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
RAG-backend/
├── app/
│   ├── main.py                    # FastAPI app entry point, CORS middleware
│   ├── config.py                  # Pydantic Settings (env var loading)
│   ├── api/
│   │   ├── ingest.py              # POST /ingest endpoint
│   │   ├── query.py               # POST /query endpoint (retrieval only, no LLM)
│   │   └── health.py              # GET /health endpoint
│   ├── services/
│   │   ├── chunker.py             # Text chunking logic (500-1000 chars, overlap)
│   │   ├── embedder.py            # Embedding service (Cohere/OpenAI)
│   │   ├── retriever.py           # Qdrant search service
│   │   └── context_builder.py    # Format chunks into context string
│   ├── db/
│   │   └── qdrant_client.py       # Qdrant connection management
│   └── models/
│       └── schemas.py             # Pydantic models for API requests/responses
│
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py        # Test chunking edge cases
│   │   ├── test_embedder.py       # Test embedding API mocking
│   │   └── test_context_builder.py # Test formatting logic
│   └── integration/
│       ├── test_ingest_endpoint.py # Test /ingest with real Qdrant
│       ├── test_query_endpoint.py  # Test /query with real Qdrant
│       └── test_health_endpoint.py # Test /health connectivity
│
├── .env.example                   # Template for environment variables
├── requirements.txt               # Python dependencies
├── Dockerfile                     # Docker image definition
├── docker-compose.yml             # Optional: Qdrant + backend locally
├── README.md                      # Setup, usage, API documentation
└── pytest.ini                     # Pytest configuration
```

**Structure Decision**: Single backend project (Option 1). No frontend in Spec 1. Backend uses standard FastAPI structure: `app/` for source, `tests/` for tests. Services layer separates concerns (chunking, embedding, retrieval).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |

**No violations identified. Spec 1 is intentionally minimal.**

---

## Phase 0: Research & Decision Log

**Status**: To be generated by `/sp.plan` command

**File**: `research.md`

**Research Tasks**:
1. **Chunking Strategy**: Determine optimal chunk size (500-1000 chars) and overlap (100-200 chars) for textbook content
2. **Embedding Provider**: Compare Cohere vs OpenAI embedding quality, cost, and latency
3. **Qdrant Collection Schema**: Define metadata structure (chapter, section, page, chunk_id)
4. **Error Handling Patterns**: FastAPI exception handling for Qdrant connection failures, embedding API rate limits
5. **Testing Strategy**: Integration test setup with pytest-asyncio and httpx for FastAPI

**Decisions to Document**:
- Cohere vs OpenAI: Which to use as default? (Cost, quality, rate limits)
- Chunk overlap: 100 chars or 200 chars? (Balance between redundancy and context preservation)
- Qdrant indexing: HNSW parameters (ef_construct, M) for speed vs accuracy tradeoff

---

## Phase 1: Design & Contracts

**Status**: To be generated by `/sp.plan` command

**Files**:
- `data-model.md`: Qdrant collection schema, Pydantic models
- `contracts/ingest.json`: OpenAPI schema for POST /ingest
- `contracts/query.json`: OpenAPI schema for POST /query
- `contracts/health.json`: OpenAPI schema for GET /health
- `quickstart.md`: Local setup guide (install deps, run server, test endpoints)

**Key Artifacts**:

### Data Model (Preview)

**Qdrant Collection**: `documentation_chunks`
```json
{
  "id": "uuid",
  "vector": [768 or 1024 floats],
  "payload": {
    "chunk_text": "string",
    "chapter": "integer",
    "section": "string",
    "page": "integer",
    "doc_id": "string",
    "chunk_index": "integer"
  }
}
```

**Pydantic Models**:
- `IngestRequest`: {content: str, metadata: {title, chapter, section, page}}
- `IngestResponse`: {status: str, doc_id: str, chunks_created: int}
- `QueryRequest`: {question: str, top_k: int}
- `QueryResponse`: {chunks: List[ChunkResult], metadata: {...}}

### API Contracts (Preview)

**POST /ingest**:
```yaml
Request:
  content: "string (Markdown or plain text)"
  metadata:
    title: "string"
    chapter: integer
    section: "string (optional)"
    page: integer

Response:
  status: "success"
  doc_id: "uuid"
  chunks_created: integer
```

**POST /query**:
```yaml
Request:
  question: "string"
  top_k: integer (default: 5)

Response:
  chunks:
    - chunk_text: "string"
      chapter: integer
      section: "string"
      page: integer
      score: float
  metadata:
    query_embedding_dim: integer
    search_latency_ms: integer
```

**GET /health**:
```yaml
Response:
  status: "ok" | "degraded" | "error"
  qdrant_connected: boolean
  embedding_api_available: boolean
```

---

## Agent Context Update

**Status**: To be executed after Phase 1

**Script**: `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`

**New Technologies to Add**:
- FastAPI (web framework)
- Qdrant (vector database)
- Cohere SDK or OpenAI SDK (embeddings)
- Pydantic (data validation)
- pytest-asyncio (async testing)

**Preserve**: Existing agent context between markers (manual additions)

---

## Success Metrics

Spec 1 is **COMPLETE** when:

1. ✅ All API endpoints (`/ingest`, `/query`, `/health`) return correct responses
2. ✅ Chunking produces 500-1000 char chunks with overlap
3. ✅ Embeddings stored in Qdrant with metadata
4. ✅ Search returns top-5 chunks ranked by cosine similarity
5. ✅ All tests pass (unit + integration)
6. ✅ Docker image builds and runs
7. ✅ README documents setup and API usage
8. ✅ Constitution check passes (re-run after Phase 1)

---

## Next Steps

**After `/sp.plan` completes**:
1. Run `/sp.tasks` to generate `tasks.md` (implementation checklist)
2. Implement tasks in order (TDD: red → green → refactor)
3. Run tests continuously (`pytest --watch`)
4. Deploy to Docker and verify health check
5. Create PR with Spec 1 completion evidence

**Blocked Until**:
- Phase 0 research resolves Cohere vs OpenAI decision
- Phase 1 design defines exact Qdrant schema
- All NEEDS CLARIFICATION items addressed

---

**Plan Status**: Ready for Phase 0 (Research)
**Constitution Status**: ✅ PASS (No violations)
**Next Command**: Execute research phase (automated by `/sp.plan`)
