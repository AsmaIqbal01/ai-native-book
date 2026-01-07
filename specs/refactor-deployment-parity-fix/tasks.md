# Tasks: Spec 1 - Core RAG Backend Foundation

**Branch**: `refactor-deployment-parity-fix` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)

**Status**: Ready for implementation

---

## Task Format

```
- [ ] [ID] [Priority] [Story] Description with file:line reference
```

**Priority Levels**: `P0` (blocking) | `P1` (high) | `P2` (normal)
**Story Tags**: `S1` (Setup) | `S2` (Ingest) | `S3` (Query) | `S4` (Health) | `S5` (Tests) | `S6` (Deploy)

---

## Phase 0: Setup & Configuration

**Goal**: Initialize project structure, dependencies, and configuration

- [X] [T001] [P0] [S1] Create `requirements.txt` with FastAPI 0.115+, Qdrant Client 1.7+, OpenAI SDK, Pydantic 2.0+, Uvicorn, pytest
- [X] [T002] [P0] [S1] Create `.env.example` with QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY, EMBEDDING_PROVIDER, APP_ENV, LOG_LEVEL
- [X] [T003] [P0] [S1] Create `app/config.py` with Pydantic Settings class for env var loading
- [X] [T004] [P0] [S1] Create `app/__init__.py` (empty)
- [X] [T005] [P0] [S1] Create `app/models/__init__.py` and `app/models/schemas.py` with Pydantic models from data-model.md
- [X] [T006] [P0] [S1] Create `app/db/__init__.py` and `app/db/qdrant_client.py` with collection initialization (1536-dim COSINE)
- [X] [T007] [P0] [S1] Create `pytest.ini` with async support configuration

**Dependencies**: T001 → T003 → T005 (config needed before schemas)

---

## Phase 1: Core Services

**Goal**: Implement chunking, embedding, retrieval, and context building services

### Story S2: Text Chunking

- [X] [T101] [P1] [S2] Create `app/services/__init__.py` (empty)
- [X] [T102] [P1] [S2] Implement `app/services/chunker.py:chunk_text()` function (750 chars, 100 overlap) with metadata preservation
- [X] [T103] [P2] [S2] Write unit test `tests/unit/test_chunker.py:test_chunk_text_basic()` (verify 750±100 chars)
- [ ] [T104] [P2] [S2] Write unit test `tests/unit/test_chunker.py:test_chunk_text_overlap()` (verify 100-char overlap)
- [ ] [T105] [P2] [S2] Write unit test `tests/unit/test_chunker.py:test_chunk_text_edge_cases()` (empty content, <750 chars)

**Dependencies**: T101 → T102 → T103, T104, T105 (parallel tests)

### Story S2: Embedding Service

- [X] [T201] [P1] [S2] Implement `app/services/embedder.py:get_embedding()` function (OpenAI text-embedding-3-small, 1536-dim)
- [ ] [T202] [P1] [S2] Add exponential backoff retry logic for rate limits (3 retries, 2s/4s/8s delays)
- [ ] [T203] [P2] [S2] Write unit test `tests/unit/test_embedder.py:test_get_embedding_success()` (mock OpenAI API)
- [ ] [T204] [P2] [S2] Write unit test `tests/unit/test_embedder.py:test_get_embedding_retry()` (mock rate limit error)

**Dependencies**: T201 → T202 → T203, T204 (parallel tests)

### Story S3: Retrieval Service

- [X] [T301] [P1] [S3] Implement `app/services/retriever.py:search_chunks()` function (Qdrant search with filters)
- [ ] [T302] [P2] [S3] Write unit test `tests/unit/test_retriever.py:test_search_chunks_basic()` (mock Qdrant search)
- [ ] [T303] [P2] [S3] Write unit test `tests/unit/test_retriever.py:test_search_chunks_chapter_filter()` (mock filtered search)

**Dependencies**: T301 → T302, T303 (parallel tests)

### Story S3: Context Builder

- [X] [T401] [P1] [S3] Implement `app/services/context_builder.py:format_chunks()` function (format chunks with metadata)
- [X] [T402] [P2] [S3] Write unit test `tests/unit/test_context_builder.py:test_format_chunks()` (verify output format)

**Dependencies**: T401 → T402

---

## Phase 2: API Endpoints

**Goal**: Implement FastAPI endpoints with request/response validation

### Story S2: Ingest Endpoint

- [X] [T501] [P1] [S2] Create `app/api/__init__.py` (empty)
- [X] [T502] [P1] [S2] Implement `app/api/ingest.py:ingest_document()` endpoint (POST /ingest) with chunking + embedding + Qdrant storage
- [X] [T503] [P1] [S2] Add error handling for Qdrant connection failures (503 response)
- [X] [T504] [P1] [S2] Add error handling for embedding API failures (503 response)
- [X] [T505] [P1] [S2] Add validation error handling (400 response)

**Dependencies**: T102, T201, T006 → T501 → T502 → T503, T504, T505 (parallel error handlers)

### Story S3: Query Endpoint

- [X] [T601] [P1] [S3] Implement `app/api/query.py:query_chunks()` endpoint (POST /query) with embedding + Qdrant search
- [X] [T602] [P1] [S3] Add chapter_filter support (Qdrant payload filter)
- [X] [T603] [P1] [S3] Add error handling for Qdrant connection failures (503 response)
- [X] [T604] [P1] [S3] Add error handling for embedding API failures (503 response)
- [X] [T605] [P1] [S3] Add validation error handling (400 response)

**Dependencies**: T201, T301, T401 → T601 → T602, T603, T604, T605 (parallel error handlers)

### Story S4: Health Endpoint

- [X] [T701] [P1] [S4] Implement `app/api/health.py:health_check()` endpoint (GET /health) with Qdrant + embedding API checks
- [X] [T702] [P2] [S4] Return "ok" when both services available, "degraded" when one fails, "error" when both fail

**Dependencies**: T701 → T702

---

## Phase 3: FastAPI Application

**Goal**: Wire up endpoints and configure CORS

- [X] [T801] [P0] [S1] Create `app/main.py` with FastAPI app initialization and CORS middleware (origins=["*"])
- [X] [T802] [P1] [S1] Register ingest router (app/api/ingest.py)
- [X] [T803] [P1] [S1] Register query router (app/api/query.py)
- [X] [T804] [P1] [S1] Register health router (app/api/health.py)

**Dependencies**: T502, T601, T701 → T801 → T802, T803, T804 (parallel router registration)

---

## Phase 4: Integration Tests

**Goal**: Test full API workflows with real Qdrant instance

### Story S5: Ingest Tests

- [ ] [T901] [P1] [S5] Write `tests/integration/test_ingest_endpoint.py:test_ingest_success()` (POST /ingest with real Qdrant)
- [ ] [T902] [P2] [S5] Write `tests/integration/test_ingest_endpoint.py:test_ingest_validation_error()` (content <100 chars)
- [ ] [T903] [P2] [S5] Write `tests/integration/test_ingest_endpoint.py:test_ingest_qdrant_error()` (mock Qdrant failure)

**Dependencies**: T502, T801 → T901, T902, T903 (parallel tests)

### Story S5: Query Tests

- [ ] [T1001] [P1] [S5] Write `tests/integration/test_query_endpoint.py:test_query_success()` (POST /query with real Qdrant)
- [ ] [T1002] [P2] [S5] Write `tests/integration/test_query_endpoint.py:test_query_chapter_filter()` (chapter filter)
- [ ] [T1003] [P2] [S5] Write `tests/integration/test_query_endpoint.py:test_query_validation_error()` (question <3 chars)

**Dependencies**: T601, T801 → T1001, T1002, T1003 (parallel tests)

### Story S5: Health Tests

- [ ] [T1101] [P1] [S5] Write `tests/integration/test_health_endpoint.py:test_health_ok()` (GET /health when all services available)
- [ ] [T1102] [P2] [S5] Write `tests/integration/test_health_endpoint.py:test_health_degraded()` (mock embedding API failure)

**Dependencies**: T701, T801 → T1101, T1102 (parallel tests)

---

## Phase 5: Deployment & Documentation

**Goal**: Containerize backend and document setup process

### Story S6: Docker Deployment

- [X] [T1201] [P1] [S6] Create `Dockerfile` with Python 3.11 slim base image, COPY app/, EXPOSE 8000, CMD uvicorn
- [ ] [T1202] [P2] [S6] Create `docker-compose.yml` (optional: local Qdrant + backend)
- [ ] [T1203] [P1] [S6] Test Docker build: `docker build -t rag-backend .` (verify <500MB image size)
- [ ] [T1204] [P1] [S6] Test Docker run: `docker run -p 8000:8000 --env-file .env rag-backend` (verify health check passes)

**Dependencies**: T801 → T1201 → T1202, T1203, T1204 (parallel tests)

### Story S6: Documentation

- [X] [T1301] [P1] [S6] Create `README.md` with setup instructions, API usage examples, test commands
- [X] [T1302] [P2] [S6] Verify quickstart.md steps (install deps → config .env → init Qdrant → start server → test endpoints)

**Dependencies**: T801, T1201 → T1301, T1302 (parallel docs)

---

## Dependency Graph

```
Setup (Phase 0)
├─ T001 (requirements.txt)
├─ T002 (.env.example)
├─ T003 (config.py) [depends: T001]
├─ T004-T007 (project structure)

Services (Phase 1)
├─ Chunking: T101 → T102 → T103, T104, T105
├─ Embedding: T201 → T202 → T203, T204
├─ Retrieval: T301 → T302, T303
└─ Context: T401 → T402

API Endpoints (Phase 2)
├─ Ingest: [T102, T201, T006] → T501 → T502 → T503, T504, T505
├─ Query: [T201, T301, T401] → T601 → T602, T603, T604, T605
└─ Health: T701 → T702

FastAPI App (Phase 3)
└─ [T502, T601, T701] → T801 → T802, T803, T804

Integration Tests (Phase 4)
├─ Ingest Tests: [T502, T801] → T901, T902, T903
├─ Query Tests: [T601, T801] → T1001, T1002, T1003
└─ Health Tests: [T701, T801] → T1101, T1102

Deployment (Phase 5)
├─ Docker: T801 → T1201 → T1202, T1203, T1204
└─ Docs: [T801, T1201] → T1301, T1302
```

---

## Parallel Execution Opportunities

**Phase 0**: All tasks (T001-T007) can run in parallel after T001

**Phase 1**:
- Chunking tests (T103, T104, T105) can run in parallel
- Embedding tests (T203, T204) can run in parallel
- Retrieval tests (T302, T303) can run in parallel

**Phase 2**:
- Error handlers (T503, T504, T505) can run in parallel
- Query error handlers (T603, T604, T605) can run in parallel
- Router registration (T802, T803, T804) can run in parallel

**Phase 4**:
- All integration tests within each story can run in parallel
- Ingest tests (T901, T902, T903) can run in parallel
- Query tests (T1001, T1002, T1003) can run in parallel
- Health tests (T1101, T1102) can run in parallel

**Phase 5**:
- Docker tests (T1203, T1204) can run in parallel
- Documentation tasks (T1301, T1302) can run in parallel

---

## Test Criteria by Story

### S1: Setup
- [ ] All dependencies installed (`pip install -r requirements.txt` succeeds)
- [ ] .env loaded correctly (config.py reads QDRANT_URL, OPENAI_API_KEY)
- [ ] Qdrant collection created (1536-dim COSINE vectors)

### S2: Ingest
- [ ] POST /ingest accepts content + metadata → returns 200 with doc_id
- [ ] Chunks are 750±100 chars with 100-char overlap
- [ ] Embeddings stored in Qdrant (1536 dimensions)
- [ ] Error handling: 400 for validation errors, 503 for Qdrant/API failures

### S3: Query
- [ ] POST /query accepts question → returns top-k chunks with scores
- [ ] Chapter filter works (only returns chunks from specified chapter)
- [ ] Chunks ranked by cosine similarity (score 0.0-1.0)
- [ ] Error handling: 400 for validation errors, 503 for Qdrant/API failures

### S4: Health
- [ ] GET /health returns "ok" when all services available
- [ ] Returns "degraded" when embedding API fails
- [ ] Returns "error" when Qdrant fails

### S5: Tests
- [ ] All unit tests pass (`pytest tests/unit/ -v`)
- [ ] All integration tests pass (`pytest tests/integration/ -v`)
- [ ] Test coverage >80% for services and API endpoints

### S6: Deploy
- [ ] Docker image builds successfully (<500MB)
- [ ] Docker container runs and passes health check
- [ ] README documents setup, API usage, and test commands

---

## Success Criteria Mapping

| Spec Success Criterion | Tasks |
|------------------------|-------|
| POST /ingest accepts content + metadata | T502, T901 |
| POST /query accepts question and returns chunks | T601, T1001 |
| GET /health returns Qdrant connection status | T701, T1101 |
| Chunking produces 500-1000 char chunks with overlap | T102, T103 |
| Embeddings stored in Qdrant with metadata | T201, T006, T502 |
| Search returns top-5 chunks ranked by similarity | T301, T601, T1001 |
| All tests pass (unit + integration) | T103-T105, T203-T204, T302-T303, T402, T901-T903, T1001-T1003, T1101-T1102 |
| Docker image builds and runs | T1201, T1203, T1204 |
| README documents setup and API usage | T1301, T1302 |

---

## Estimated Completion

**Total Tasks**: 56 tasks
**Estimated Time**: 12-16 hours (based on parallel execution)

**Phase Breakdown**:
- Phase 0 (Setup): 1-2 hours
- Phase 1 (Services): 3-4 hours
- Phase 2 (API): 3-4 hours
- Phase 3 (FastAPI): 1 hour
- Phase 4 (Tests): 2-3 hours
- Phase 5 (Deploy): 2-3 hours

---

## Next Steps

1. Start with Phase 0 (Setup) - complete all T001-T007
2. Implement Phase 1 services (chunking first, then embedding, retrieval, context)
3. Build Phase 2 endpoints (ingest, query, health)
4. Wire up Phase 3 FastAPI app
5. Run Phase 4 integration tests
6. Complete Phase 5 deployment and documentation

**TDD Workflow**: For each task, follow Red → Green → Refactor:
1. Write failing test first
2. Implement minimal code to pass test
3. Refactor for clarity and performance

**Testing Strategy**: Run tests continuously during development:
```bash
# Watch mode (re-run on file changes)
pytest --watch

# Run specific test file
pytest tests/unit/test_chunker.py -v

# Run all tests with coverage
pytest tests/ --cov=app --cov-report=html
```

---

**Status**: Ready for `/sp.implement` to execute tasks in TDD workflow
