# Tasks: RAG Chatbot Backend

**Input**: Design documents from `/specs/`
**Prerequisites**: plan.md (✅), sp.specification.md (✅), agent.system.md (✅)

**Tests**: Tests are included as specified in the plan (Unit, Integration, Constitution Compliance)

**Organization**: Tasks are grouped by development phase following the spec-driven implementation plan

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which feature/phase this task belongs to (e.g., SETUP, RAG, AGENT, TEST)
- Include exact file paths in descriptions

## Path Conventions

- Backend project: `app/`, `scripts/`, `tests/` at repository root
- Specs: `specs/` at repository root

---

## Phase 1: Setup (Foundation & Infrastructure)

**Purpose**: Project initialization, database setup, and basic connectivity

**⚠️ CRITICAL**: Must complete before ANY feature implementation

- [ ] T001 Create project directory structure: `app/{api,agents,db,services,models,utils}`, `scripts/`, `tests/{unit,integration,constitution}`
- [ ] T002 [P] Configure Python environment with pyproject.toml dependencies (FastAPI, OpenAI Agents SDK, qdrant-client, asyncpg)
- [ ] T003 [P] Create .env.example with all required environment variables (LLM keys, Qdrant, Neon, OpenAI)
- [ ] T004 [P] Create .gitignore with Python, IDE, secrets, and logs exclusions
- [ ] T005 Create app/config.py with Pydantic Settings for environment variable management
- [ ] T006 Create app/db/neon_client.py with asyncpg connection pool (min=2, max=10)
- [ ] T007 [P] Create app/db/qdrant_client.py with Qdrant client initialization and health_check method
- [ ] T008 Create scripts/setup_neon.py to initialize Neon schema (documents, chunks, query_logs tables with indexes)
- [ ] T009 [P] Create scripts/setup_qdrant.py to create book_chunks collection (768-dim, Cosine, HNSW config)
- [ ] T010 Execute scripts/setup_neon.py to deploy schema to Neon Postgres
- [ ] T011 Execute scripts/setup_qdrant.py to create collection in Qdrant Cloud
- [ ] T012 Create app/api/health.py with GET /health endpoint checking Neon, Qdrant, LLM connectivity
- [ ] T013 Create app/main.py as FastAPI app entry point, include health router
- [ ] T014 Verify GET /health returns 200 OK with all services "connected"

**Checkpoint**: Foundation ready - all external services accessible, basic API running

---

## Phase 2: Retrieval Pipeline (Ingestion & Search)

**Purpose**: Implement content ingestion, embedding, chunking, and vector search

**User Story 3 (P3)**: As an admin, I can ingest book content into the system

### Core Services

- [ ] T015 [P] Create app/models/schemas.py with Pydantic models (QueryRequest, QueryResponse, IngestRequest, IngestResponse, Citation, DocumentMetadata, EmbedRequest, EmbedResponse)
- [ ] T016 [P] Create app/services/embedder.py with EmbeddingService (embed_text, embed_batch methods using text-embedding-3-small)
- [ ] T017 [P] Create app/services/chunker.py with chunk_text function (512 tokens, 50 overlap, using tiktoken)
- [ ] T018 Create app/services/retriever.py with RetrieverService.search method (Qdrant search with chapter/section filters, return top-k chunks with metadata)

### Ingestion Endpoint

- [ ] T019 Create app/api/ingest.py with POST /ingest endpoint (parse → chunk → store in Neon → embed → store in Qdrant)
- [ ] T020 Implement content parsing in app/api/ingest.py (extract plain text from Markdown)
- [ ] T021 Implement document storage in app/api/ingest.py (insert into Neon documents table)
- [ ] T022 Implement chunk storage in app/api/ingest.py (insert into Neon chunks table with metadata)
- [ ] T023 Implement embedding generation in app/api/ingest.py (batch embed chunks)
- [ ] T024 Implement vector storage in app/api/ingest.py (upsert to Qdrant with payload)

### Re-embedding Endpoint

- [ ] T025 Create app/api/embed.py with POST /embed endpoint (fetch chunks by doc_id → re-embed → update Qdrant)
- [ ] T026 Implement chunk fetching in app/api/embed.py (query Neon by doc_id)
- [ ] T027 Implement embedding update in app/api/embed.py (re-embed and upsert to Qdrant)

### Testing & Validation

- [ ] T028 [P] Write unit test tests/unit/test_chunker.py (test 512-token chunking, overlap, boundary cases)
- [ ] T029 [P] Write unit test tests/unit/test_embedder.py (test single/batch embedding with mocked OpenAI API)
- [ ] T030 Test POST /ingest with sample chapter (500 tokens → 2 chunks)
- [ ] T031 Verify ingestion: Check Neon contains 1 document and 2 chunks
- [ ] T032 Verify ingestion: Check Qdrant contains 2 vectors with correct metadata
- [ ] T033 Test POST /embed re-embeds existing document successfully

**Checkpoint**: Ingestion pipeline complete - can ingest book content and search vectors

---

## Phase 3: RAG Agent & Query Endpoint (Main RAG Logic)

**Purpose**: Implement dual-mode query system with constitution-bound agent

**User Story 1 (P1 - MVP)**: As a student, I can ask questions and get grounded answers from book content (Normal RAG Mode)
**User Story 2 (P2)**: As a student, I can ask questions about selected text and get answers only from that text (Selected-Text-Only Mode)

### Agent Configuration

- [ ] T034 Verify specs/agent.system.md contains complete system instructions (constitution-bound rules)
- [ ] T035 Create app/agents/rag_agent.py with Agent initialization (load specs/agent.system.md, configure OpenAIChatCompletionsModel)
- [ ] T036 Configure LLM client in app/agents/rag_agent.py (AsyncOpenAI with Gemini/Claude via OpenAI-compatible interface)
- [ ] T037 Create app/agents/runner.py with run_rag_query function (invoke Runner.run with agent and formatted prompt)

### Query Pipeline Services

- [ ] T038 [P] Create app/services/mode_detector.py with detect_mode function (return "selected_text_only" if selected_text present, else "normal_rag")
- [ ] T039 [P] Create app/services/context_builder.py with build_context_normal_rag and build_context_selected_text functions

### Query Endpoint

- [ ] T040 Create app/api/query.py with POST /query endpoint orchestration
- [ ] T041 Implement mode detection in app/api/query.py (call detect_mode)
- [ ] T042 Implement normal_rag flow in app/api/query.py (embed query → search Qdrant → build context)
- [ ] T043 Implement selected_text_only flow in app/api/query.py (skip Qdrant → build context from selected_text)
- [ ] T044 Implement agent invocation in app/api/query.py (call run_rag_query with context)
- [ ] T045 Implement response building in app/api/query.py (format answer with citations and metadata)
- [ ] T046 Implement query logging in app/api/query.py (insert into Neon query_logs table)

### Chapters Metadata Endpoint

- [ ] T047 Create app/api/chapters.py with GET /chapters endpoint (fetch distinct chapters and sections from Neon)
- [ ] T048 Implement SQL query in app/api/chapters.py (GROUP BY chapter with ARRAY_AGG for sections)

### Testing & Validation

- [ ] T049 [P] Write unit test tests/unit/test_mode_detector.py (test normal_rag and selected_text_only detection)
- [ ] T050 [P] Write unit test tests/unit/test_context_builder.py (test context formatting for both modes)
- [ ] T051 Test POST /query in normal_rag mode (verify answer with citations from Qdrant)
- [ ] T052 Test POST /query in selected_text_only mode (verify answer uses only selected text, no Qdrant call)
- [ ] T053 Test GET /chapters returns list of chapters with sections

**Checkpoint**: Core RAG functionality complete - both query modes working

---

## Phase 4: Testing & Hardening (Quality & Reliability)

**Purpose**: Comprehensive testing, error handling, and constitution compliance

### Error Handling & Resilience

- [ ] T054 [P] Create app/utils/logging.py with StructuredJSONLogger (timestamp, level, message, context)
- [ ] T055 [P] Create app/utils/retry.py with retry_with_exponential_backoff decorator (max 3 retries, exponential backoff)
- [ ] T056 Add error handling to app/api/query.py (ValidationError → 400, ConnectionError → 503, TimeoutError → 504, UnexpectedResponse → 500)
- [ ] T057 Add error handling to app/api/ingest.py (graceful failures with appropriate HTTP status codes)
- [ ] T058 Add retry decorator to LLM calls in app/agents/runner.py
- [ ] T059 Add structured logging to all endpoints (query, ingest, embed, health)
- [ ] T060 Implement error metadata specification from spec section 5.2.3 (latency breakdown, debug mode support)

### Integration Tests

- [ ] T061 [P] Create tests/conftest.py with pytest fixtures (test database, mock clients)
- [ ] T062 [P] Write integration test tests/integration/test_ingest_endpoint.py (end-to-end ingestion with mock/real DBs)
- [ ] T063 [P] Write integration test tests/integration/test_query_endpoint.py (test normal_rag and selected_text_only flows)
- [ ] T064 Write integration test tests/integration/test_rag_pipeline.py (test full pipeline: embed → search → agent → response)

### Constitution Compliance Tests

- [ ] T065 Create tests/constitution/test_constitution_compliance.py test file
- [ ] T066 [P] Write test_principle_i_retrieval_first: Agent refuses to answer without context (zero-shot blocked)
- [ ] T067 [P] Write test_principle_ii_source_bounded: Agent does not introduce external facts
- [ ] T068 [P] Write test_principle_iii_selected_text_strict: Agent uses ONLY selected text, no Qdrant mixing
- [ ] T069 [P] Write test_principle_iv_citation_traceability: Response includes chunk IDs and metadata
- [ ] T070 [P] Write test_principle_vi_safety: Agent refuses out-of-scope questions
- [ ] T071 Run all constitution tests and verify 100% pass rate

### Code Quality & Coverage

- [ ] T072 Add Pydantic validators to all request models in app/models/schemas.py (min/max lengths, value ranges)
- [ ] T073 Run pytest with coverage: `pytest --cov=app --cov-report=html`
- [ ] T074 Verify test coverage ≥ 80%
- [ ] T075 Run type checking with mypy: `mypy app/`
- [ ] T076 Run linting with ruff: `ruff check app/ tests/`
- [ ] T077 Run formatting with black: `black app/ tests/`

**Checkpoint**: All tests passing, error handling robust, constitution compliance verified

---

## Phase 5: Documentation & Deployment (Production Readiness)

**Purpose**: Complete documentation, deployment configuration, and performance validation

### Documentation

- [ ] T078 Write comprehensive README.md (project overview, setup, API endpoints, deployment guide)
- [ ] T079 [P] Create .dockerignore with appropriate patterns (node_modules, .git, .env, logs, cache)
- [ ] T080 [P] Verify FastAPI auto-generates OpenAPI docs at /docs endpoint
- [ ] T081 [P] Write specs/data-model.md with finalized schemas and ER diagram
- [ ] T082 [P] Write specs/contracts/query.contract.md with request/response schemas for both modes
- [ ] T083 [P] Write specs/contracts/ingest.contract.md with ingestion API contract
- [ ] T084 [P] Write specs/contracts/embed.contract.md with re-embedding API contract

### Deployment Scripts

- [ ] T085 [P] Create scripts/start.sh startup script (load .env, run uvicorn)
- [ ] T086 [P] Create Dockerfile for containerization (Python 3.11-slim, install deps, copy source, expose 8000)
- [ ] T087 [P] Create scripts/ingest_book.py bulk ingestion script (read Markdown files, call POST /ingest)
- [ ] T088 [P] Create scripts/test_connection.py to verify Neon + Qdrant connectivity

### Cloud Deployment

- [ ] T089 Choose cloud platform (Render/Railway/Fly.io) based on requirements
- [ ] T090 Configure deployment (set environment variables: LLM keys, Qdrant, Neon, OpenAI)
- [ ] T091 Deploy backend to chosen cloud platform via GitHub integration or CLI
- [ ] T092 Verify /health endpoint returns 200 OK in production
- [ ] T093 Verify /docs OpenAPI UI accessible in production
- [ ] T094 Test POST /query endpoint from production URL

### Performance Testing

- [ ] T095 Run performance benchmarks using locust or ab (measure p50, p95, p99 latency)
- [ ] T096 Document performance results in specs/performance.md (query latency, embedding time, retrieval time)
- [ ] T097 Verify query latency < 3s (p95) meets spec requirement
- [ ] T098 Optimize if needed (cache embeddings, batch Qdrant searches, tune chunk size)

**Checkpoint**: Backend deployed, documented, and performance-validated

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and production hardening

- [ ] T099 [P] Add rate limiting to /query endpoint (10 requests/minute per IP)
- [ ] T100 [P] Add API key authentication to /ingest and /embed endpoints (admin-only)
- [ ] T101 [P] Create database backup strategy documentation
- [ ] T102 Implement X-Debug-Mode header support for extended metadata (latency breakdown, tokens used)
- [ ] T103 [P] Add monitoring/alerting setup documentation (logs, metrics, traces)
- [ ] T104 Run final end-to-end validation with frontend Docusaurus integration
- [ ] T105 Create runbook documentation for common operational tasks

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Retrieval Pipeline (Phase 2)**: Depends on Setup (Phase 1) completion
- **RAG Agent (Phase 3)**: Depends on Retrieval Pipeline (Phase 2) completion
- **Testing & Hardening (Phase 4)**: Depends on RAG Agent (Phase 3) completion
- **Documentation & Deployment (Phase 5)**: Depends on Testing (Phase 4) completion
- **Polish (Phase 6)**: Depends on Deployment (Phase 5) completion

### Within Each Phase

**Phase 1 (Setup)**:
- T001 → T002-T004 [P] → T005 → T006, T007 [P] → T008, T009 [P] → T010 → T011 → T012 → T013 → T014

**Phase 2 (Retrieval)**:
- T015-T018 [P] → T019 → T020-T024 (sequential in ingest) → T025-T027 (sequential in embed) → T028-T033 (validation)

**Phase 3 (RAG Agent)**:
- T034 → T035-T037 (sequential for agent) → T038-T039 [P] → T040 → T041-T046 (sequential orchestration) → T047-T048 → T049-T053 [P tests]

**Phase 4 (Testing)**:
- T054-T055 [P] → T056-T060 → T061-T064 [P] → T065 → T066-T071 [P] → T072-T077

**Phase 5 (Deployment)**:
- T078-T088 [P docs & scripts] → T089 → T090-T094 (sequential deployment) → T095-T098 (performance)

**Phase 6 (Polish)**:
- T099-T105 [P] (all can run in parallel)

### Parallel Opportunities

- **Phase 1**: T002-T004, T007, T009 can run in parallel
- **Phase 2**: T015-T018 can run in parallel; T028-T029 tests can run in parallel
- **Phase 3**: T038-T039, T049-T050 can run in parallel
- **Phase 4**: T054-T055, T061-T064, T066-T070 can run in parallel
- **Phase 5**: T078-T088 documentation and scripts can run in parallel
- **Phase 6**: All T099-T105 can run in parallel

---

## Implementation Strategy

### MVP First (Critical Path)

1. **Phase 1: Setup** (T001-T014) - Foundation
2. **Phase 2: Retrieval** (T015-T033) - Can ingest content
3. **Phase 3: RAG Agent** (T034-T053) - Can query content
4. **VALIDATE MVP**: Test normal_rag mode with sample queries
5. **Phase 4: Testing** (T054-T077) - Production-ready quality
6. **Phase 5: Deploy** (T078-T098) - Live in production

**MVP Scope**: Phases 1-3 deliver functional RAG chatbot. Phases 4-5 make it production-ready.

### Incremental Delivery

1. After Phase 1 → Foundation ready for development
2. After Phase 2 → Can ingest book content, not yet queryable
3. After Phase 3 → **MVP ready** - can query and get answers
4. After Phase 4 → Production-quality with full testing
5. After Phase 5 → Deployed and documented
6. After Phase 6 → Hardened with rate limiting, auth, monitoring

### Constitution Validation Checkpoints

- After T053 (Phase 3 complete): Verify mode detection works correctly
- After T071 (Phase 4): Verify all 6 constitution principles enforced
- After T094 (Production deployed): Smoke test constitution compliance in production

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- Tasks T001-T014 are CRITICAL blocking tasks - must complete before Phase 2
- Constitution tests (T065-T071) are NON-NEGOTIABLE - must achieve 100% pass rate
- Test coverage target: ≥ 80% (T074)
- Performance target: Query latency < 3s p95 (T097)
- Security: Secrets only via .env (T003), never hardcoded
- Deployment: Choose one cloud platform (Render/Railway/Fly.io) at T089
- MVP delivery: Phases 1-3 (T001-T053) deliver minimal viable product
