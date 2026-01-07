# Spec 1: Core RAG Backend Foundation

**Created**: 2025-12-25
**Status**: Planning
**Branch**: refactor-deployment-parity-fix

## 1. Purpose

**What core problem Spec 1 solves:**

Spec 1 establishes the foundational capability to ingest textbook content, store it in a vector database, and retrieve relevant passages in response to user queries. This is the minimum viable RAG (Retrieval-Augmented Generation) system that enables all future specifications.

**Why this spec must exist before others:**

- **Spec 2 (Agents)** requires retrievable context → depends on Spec 1's retrieval system
- **Spec 3 (Advanced RAG)** requires a working pipeline → depends on Spec 1's ingestion and search
- **Spec 4 (Frontend Integration)** requires a query endpoint → depends on Spec 1's API
- **Spec 5 (Token Efficiency)** requires a context builder → depends on Spec 1's context formatting

**What capability the system gains once Spec 1 is complete:**

✅ Accept textbook content (Markdown/PDF)
✅ Chunk and embed content into vector representations
✅ Store embeddings in Qdrant vector database
✅ Accept user questions via HTTP API
✅ Retrieve semantically relevant chunks
✅ Return formatted context to enable answer generation

**This is the foundation: No agents, no LLMs, no frontend integration—just reliable ingestion and retrieval.**

---

## 2. Scope

### Included in Spec 1:

**Backend API** (FastAPI):
- ✅ `/ingest` endpoint: Accept content + metadata → chunk → embed → store in Qdrant
- ✅ `/query` endpoint: Accept question → embed → search Qdrant → return chunks
- ✅ `/health` endpoint: Verify Qdrant connection status
- ✅ CORS configuration for future frontend integration

**Data Pipeline**:
- ✅ Text chunking (configurable chunk size, overlap)
- ✅ Embedding generation (Cohere or OpenAI)
- ✅ Vector storage (Qdrant collection with metadata)
- ✅ Semantic search (cosine similarity, top-k retrieval)

**Context Building**:
- ✅ Format retrieved chunks with metadata (chapter, section, page)
- ✅ Concatenate chunks into structured context string
- ✅ No deduplication, no token limits (deferred to future specs)

**Configuration**:
- ✅ `.env` file for API keys (Qdrant, Cohere/OpenAI)
- ✅ Pydantic settings for type-safe config
- ✅ Docker support for containerized deployment

**Testing**:
- ✅ Unit tests for chunking logic
- ✅ Integration tests for `/ingest` and `/query` endpoints
- ✅ Health check verification

### Excluded from Spec 1 (Future Specs):

❌ **LLM Integration** → Spec 2 (Agents handle answer generation)
❌ **Multi-Agent Architecture** → Spec 3 (Advanced RAG patterns)
❌ **Frontend UI** → Spec 4 (React/Docusaurus integration)
❌ **Token Efficiency** → Spec 5 (Deduplication, truncation, compression)
❌ **User Authentication** → Future spec (not required for MVP)
❌ **Rate Limiting** → Future spec (optimization, not foundation)
❌ **Caching** → Future spec (performance enhancement)
❌ **Multi-Language Support** → Future spec (localization)

---

## 3. High-Level Architecture

### Components

**1. FastAPI Application** (`app/main.py`)
- **Responsibility**: HTTP server, route registration, CORS middleware
- **Technology**: FastAPI, Uvicorn
- **Interfaces**: Exposes REST API endpoints

**2. Ingestion Pipeline** (`app/services/ingest.py`)
- **Responsibility**: Chunk text, generate embeddings, store in Qdrant
- **Technology**: Python text processing, Cohere/OpenAI API, Qdrant client
- **Flow**:
  ```
  Content → Chunk (configurable size) → Embed → Store in Qdrant
  ```

**3. Retrieval Service** (`app/services/retriever.py`)
- **Responsibility**: Embed query, search Qdrant, return ranked chunks
- **Technology**: Cohere/OpenAI embedding API, Qdrant vector search
- **Flow**:
  ```
  Question → Embed → Search Qdrant → Return top-k chunks
  ```

**4. Context Builder** (`app/services/context_builder.py`)
- **Responsibility**: Format chunks into structured context string
- **Technology**: Python string formatting
- **Output**:
  ```
  Chunk 1: [text]
  Chapter: X, Section: Y, Page: Z

  Chunk 2: [text]
  Chapter: A, Section: B, Page: C
  ```

**5. Qdrant Client** (`app/db/qdrant_client.py`)
- **Responsibility**: Manage Qdrant connection, collection operations
- **Technology**: qdrant-client library
- **Operations**: Create collection, insert vectors, query vectors

**6. Configuration** (`app/config.py`)
- **Responsibility**: Load environment variables, validate settings
- **Technology**: Pydantic Settings
- **Variables**: QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY, OPENAI_API_KEY

### Data Flow

**Ingestion**:
```
HTTP POST /ingest
   ↓
{content, metadata}
   ↓
Chunking Service (500-1000 chars, 100 char overlap)
   ↓
Embedding Service (Cohere/OpenAI → 768/1024-dim vectors)
   ↓
Qdrant Storage (collection: documentation_chunks)
   ↓
Response: {status, doc_id, chunks_created}
```

**Query**:
```
HTTP POST /query
   ↓
{question}
   ↓
Embedding Service (question → 768/1024-dim vector)
   ↓
Qdrant Search (cosine similarity, top_k=5)
   ↓
Context Builder (format chunks with metadata)
   ↓
Response: {chunks: [...], metadata: {...}}
```

---

## 4. Success Criteria (Non-Negotiable)

Spec 1 is **COMPLETE** when ALL of the following are TRUE:

### API Functionality:
- [ ] `POST /ingest` accepts content + metadata and returns success status
- [ ] `POST /query` accepts a question and returns relevant chunks
- [ ] `GET /health` returns Qdrant connection status
- [ ] All endpoints return proper HTTP status codes (200, 400, 500)
- [ ] CORS is configured to allow `localhost:3000` (for future frontend)

### Data Pipeline:
- [ ] Chunking produces 500-1000 character chunks with 100 char overlap
- [ ] Embeddings are generated using Cohere or OpenAI (configurable)
- [ ] Embeddings are stored in Qdrant with metadata (chapter, section, page)
- [ ] Search returns top-5 most similar chunks by cosine similarity
- [ ] Context builder formats chunks with `Chapter: X, Section: Y, Page: Z`

### Testing:
- [ ] Unit tests pass for chunking logic (edge cases: empty text, large text)
- [ ] Integration tests pass for `/ingest` endpoint (success + error cases)
- [ ] Integration tests pass for `/query` endpoint (success + no results)
- [ ] Health check test verifies Qdrant connectivity

### Configuration:
- [ ] `.env.example` documents all required environment variables
- [ ] Pydantic Settings validates config on startup
- [ ] Missing API keys produce clear error messages (not silent failures)

### Deployment:
- [ ] Docker image builds successfully (`docker build`)
- [ ] Docker container runs and serves API on port 8000
- [ ] Qdrant connection works inside Docker (via QDRANT_URL env var)

### Documentation:
- [ ] README.md explains how to run the backend locally
- [ ] README.md documents `/ingest` and `/query` request/response schemas
- [ ] OpenAPI docs are auto-generated at `/docs` (FastAPI Swagger UI)

---

## 5. Explicit Out-of-Scope

The following are **PROHIBITED** in Spec 1 to prevent scope creep:

### Features:
❌ **LLM Answer Generation**: No calls to GPT-4, Claude, or any LLM. This is Spec 2.
❌ **Agent Orchestration**: No multi-agent system, no query routing, no synthesis agents. This is Spec 3.
❌ **Frontend UI**: No React components, no Docusaurus integration. This is Spec 4.
❌ **Token Optimization**: No deduplication, no token limits, no compression. This is Spec 5.
❌ **User Management**: No authentication, no user accounts, no sessions.
❌ **Rate Limiting**: No request throttling, no quota enforcement.
❌ **Caching**: No Redis, no in-memory caching, no response caching.

### Integrations:
❌ **Neon Postgres**: Not required for Spec 1. Qdrant stores vectors; no need for relational DB yet.
❌ **GitHub API**: No automated content fetching from GitHub.
❌ **Email/Notifications**: No email alerts, no webhooks.

### Optimizations:
❌ **Parallel Ingestion**: Ingest content sequentially. Parallelization is future work.
❌ **Streaming Responses**: Return chunks as JSON arrays, not streamed.
❌ **Vector Compression**: Use full-precision embeddings (no quantization).
❌ **Hybrid Search**: Qdrant vector search only (no BM25, no keyword fallback).

---

## 6. Dependencies & Assumptions

### Assumptions:
1. **Qdrant is externally managed**: Spec 1 assumes Qdrant is already running (cloud or local). We don't manage Qdrant deployment.
2. **Cohere/OpenAI API keys are provided**: Users must bring their own API keys.
3. **Content is in Markdown/plain text**: No PDF parsing in Spec 1 (can be added in future).
4. **English language only**: No multi-language support (future spec).
5. **Single user**: No concurrency concerns, no multi-tenancy.

### External Dependencies:
- **Qdrant**: Vector database for embeddings (required, must be accessible via QDRANT_URL)
- **Cohere or OpenAI**: Embedding API (one of the two required, configurable)
- **Python 3.11+**: Runtime environment
- **FastAPI**: Web framework
- **Docker**: For containerized deployment (optional for local dev)

### What Later Specs Can Assume:
Once Spec 1 is complete, future specs can safely assume:
- ✅ `/query` endpoint exists and returns relevant chunks
- ✅ Qdrant contains embedded textbook content
- ✅ Context builder formats chunks consistently
- ✅ API accepts HTTP requests with CORS configured
- ✅ Health checks verify system readiness

---

## 7. Definition of Done

Spec 1 is **DONE** when:

1. **All success criteria pass** (see Section 4)
2. **Constitution compliance verified**:
   - Code follows PEP 8 (Python style)
   - All code examples are tested (chunking, embedding, retrieval)
   - Documentation is complete (README, OpenAPI docs)
3. **No blocked dependencies**: Qdrant connection works, embedding API responds
4. **Deployment verified**: Docker image runs and serves API
5. **Tests green**: All unit + integration tests pass
6. **Review approved**: Code review confirms adherence to spec

---

## 8. Risk Mitigation

### Known Risks:

**Risk 1: Qdrant Connection Failure**
- **Mitigation**: Health check endpoint detects failures early. Provide clear error messages.

**Risk 2: Embedding API Rate Limits**
- **Mitigation**: Use exponential backoff for retries. Document rate limit errors in logs.

**Risk 3: Large Content Ingestion**
- **Mitigation**: Chunking prevents oversized payloads. Log chunk counts for visibility.

**Risk 4: Search Returns No Results**
- **Mitigation**: Return empty array with 200 status (not 404). Document expected behavior.

---

## Non-Goals (Explicit)

These are intentionally **NOT** goals for Spec 1:

- **Performance optimization** (future: caching, parallel ingestion)
- **Advanced search features** (future: hybrid search, reranking)
- **Multi-modal support** (future: images, audio)
- **Real-time updates** (future: websockets, streaming)
- **Analytics/telemetry** (future: query logging, usage metrics)

---

**This spec defines the minimum viable RAG backend foundation. All enhancements belong to future specs.**
