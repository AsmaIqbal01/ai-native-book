# Phase 0: Research & Decision Log
## Spec 1 - Core RAG Backend Foundation

**Date**: 2025-12-25
**Branch**: refactor-deployment-parity-fix
**Status**: Complete

---

## Research Task 1: Chunking Strategy

**Question**: What is the optimal chunk size and overlap for textbook content?

**Research**:

**Chunk Size Considerations**:
- **Too small** (<300 chars): Loses semantic context, requires more chunks per query, increases embedding costs
- **Too large** (>1500 chars): Exceeds embedding model context windows, reduces precision in retrieval
- **Industry standard**: 500-1000 characters for dense text content

**Overlap Considerations**:
- **No overlap** (0 chars): Risk splitting concepts across chunks, poor boundary handling
- **Too much overlap** (>200 chars): Redundancy increases storage and search time
- **Industry standard**: 10-20% of chunk size (50-200 chars)

**Decision**: **750 characters with 100 character overlap**

**Rationale**:
- 750 chars ≈ 150-180 tokens (avg English word = 4 chars, 1.3 tokens/word)
- Fits well within embedding models' context windows (Cohere: 512 tokens, OpenAI: 8192 tokens)
- 100 char overlap (13%) preserves concept continuity at chunk boundaries
- Balances precision (smaller chunks) with context (larger chunks)

**Alternatives Considered**:
- 500 chars / 50 overlap: Too fragmented for educational content
- 1000 chars / 200 overlap: Higher redundancy, slower search

**Implementation**:
```python
# app/services/chunker.py
CHUNK_SIZE = 750
CHUNK_OVERLAP = 100
```

---

## Research Task 2: Embedding Provider

**Question**: Should we use Cohere or OpenAI for embeddings?

**Comparison**:

| Factor | Cohere | OpenAI |
|--------|--------|--------|
| **Vector Dimensions** | 768 (embed-multilingual-light-v3.0) | 1536 (text-embedding-3-small) or 3072 (text-embedding-3-large) |
| **Cost** | $0.10 / 1M tokens | $0.02 / 1M tokens (3-small), $0.13 / 1M tokens (3-large) |
| **Rate Limits** | 10,000 requests/min (free tier) | 3,000 requests/min (tier 1) |
| **Quality** | Optimized for retrieval tasks | General-purpose, high quality |
| **API Latency** | ~200-500ms | ~150-400ms |
| **Multilingual** | Yes (100+ languages) | Yes (via text-embedding-3) |

**Decision**: **Use OpenAI text-embedding-3-small (1536 dimensions) as default**

**Rationale**:
- **5x cheaper** than Cohere for equivalent quality
- **Higher dimensions** (1536 vs 768) → better retrieval precision
- **Lower latency** on average (150-400ms vs 200-500ms)
- **Simpler API**: OpenAI SDK is more widely used, better documented
- **Flexibility**: Can upgrade to text-embedding-3-large if needed

**Fallback**: Support Cohere as an alternative (configurable via env var) for users who prefer it or hit OpenAI rate limits.

**Implementation**:
```python
# .env
EMBEDDING_PROVIDER=openai  # or "cohere"
OPENAI_API_KEY=sk-...
COHERE_API_KEY=...  # optional

# app/services/embedder.py
if settings.embedding_provider == "openai":
    model = "text-embedding-3-small"
    dimensions = 1536
elif settings.embedding_provider == "cohere":
    model = "embed-multilingual-light-v3.0"
    dimensions = 768
```

**Cost Estimate** (for MVP):
- 10 chapters × 10k words/chapter = 100k words
- 100k words × 750 char chunks = ~133 chunks/chapter = 1330 chunks total
- 1330 chunks × 750 chars = ~250k tokens
- Cost: 250k tokens × $0.02 / 1M = **$0.005 (0.5 cents)**

**Alternatives Considered**:
- Cohere: More expensive ($0.10/1M), but good for multilingual use cases
- Local embeddings (sentence-transformers): Free but requires GPU, slower, lower quality

---

## Research Task 3: Qdrant Collection Schema

**Question**: What metadata structure should we store with each chunk?

**Research**:

**Required Metadata**:
1. **chunk_text** (string): Full text of the chunk (stored in payload, not just vector)
2. **chapter** (integer): Chapter number (for filtering queries like "Search only Chapter 3")
3. **section** (string): Section name (e.g., "Introduction to ROS2")
4. **page** (integer): Page number (for citation purposes)
5. **doc_id** (string/UUID): Original document identifier
6. **chunk_index** (integer): Position of chunk within document (0, 1, 2, ...)

**Optional Metadata** (for future):
- **title** (string): Document title
- **created_at** (timestamp): When chunk was created
- **version** (string): Document version (for updates)

**Decision**: **Use minimal schema with 6 required fields**

**Qdrant Collection Config**:
```python
# app/db/qdrant_client.py
from qdrant_client.models import VectorParams, Distance

client.create_collection(
    collection_name="documentation_chunks",
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small
        distance=Distance.COSINE  # Cosine similarity for semantic search
    )
)
```

**Payload Schema**:
```json
{
  "chunk_text": "ROS2 (Robot Operating System 2) is a set of software libraries...",
  "chapter": 3,
  "section": "Introduction to ROS2",
  "page": 42,
  "doc_id": "abc-123-def",
  "chunk_index": 5
}
```

**Rationale**:
- **chapter + section + page**: Enable precise citations (required by constitution)
- **doc_id**: Support document updates (replace all chunks with same doc_id)
- **chunk_index**: Preserve original order (useful for context reconstruction)
- **chunk_text in payload**: Qdrant only stores vectors; we need text for responses

**Alternatives Considered**:
- Store title + created_at: Deferred to future (not needed for MVP)
- Store embedding source (Cohere vs OpenAI): Deferred (can infer from vector size)

---

## Research Task 4: Error Handling Patterns

**Question**: How should we handle Qdrant connection failures and embedding API rate limits?

**Research**:

**FastAPI Exception Handling Best Practices**:
1. **Use custom exceptions** for domain-specific errors
2. **Return proper HTTP status codes**:
   - 400: Bad Request (invalid input)
   - 503: Service Unavailable (Qdrant/API down)
   - 504: Gateway Timeout (request took too long)
   - 500: Internal Server Error (unexpected exceptions)
3. **Provide clear error messages** in response body
4. **Log errors** with context (for debugging)

**Decision**: **Implement exception hierarchy with FastAPI exception handlers**

**Implementation**:
```python
# app/models/exceptions.py
class QdrantConnectionError(Exception):
    """Raised when Qdrant is unreachable"""
    pass

class EmbeddingAPIError(Exception):
    """Raised when embedding API fails (rate limit, network, etc.)"""
    pass

class ChunkingError(Exception):
    """Raised when chunking fails (empty content, invalid format)"""
    pass

# app/main.py
from fastapi import HTTPException

@app.exception_handler(QdrantConnectionError)
async def qdrant_exception_handler(request, exc):
    return JSONResponse(
        status_code=503,
        content={"error": "Vector database unavailable", "detail": str(exc)}
    )

@app.exception_handler(EmbeddingAPIError)
async def embedding_exception_handler(request, exc):
    return JSONResponse(
        status_code=503,
        content={"error": "Embedding service unavailable", "detail": str(exc)}
    )
```

**Rate Limit Handling**:
```python
# app/services/embedder.py
import time
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10)
)
async def embed_text(text: str) -> list[float]:
    try:
        response = await openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding
    except RateLimitError as e:
        logger.warning(f"Rate limit hit, retrying... {e}")
        raise  # Tenacity will retry
    except Exception as e:
        raise EmbeddingAPIError(f"Embedding failed: {e}")
```

**Rationale**:
- **Exponential backoff**: Retries with increasing delays (2s, 4s, 8s) to avoid overwhelming APIs
- **Max 3 attempts**: Fail fast if service is truly down (don't retry forever)
- **Specific exceptions**: Allow different handling for different failure modes
- **Structured logging**: Include request context for debugging

**Alternatives Considered**:
- Circuit breaker pattern: Too complex for MVP (implement in Spec 3 if needed)
- Immediate failure (no retries): Too fragile for transient network errors

---

## Research Task 5: Testing Strategy

**Question**: How do we test async FastAPI endpoints with Qdrant dependencies?

**Research**:

**FastAPI Testing Best Practices**:
1. **Use `httpx.AsyncClient`** for async endpoint testing
2. **Use `pytest-asyncio`** for async test functions
3. **Mock external services** (Qdrant, OpenAI) for unit tests
4. **Use test database** for integration tests (separate Qdrant collection)

**Decision**: **Dual testing approach: Unit tests with mocks, Integration tests with real Qdrant**

**Unit Testing** (Fast, No External Dependencies):
```python
# tests/unit/test_chunker.py
import pytest
from app.services.chunker import chunk_text

def test_chunking_basic():
    text = "A" * 1500  # 1500 characters
    chunks = chunk_text(text, chunk_size=750, overlap=100)

    assert len(chunks) == 3  # (1500 - 100) / (750 - 100) + 1 ≈ 3
    assert len(chunks[0]) == 750
    assert chunks[0][-100:] == chunks[1][:100]  # Overlap check

def test_chunking_empty():
    with pytest.raises(ChunkingError):
        chunk_text("")

# tests/unit/test_embedder.py (mocked)
from unittest.mock import AsyncMock, patch

@pytest.mark.asyncio
@patch("app.services.embedder.openai_client.embeddings.create")
async def test_embed_text_success(mock_create):
    mock_create.return_value = AsyncMock(data=[AsyncMock(embedding=[0.1] * 1536)])

    from app.services.embedder import embed_text
    result = await embed_text("test")

    assert len(result) == 1536
    mock_create.assert_called_once()
```

**Integration Testing** (Slower, Uses Real Qdrant):
```python
# tests/integration/test_ingest_endpoint.py
import pytest
from httpx import AsyncClient
from app.main import app

@pytest.mark.asyncio
async def test_ingest_success():
    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/ingest", json={
            "content": "ROS2 is a robotics middleware.",
            "metadata": {
                "title": "Chapter 1",
                "chapter": 1,
                "section": "Introduction",
                "page": 1
            }
        })

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert data["chunks_created"] > 0
        assert "doc_id" in data

# tests/integration/test_query_endpoint.py
@pytest.mark.asyncio
async def test_query_returns_chunks():
    # First ingest a document (setup)
    # ... (ingest call here)

    async with AsyncClient(app=app, base_url="http://test") as client:
        response = await client.post("/query", json={
            "question": "What is ROS2?",
            "top_k": 5
        })

        assert response.status_code == 200
        data = response.json()
        assert "chunks" in data
        assert len(data["chunks"]) <= 5
        assert data["chunks"][0]["chunk_text"]  # Not empty
```

**Test Configuration**:
```python
# pytest.ini
[pytest]
asyncio_mode = auto
testpaths = tests
python_files = test_*.py
python_functions = test_*

# conftest.py
import pytest
from app.config import settings

@pytest.fixture(scope="session")
def test_qdrant_collection():
    """Create a test Qdrant collection, delete after tests"""
    from app.db.qdrant_client import get_client

    client = get_client()
    collection_name = "test_documentation_chunks"

    # Create collection
    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )

    yield collection_name

    # Cleanup
    client.delete_collection(collection_name)
```

**Rationale**:
- **Unit tests** verify logic in isolation (fast, no network calls)
- **Integration tests** verify API contracts with real dependencies
- **Separate test collection** prevents polluting production data
- **Fixtures** handle setup/teardown (create/delete test collections)

**Alternatives Considered**:
- Only unit tests: Insufficient for API contract validation
- Only integration tests: Too slow for TDD workflow
- Docker Compose for test Qdrant: Adds complexity (use cloud Qdrant for MVP)

---

## Summary of Decisions

| Research Area | Decision | Impact |
|---------------|----------|--------|
| **Chunking** | 750 chars, 100 overlap | Balanced precision + context |
| **Embedding Provider** | OpenAI text-embedding-3-small | 5x cheaper, better quality |
| **Vector Dimensions** | 1536 | OpenAI default, high precision |
| **Qdrant Schema** | 6 metadata fields (chapter, section, page, etc.) | Minimal but sufficient for MVP |
| **Error Handling** | Custom exceptions + exponential backoff | Resilient to transient failures |
| **Testing** | Dual approach: unit (mocked) + integration (real Qdrant) | Fast feedback + contract validation |

---

## Phase 0 Complete ✅

**All NEEDS CLARIFICATION items resolved.**

**Next**: Proceed to Phase 1 (Design & Contracts)
