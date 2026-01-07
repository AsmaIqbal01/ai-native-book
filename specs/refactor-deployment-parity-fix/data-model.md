# Phase 1: Data Model
## Spec 1 - Core RAG Backend Foundation

**Date**: 2025-12-25
**Branch**: refactor-deployment-parity-fix
**Status**: Complete

---

## Qdrant Vector Database Schema

### Collection: `documentation_chunks`

**Purpose**: Store embedded textbook chunks with metadata for semantic search.

**Configuration**:
```python
from qdrant_client.models import VectorParams, Distance

collection_config = {
    "collection_name": "documentation_chunks",
    "vectors_config": VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimensions
        distance=Distance.COSINE  # Cosine similarity for semantic search
    )
}
```

**Vector Entry Structure**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "vector": [0.023, -0.145, 0.891, ... /* 1536 floats */],
  "payload": {
    "chunk_text": "ROS2 (Robot Operating System 2) is a set of software libraries and tools...",
    "chapter": 3,
    "section": "Introduction to ROS2",
    "page": 42,
    "doc_id": "chapter-3-ros2-fundamentals",
    "chunk_index": 5
  }
}
```

**Field Definitions**:

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| `id` | UUID (string) | Yes | Unique identifier for chunk | `550e8400-...` |
| `vector` | float[] | Yes | Embedding vector (1536 dimensions) | `[0.023, -0.145, ...]` |
| `chunk_text` | string | Yes | Full text of the chunk | `"ROS2 is a..."` |
| `chapter` | integer | Yes | Chapter number (1-100) | `3` |
| `section` | string | Yes | Section name | `"Introduction to ROS2"` |
| `page` | integer | Yes | Page number | `42` |
| `doc_id` | string | Yes | Original document identifier | `"chapter-3-ros2"` |
| `chunk_index` | integer | Yes | Position within document (0-based) | `5` |

**Indexing**:
- **Primary Index**: HNSW (Hierarchical Navigable Small World) for vector search
- **Payload Filters**: Enabled for `chapter`, `section`, `page` (metadata filtering)

**HNSW Parameters**:
```python
hnsw_config = {
    "m": 16,  # Number of bi-directional links (higher = better recall, more memory)
    "ef_construct": 100  # Size of dynamic candidate list (higher = better index quality)
}
```

---

## Pydantic Models (API Schemas)

### 1. Ingest Request/Response

**IngestRequest** (`POST /ingest`):
```python
from pydantic import BaseModel, Field

class DocumentMetadata(BaseModel):
    """Metadata for ingested document"""
    title: str = Field(..., min_length=1, max_length=200, description="Document title")
    chapter: int = Field(..., ge=1, le=100, description="Chapter number (1-100)")
    section: str = Field(default="", max_length=200, description="Section name (optional)")
    page: int = Field(..., ge=1, description="Starting page number")

class IngestRequest(BaseModel):
    """Request to ingest document content"""
    content: str = Field(..., min_length=100, description="Document content (Markdown or plain text)")
    metadata: DocumentMetadata

    class Config:
        json_schema_extra = {
            "example": {
                "content": "# Chapter 3: ROS2 Fundamentals\n\nROS2 is a robotics middleware...",
                "metadata": {
                    "title": "AI-Native Robotics Chapter 3",
                    "chapter": 3,
                    "section": "Introduction to ROS2",
                    "page": 42
                }
            }
        }
```

**IngestResponse** (`POST /ingest` → 200):
```python
class IngestResponse(BaseModel):
    """Response from ingest endpoint"""
    status: str = Field(..., description="Status: 'success' or 'error'")
    doc_id: str = Field(..., description="Unique document identifier (UUID)")
    chunks_created: int = Field(..., ge=0, description="Number of chunks created")
    embeddings_stored: int = Field(..., ge=0, description="Number of embeddings stored in Qdrant")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "success",
                "doc_id": "550e8400-e29b-41d4-a716-446655440000",
                "chunks_created": 12,
                "embeddings_stored": 12
            }
        }
```

---

### 2. Query Request/Response

**QueryRequest** (`POST /query`):
```python
class QueryRequest(BaseModel):
    """Request to search for relevant chunks"""
    question: str = Field(..., min_length=3, max_length=1000, description="User question")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of chunks to retrieve (1-20)")
    chapter_filter: int | None = Field(default=None, ge=1, le=100, description="Optional chapter filter")

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is ROS2 and how does it differ from ROS1?",
                "top_k": 5,
                "chapter_filter": 3
            }
        }
```

**ChunkResult** (nested in QueryResponse):
```python
class ChunkResult(BaseModel):
    """Individual chunk result from search"""
    chunk_text: str = Field(..., description="Full text of the chunk")
    chapter: int = Field(..., description="Chapter number")
    section: str = Field(..., description="Section name")
    page: int = Field(..., description="Page number")
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score (0.0 - 1.0)")

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_text": "ROS2 is a robotics middleware that provides...",
                "chapter": 3,
                "section": "Introduction to ROS2",
                "page": 42,
                "score": 0.89
            }
        }
```

**QueryResponse** (`POST /query` → 200):
```python
class QueryMetadata(BaseModel):
    """Metadata about the query execution"""
    query_embedding_dim: int = Field(..., description="Embedding vector dimensions")
    search_latency_ms: int = Field(..., ge=0, description="Search latency in milliseconds")
    qdrant_hits: int = Field(..., ge=0, description="Total hits from Qdrant")

class QueryResponse(BaseModel):
    """Response from query endpoint"""
    chunks: list[ChunkResult] = Field(..., description="Retrieved chunks ranked by relevance")
    metadata: QueryMetadata

    class Config:
        json_schema_extra = {
            "example": {
                "chunks": [
                    {
                        "chunk_text": "ROS2 (Robot Operating System 2) is...",
                        "chapter": 3,
                        "section": "Introduction to ROS2",
                        "page": 42,
                        "score": 0.89
                    },
                    {
                        "chunk_text": "Unlike ROS1, ROS2 uses DDS...",
                        "chapter": 3,
                        "section": "ROS2 Architecture",
                        "page": 45,
                        "score": 0.82
                    }
                ],
                "metadata": {
                    "query_embedding_dim": 1536,
                    "search_latency_ms": 234,
                    "qdrant_hits": 15
                }
            }
        }
```

---

### 3. Health Check Response

**HealthResponse** (`GET /health` → 200):
```python
from enum import Enum

class ServiceStatus(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    ERROR = "error"

class HealthResponse(BaseModel):
    """Health check response"""
    status: ServiceStatus = Field(..., description="Overall system status")
    qdrant_connected: bool = Field(..., description="Qdrant connection status")
    embedding_api_available: bool = Field(..., description="Embedding API availability")
    timestamp: str = Field(..., description="ISO 8601 timestamp")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "ok",
                "qdrant_connected": True,
                "embedding_api_available": True,
                "timestamp": "2025-12-25T12:34:56.789Z"
            }
        }
```

---

## Error Response Schema

**ErrorResponse** (Used for 400, 500, 503, 504):
```python
class ErrorResponse(BaseModel):
    """Standard error response"""
    error: str = Field(..., description="Error type/category")
    detail: str = Field(..., description="Detailed error message")
    status_code: int = Field(..., description="HTTP status code")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Vector database unavailable",
                "detail": "Failed to connect to Qdrant at https://qdrant-server:6333",
                "status_code": 503
            }
        }
```

---

## Validation Rules

### Content Validation (`IngestRequest`):
- **Minimum content length**: 100 characters (prevents empty/trivial content)
- **Maximum content length**: 100,000 characters (prevents memory exhaustion)
- **Chapter range**: 1-100 (validates chapter number)
- **Page range**: ≥1 (no negative pages)

### Query Validation (`QueryRequest`):
- **Minimum question length**: 3 characters (prevents empty queries)
- **Maximum question length**: 1000 characters (prevents abuse)
- **top_k range**: 1-20 (prevents excessive results)
- **Chapter filter**: 1-100 (if provided)

### Chunk Validation (Internal):
- **Chunk size**: 650-850 characters (target 750 ± 100)
- **Chunk overlap**: 100 characters (fixed)
- **Embedding vector**: Exactly 1536 floats (OpenAI dimension)

---

## Database Migrations (N/A for Qdrant)

**Note**: Qdrant is schema-less for payloads. No migrations required for Spec 1.

**Collection Initialization**:
```python
# app/db/qdrant_client.py
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

def initialize_collection():
    """Create Qdrant collection if it doesn't exist"""
    client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)

    collections = client.get_collections().collections
    if "documentation_chunks" not in [c.name for c in collections]:
        client.create_collection(
            collection_name="documentation_chunks",
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
        )
        logger.info("Created Qdrant collection: documentation_chunks")
    else:
        logger.info("Qdrant collection already exists: documentation_chunks")
```

---

## Summary

**Data Models Defined**:
1. ✅ Qdrant vector schema (1536-dim embeddings + 6 metadata fields)
2. ✅ Ingest API (IngestRequest, IngestResponse)
3. ✅ Query API (QueryRequest, QueryResponse, ChunkResult)
4. ✅ Health Check API (HealthResponse)
5. ✅ Error responses (ErrorResponse)

**Validation Rules**:
- ✅ Content length constraints (100-100k chars)
- ✅ Query length constraints (3-1000 chars)
- ✅ Chapter/page validation (1-100, ≥1)
- ✅ top_k limits (1-20)

**Next**: Generate API contracts (OpenAPI schemas)
