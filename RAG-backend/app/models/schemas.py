"""
Pydantic models for request and response schemas.
"""

from pydantic import BaseModel, Field
from typing import Optional


# Document metadata
class DocumentMetadata(BaseModel):
    """Metadata for a book document (chapter/section)."""

    title: str
    chapter: int = Field(..., ge=1, le=100)
    section: Optional[str] = None
    page_start: Optional[int] = None
    page_end: Optional[int] = None


# Ingest endpoint
class IngestRequest(BaseModel):
    """Request model for POST /ingest endpoint."""

    content: str = Field(..., min_length=1, description="Markdown/HTML content to ingest")
    metadata: DocumentMetadata


class IngestResponse(BaseModel):
    """Response model for POST /ingest endpoint."""

    status: str
    doc_id: str
    chunks_created: int
    embeddings_stored: int


# Embed endpoint
class EmbedRequest(BaseModel):
    """Request model for POST /embed endpoint."""

    doc_id: str = Field(..., description="Document ID to re-embed")


class EmbedResponse(BaseModel):
    """Response model for POST /embed endpoint."""

    status: str
    doc_id: str
    embeddings_updated: int


# Query endpoint
class QueryRequest(BaseModel):
    """Request model for POST /query endpoint."""

    question: str = Field(..., min_length=1, max_length=1000)
    chapter: Optional[int] = Field(None, ge=1, le=100)
    section: Optional[str] = Field(None, max_length=200)
    selected_text: Optional[str] = Field(None, max_length=5000)
    top_k: int = Field(5, ge=1, le=20)

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What is ROS 2?",
                "chapter": 2,
                "top_k": 5
            }
        }


class Citation(BaseModel):
    """Citation information for a retrieved chunk."""

    chapter: int
    section: Optional[str]
    page: Optional[int]
    chunk_text: str


class QueryResponse(BaseModel):
    """Response model for POST /query endpoint."""

    answer: str
    mode: str  # "normal_rag" or "selected_text_only"
    citations: list[Citation]
    metadata: dict

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "ROS 2 is a flexible framework for robot software development.",
                "mode": "normal_rag",
                "citations": [
                    {
                        "chapter": 2,
                        "section": "Introduction",
                        "page": 40,
                        "chunk_text": "ROS 2 is a flexible framework..."
                    }
                ],
                "metadata": {
                    "retrieved_chunks": 5,
                    "chunk_ids": ["uuid1", "uuid2"]
                }
            }
        }


# Chapters endpoint
class ChapterInfo(BaseModel):
    """Information about a book chapter."""

    chapter: int
    title: str
    sections: list[str]


class ChaptersResponse(BaseModel):
    """Response model for GET /chapters endpoint."""

    chapters: list[ChapterInfo]
