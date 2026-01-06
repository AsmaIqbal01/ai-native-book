"""
Ingestion endpoint for adding book content to the RAG system.

Orchestrates: parsing → chunking → Neon storage → embedding → Qdrant storage
"""

from fastapi import APIRouter, HTTPException
from app.models.schemas import IngestRequest, IngestResponse
from app.ingestion.embedder import EmbeddingService
from app.ingestion.chunker import chunk_text, count_tokens
from app.db.neon_client import get_pool
from app.retrieval.qdrant_client import get_client
from qdrant_client.models import PointStruct
import uuid

router = APIRouter()


@router.post("/ingest", response_model=IngestResponse)
async def ingest_content(request: IngestRequest):
    """
    Ingest book content into the RAG system.

    Process:
    1. Parse content (extract plain text)
    2. Chunk text (512 tokens, 50 overlap)
    3. Store document in Neon
    4. Store chunks in Neon
    5. Embed chunks
    6. Store embeddings in Qdrant

    Args:
        request: IngestRequest with content and metadata.

    Returns:
        IngestResponse: Summary of ingestion.
    """
    try:
        # Step 1: Extract plain text (for now, assume content is already plain text)
        # TODO: Add Markdown/HTML parsing if needed
        plain_text = request.content

        # Step 2: Chunk text
        chunks = chunk_text(plain_text, chunk_size=512, overlap=50)

        if not chunks:
            raise HTTPException(status_code=400, detail="Content too short to chunk")

        # Step 3: Store document in Neon
        pool = await get_pool()
        async with pool.acquire() as conn:
            doc_id = await conn.fetchval(
                """
                INSERT INTO documents (title, chapter, section, page_start, page_end)
                VALUES ($1, $2, $3, $4, $5)
                RETURNING id
                """,
                request.metadata.title,
                request.metadata.chapter,
                request.metadata.section,
                request.metadata.page_start,
                request.metadata.page_end
            )

            # Step 4: Store chunks in Neon
            chunk_ids = []
            for idx, chunk_text_content in enumerate(chunks):
                token_count = count_tokens(chunk_text_content)

                # Approximate page number (if page_start provided)
                page = request.metadata.page_start + idx if request.metadata.page_start else None

                chunk_id = await conn.fetchval(
                    """
                    INSERT INTO chunks
                    (doc_id, chunk_index, chunk_text, token_count, chapter, section, page)
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    RETURNING id
                    """,
                    doc_id,
                    idx,
                    chunk_text_content,
                    token_count,
                    request.metadata.chapter,
                    request.metadata.section,
                    page
                )
                chunk_ids.append(chunk_id)

        # Step 5: Embed chunks
        embedder = EmbeddingService()
        embeddings = await embedder.embed_batch(chunks)

        # Step 6: Store embeddings in Qdrant
        qdrant_client = get_client()
        points = []

        for chunk_id, embedding, chunk_text_content, idx in zip(chunk_ids, embeddings, chunks, range(len(chunks))):
            page = request.metadata.page_start + idx if request.metadata.page_start else None

            point = PointStruct(
                id=str(chunk_id),
                vector=embedding,
                payload={
                    "chunk_id": str(chunk_id),
                    "doc_id": str(doc_id),
                    "chunk_text": chunk_text_content,
                    "chapter": request.metadata.chapter,
                    "section": request.metadata.section,
                    "page": page,
                    "token_count": count_tokens(chunk_text_content)
                }
            )
            points.append(point)

        qdrant_client.upsert(
            collection_name="book_chunks",
            points=points
        )

        return IngestResponse(
            status="success",
            doc_id=str(doc_id),
            chunks_created=len(chunks),
            embeddings_stored=len(embeddings)
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")
