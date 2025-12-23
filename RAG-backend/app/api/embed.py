"""
Re-embedding endpoint for updating document embeddings.

Useful when changing embedding models or updating embeddings for existing content.
"""

from fastapi import APIRouter, HTTPException
from app.models.schemas import EmbedRequest, EmbedResponse
from app.services.embedder import EmbeddingService
from app.db.neon_client import get_pool
from app.db.qdrant_client import get_client
from qdrant_client.models import PointStruct
import uuid

router = APIRouter()


@router.post("/embed", response_model=EmbedResponse)
async def re_embed_document(request: EmbedRequest):
    """
    Re-embed an existing document.

    Process:
    1. Fetch chunks from Neon by doc_id
    2. Generate new embeddings
    3. Update embeddings in Qdrant

    Args:
        request: EmbedRequest with doc_id.

    Returns:
        EmbedResponse: Summary of re-embedding.
    """
    try:
        # Step 1: Fetch chunks from Neon
        pool = await get_pool()
        async with pool.acquire() as conn:
            chunks = await conn.fetch(
                """
                SELECT id, chunk_text, chunk_index, chapter, section, page, token_count, doc_id
                FROM chunks
                WHERE doc_id = $1
                ORDER BY chunk_index
                """,
                uuid.UUID(request.doc_id)
            )

            if not chunks:
                raise HTTPException(status_code=404, detail=f"Document {request.doc_id} not found")

        # Step 2: Embed chunks
        chunk_texts = [chunk["chunk_text"] for chunk in chunks]
        embedder = EmbeddingService()
        embeddings = await embedder.embed_batch(chunk_texts)

        # Step 3: Update Qdrant
        qdrant_client = get_client()
        points = []

        for chunk, embedding in zip(chunks, embeddings):
            point = PointStruct(
                id=str(chunk["id"]),
                vector=embedding,
                payload={
                    "chunk_id": str(chunk["id"]),
                    "doc_id": str(chunk["doc_id"]),
                    "chunk_text": chunk["chunk_text"],
                    "chapter": chunk["chapter"],
                    "section": chunk["section"],
                    "page": chunk["page"],
                    "token_count": chunk["token_count"]
                }
            )
            points.append(point)

        qdrant_client.upsert(
            collection_name="book_chunks",
            points=points
        )

        return EmbedResponse(
            status="success",
            doc_id=request.doc_id,
            embeddings_updated=len(embeddings)
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Re-embedding failed: {str(e)}")
