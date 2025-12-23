"""
Retrieval service for searching Qdrant vector database.

Handles semantic search with metadata filtering.
"""

from typing import Optional
from qdrant_client.models import Filter, FieldCondition, MatchValue
from app.db.qdrant_client import get_client


class RetrieverService:
    """Service for retrieving relevant chunks from Qdrant."""

    def __init__(self):
        self.client = get_client()
        self.collection_name = "book_chunks"

    async def search(
        self,
        query_vector: list[float],
        chapter: Optional[int] = None,
        section: Optional[str] = None,
        top_k: int = 5
    ) -> list[dict]:
        """
        Search Qdrant with optional metadata filters.

        Args:
            query_vector: 768-dimensional query embedding.
            chapter: Optional chapter filter.
            section: Optional section filter.
            top_k: Number of results to return.

        Returns:
            list[dict]: List of chunks with metadata.
        """
        # Build filter conditions
        filter_conditions = []

        if chapter is not None:
            filter_conditions.append(
                FieldCondition(key="chapter", match=MatchValue(value=chapter))
            )

        if section is not None:
            filter_conditions.append(
                FieldCondition(key="section", match=MatchValue(value=section))
            )

        # Create filter (None if no conditions)
        query_filter = Filter(must=filter_conditions) if filter_conditions else None

        # Perform search
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=query_filter,
            limit=top_k,
        ).points

        # Format results
        chunks = []
        for hit in results:
            # Extract payload (hit is a ScoredPoint object)
            payload = hit.payload if hasattr(hit, 'payload') else {}
            chunks.append({
                "id": str(hit.id),
                "payload": payload,
                "score": hit.score if hasattr(hit, 'score') else 0.0,
            })

        return chunks
