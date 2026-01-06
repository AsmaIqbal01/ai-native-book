"""
Qdrant Indexing Service for Documentation Chunks.

Handles embedding generation and storage of documentation chunks in Qdrant
following the canonical schema.
"""

import asyncio
from typing import List, Dict, Any, Optional
from qdrant_client import AsyncQdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    CollectionInfo,
    OptimizersConfigDiff,
)
from app.config import settings
from app.ingestion.embedder import EmbeddingService
import logging

logger = logging.getLogger(__name__)




class QdrantIndexer:
    """
    Service for indexing documentation chunks into Qdrant vector database.

    Handles:
    - Collection creation and management
    - Batch embedding generation
    - Efficient chunk storage with metadata
    - Schema validation
    """

    def __init__(
        self,
        collection_name: str = "documentation_chunks",
        vector_size: int = 1024,  # Cohere default, 768 for OpenAI
        batch_size: int = 50
    ):
        """
        Initialize Qdrant indexer.

        Args:
            collection_name: Name of Qdrant collection
            vector_size: Dimension of embedding vectors
            batch_size: Number of chunks to process at once
        """
        self.collection_name = collection_name
        self.vector_size = vector_size
        self.batch_size = batch_size

        # Initialize Qdrant client
        self.client = AsyncQdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Initialize embedding service
        self.embedder = EmbeddingService()

        # Adjust vector size based on provider
        if self.embedder.provider == "openai":
            self.vector_size = 768
        else:
            self.vector_size = 1024

    async def create_collection(self, recreate: bool = False) -> None:
        """
        Create Qdrant collection for documentation chunks.

        Args:
            recreate: If True, delete existing collection and create new one
        """
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_exists = any(
                col.name == self.collection_name
                for col in collections.collections
            )

            if collection_exists:
                if recreate:
                    logger.info(f"Deleting existing collection: {self.collection_name}")
                    await self.client.delete_collection(self.collection_name)
                else:
                    logger.info(f"Collection {self.collection_name} already exists")
                    return

            # Create collection
            logger.info(f"Creating collection: {self.collection_name}")
            await self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                ),
                optimizers_config=OptimizersConfigDiff(
                    indexing_threshold=10000,  # Build index after 10k points
                )
            )
            logger.info(f"Collection created: {self.collection_name}")

        except Exception as e:
            logger.error(f"Error creating collection: {e}")
            raise

    async def get_collection_info(self) -> Optional[CollectionInfo]:
        """Get information about the collection."""
        try:
            return await self.client.get_collection(self.collection_name)
        except Exception:
            return None

    async def index_chunks(
        self,
        chunks: List[Dict[str, Any]],
        show_progress: bool = True
    ) -> Dict[str, Any]:
        """
        Index documentation chunks into Qdrant.

        Args:
            chunks: List of chunk objects following canonical schema
            show_progress: Whether to print progress

        Returns:
            Dictionary with indexing statistics
        """
        total_chunks = len(chunks)
        indexed_count = 0
        failed_count = 0
        batch_num = 0

        logger.info(f"Starting indexing of {total_chunks} chunks...")

        # Process in batches
        for i in range(0, total_chunks, self.batch_size):
            batch_num += 1
            batch = chunks[i:i + self.batch_size]

            try:
                # Generate embeddings for batch
                texts = [chunk['chunk_text'] for chunk in batch]
                embeddings = await self.embedder.embed_batch(texts)

                # Create points for Qdrant
                points = []
                for chunk, embedding in zip(batch, embeddings):
                    point = PointStruct(
                        id=chunk['chunk_id'],
                        vector=embedding,
                        payload={
                            'doc_id': chunk['doc_id'],
                            'chunk_text': chunk['chunk_text'],
                            'chapter': chunk['chapter'],
                            'section': chunk['section'],
                            'page': chunk['page'],
                            'token_count': chunk['token_count'],
                        }
                    )
                    points.append(point)

                # Upload to Qdrant
                await self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )

                indexed_count += len(batch)

                if show_progress:
                    progress = (indexed_count / total_chunks) * 100
                    logger.info(f"Batch {batch_num}: Indexed {indexed_count}/{total_chunks} chunks ({progress:.1f}%)")

            except Exception as e:
                failed_count += len(batch)
                logger.error(f"Error indexing batch {batch_num}: {e}")
                if show_progress:
                    logger.error(f"✗ Batch {batch_num} failed: {e}")

        # Get final collection info
        collection_info = await self.get_collection_info()
        total_in_collection = collection_info.points_count if collection_info else 0

        stats = {
            'total_chunks': total_chunks,
            'indexed_count': indexed_count,
            'failed_count': failed_count,
            'success_rate': (indexed_count / total_chunks * 100) if total_chunks > 0 else 0,
            'total_in_collection': total_in_collection
        }

        logger.info(f"Indexing complete: {indexed_count}/{total_chunks} chunks indexed successfully")

        return stats

    async def search_chunks(
        self,
        query: str,
        limit: int = 10,
        chapter_filter: Optional[int] = None,
        score_threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks using semantic similarity.

        Args:
            query: Search query
            limit: Maximum number of results
            chapter_filter: Optional chapter number to filter by
            score_threshold: Minimum similarity score (0-1)

        Returns:
            List of matching chunks with scores
        """
        # Generate query embedding
        query_embedding = await self.embedder.embed_text(query)

        # Build filter if chapter specified
        query_filter = None
        if chapter_filter is not None:
            from qdrant_client.models import Filter, FieldCondition, MatchValue
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="chapter",
                        match=MatchValue(value=chapter_filter)
                    )
                ]
            )

        # Search
        results = await self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            query_filter=query_filter,
            score_threshold=score_threshold
        )

        # Format results
        formatted_results = []
        for hit in results:
            formatted_results.append({
                'chunk_id': hit.id,
                'score': hit.score,
                'chunk_text': hit.payload.get('chunk_text', ''),
                'chapter': hit.payload.get('chapter', 0),
                'section': hit.payload.get('section', ''),
                'page': hit.payload.get('page', 0),
                'token_count': hit.payload.get('token_count', 0),
                'doc_id': hit.payload.get('doc_id', '')
            })

        return formatted_results

    async def delete_by_doc_id(self, doc_id: str) -> int:
        """
        Delete all chunks for a specific document.

        Args:
            doc_id: Document UUID

        Returns:
            Number of chunks deleted
        """
        from qdrant_client.models import Filter, FieldCondition, MatchValue

        # Get points to delete
        filter_condition = Filter(
            must=[
                FieldCondition(
                    key="doc_id",
                    match=MatchValue(value=doc_id)
                )
            ]
        )

        # Delete points
        await self.client.delete(
            collection_name=self.collection_name,
            points_selector=filter_condition
        )

        logger.info(f"Deleted chunks for doc_id: {doc_id}")
        return 0  # Qdrant doesn't return count

    async def get_stats(self) -> Dict[str, Any]:
        """Get collection statistics."""
        info = await self.get_collection_info()

        if not info:
            return {'error': 'Collection not found'}

        return {
            'collection_name': self.collection_name,
            'total_chunks': info.points_count,
            'vector_size': info.config.params.vectors.size,
            'distance_metric': info.config.params.vectors.distance,
            'indexed': info.indexed_vectors_count if hasattr(info, 'indexed_vectors_count') else 'N/A',
        }

    async def close(self):
        """Close the Qdrant client connection."""
        await self.client.close()


async def validate_indexed_chunks(
    indexer: QdrantIndexer,
    sample_size: int = 5
) -> Dict[str, Any]:
    """
    Validate that chunks are properly indexed in Qdrant.

    Args:
        indexer: QdrantIndexer instance
        sample_size: Number of random chunks to validate

    Returns:
        Validation results
    """
    try:
        # Search with a generic query
        results = await indexer.search_chunks(
            query="introduction",
            limit=sample_size,
            score_threshold=0.0  # Accept any match for validation
        )

        validation = {
            'status': 'success' if len(results) > 0 else 'failed',
            'samples_found': len(results),
            'sample_chunks': []
        }

        for result in results:
            validation['sample_chunks'].append({
                'chunk_id': result['chunk_id'],
                'chapter': result['chapter'],
                'section': result['section'],
                'token_count': result['token_count'],
                'preview': result['chunk_text'][:100] + '...'
            })

        return validation

    except Exception as e:
        return {
            'status': 'error',
            'error': str(e)
        }
