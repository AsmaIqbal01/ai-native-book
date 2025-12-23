"""
Qdrant Vector Database client.

Provides connection to Qdrant Cloud for vector search operations.
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from app.config import settings

# Global Qdrant client
_client: QdrantClient | None = None


def get_client() -> QdrantClient:
    """
    Get or create the Qdrant client.

    Returns:
        QdrantClient: Qdrant client instance.
    """
    global _client

    if _client is None:
        _client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

    return _client


async def health_check() -> bool:
    """
    Check if the Qdrant connection is healthy.

    Returns:
        bool: True if connection is healthy, False otherwise.
    """
    try:
        client = get_client()
        # Get cluster info to verify connection
        client.get_collections()
        return True
    except Exception:
        return False


def close_client() -> None:
    """Close the Qdrant client gracefully."""
    global _client

    if _client is not None:
        _client.close()
        _client = None
