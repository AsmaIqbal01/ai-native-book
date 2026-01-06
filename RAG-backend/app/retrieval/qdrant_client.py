"""
Qdrant Vector Database client.

Provides connection to Qdrant Cloud for vector search operations with
retry logic and graceful degradation.
"""

import logging
import time
from typing import Optional

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from app.config import settings

logger = logging.getLogger(__name__)

# Global Qdrant client
_client: QdrantClient | None = None
_connection_healthy: bool = False


def get_client(retry_attempts: int = 3, retry_delay: float = 2.0) -> Optional[QdrantClient]:
    """
    Get or create the Qdrant client with retry logic.

    Args:
        retry_attempts: Number of connection attempts (default: 3)
        retry_delay: Initial delay between retries in seconds (default: 2.0)

    Returns:
        QdrantClient: Qdrant client instance, or None if connection fails.
    """
    global _client, _connection_healthy

    if _client is not None and _connection_healthy:
        return _client

    last_error = None
    current_delay = retry_delay

    for attempt in range(1, retry_attempts + 1):
        try:
            logger.info(
                f"[Qdrant] Attempting connection to {settings.qdrant_url} "
                f"(attempt {attempt}/{retry_attempts})..."
            )

            _client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10,  # 10 second timeout
            )

            # Verify connection by fetching collections
            _client.get_collections()
            _connection_healthy = True

            logger.info(f"[Qdrant] ✓ Successfully connected to {settings.qdrant_url}")
            return _client

        except Exception as e:
            last_error = e
            _connection_healthy = False

            if attempt < retry_attempts:
                logger.warning(
                    f"[Qdrant] ✗ Connection attempt {attempt}/{retry_attempts} failed: {e}"
                )
                logger.info(f"[Qdrant] Retrying in {current_delay:.1f} seconds...")
                time.sleep(current_delay)
                current_delay *= 2  # Exponential backoff
            else:
                logger.error(
                    f"[Qdrant] ✗ All {retry_attempts} connection attempts failed. "
                    f"Last error: {e}"
                )
                _log_connection_troubleshooting()

    return None


def _log_connection_troubleshooting() -> None:
    """Log troubleshooting steps for Qdrant connection issues."""
    logger.error(
        "\n"
        "════════════════════════════════════════════════════════════════\n"
        "  QDRANT CONNECTION FAILED - TROUBLESHOOTING STEPS\n"
        "════════════════════════════════════════════════════════════════\n"
        "\n"
        f"URL: {settings.qdrant_url}\n"
        "\n"
        "Common issues:\n"
        "\n"
        "1. LOCAL SETUP (localhost/127.0.0.1):\n"
        "   → Qdrant server not running\n"
        "   → Start with Docker:\n"
        "     docker run -d --name qdrant -p 6333:6333 -p 6334:6334 qdrant/qdrant\n"
        "\n"
        "2. CLOUD SETUP (*.qdrant.io):\n"
        "   → Invalid QDRANT_URL or QDRANT_API_KEY in .env\n"
        "   → Check credentials at https://cloud.qdrant.io\n"
        "\n"
        "3. NETWORK ISSUES:\n"
        "   → Firewall blocking port 6333\n"
        "   → VPN or proxy interference\n"
        "   → Test: curl http://localhost:6333/health\n"
        "\n"
        "4. WINDOWS-SPECIFIC:\n"
        "   → Windows Firewall blocking connection\n"
        "   → Docker Desktop not running\n"
        "   → Use 'localhost' instead of '127.0.0.1' in .env\n"
        "\n"
        "════════════════════════════════════════════════════════════════\n"
    )


async def health_check() -> bool:
    """
    Check if the Qdrant connection is healthy.

    Returns:
        bool: True if connection is healthy, False otherwise.
    """
    global _connection_healthy

    try:
        client = get_client(retry_attempts=1)
        if client is None:
            return False

        # Get collections to verify connection
        client.get_collections()
        _connection_healthy = True
        return True

    except Exception as e:
        logger.debug(f"[Qdrant] Health check failed: {e}")
        _connection_healthy = False
        return False


def initialize_collection() -> bool:
    """
    Create Qdrant collection if it doesn't exist.

    Creates 'documentation_chunks' collection with:
    - 1536-dimensional vectors (OpenAI text-embedding-3-small)
    - Cosine distance metric for semantic similarity

    Returns:
        bool: True if collection was created/verified, False if Qdrant unavailable.
    """
    client = get_client()

    if client is None:
        logger.warning(
            "[Qdrant] ⚠ Skipping collection initialization - Qdrant unavailable. "
            "App will continue without vector search."
        )
        return False

    collection_name = "documentation_chunks"

    try:
        # Check if collection exists
        collections = client.get_collections().collections
        collection_names = [c.name for c in collections]

        if collection_name not in collection_names:
            # Create collection with 1536-dim vectors (OpenAI text-embedding-3-small)
            client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=1536,  # OpenAI text-embedding-3-small dimensions
                    distance=Distance.COSINE  # Cosine similarity for semantic search
                )
            )
            logger.info(f"[Qdrant] ✓ Created collection: {collection_name}")
        else:
            logger.info(f"[Qdrant] ✓ Collection exists: {collection_name}")

        return True

    except Exception as e:
        logger.error(f"[Qdrant] ✗ Failed to initialize collection: {e}")
        return False


def close_client() -> None:
    """Close the Qdrant client gracefully."""
    global _client, _connection_healthy

    if _client is not None:
        try:
            _client.close()
            logger.info("[Qdrant] Connection closed")
        except Exception as e:
            logger.warning(f"[Qdrant] Error closing connection: {e}")
        finally:
            _client = None
            _connection_healthy = False


def is_available() -> bool:
    """
    Check if Qdrant is currently available.

    Returns:
        bool: True if Qdrant connection is healthy, False otherwise.
    """
    global _connection_healthy
    return _connection_healthy
