"""
Neon Serverless Postgres client using asyncpg.

Provides connection pooling for async database operations.
"""

import asyncpg
from app.config import settings

# Global connection pool
_pool: asyncpg.Pool | None = None


async def get_pool() -> asyncpg.Pool:
    """
    Get or create the asyncpg connection pool.

    Returns:
        asyncpg.Pool: Connection pool for Neon Postgres.

    Raises:
        Exception: If connection pool creation fails.
    """
    global _pool

    if _pool is None:
        _pool = await asyncpg.create_pool(
            dsn=settings.neon_database_url,
            min_size=2,  # Minimum connections in pool
            max_size=10,  # Maximum connections in pool
            command_timeout=60,  # Command timeout in seconds
        )

    return _pool


async def close_pool() -> None:
    """Close the connection pool gracefully."""
    global _pool

    if _pool is not None:
        await _pool.close()
        _pool = None


async def health_check() -> bool:
    """
    Check if the Neon database connection is healthy.

    Returns:
        bool: True if connection is healthy, False otherwise.
    """
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Simple query to test connection
            result = await conn.fetchval("SELECT 1")
            return result == 1
    except Exception:
        return False
