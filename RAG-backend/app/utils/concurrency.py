"""
Concurrency limiter utilities for managing LLM API calls.

Implements a semaphore-based concurrency limiter to prevent overwhelming
LLM providers with too many simultaneous requests.
"""

import asyncio
from typing import Optional
import logging

# Import the rate limit configuration
from app.config.rate_limit_config import rate_limit_config

logger = logging.getLogger(__name__)


class ConcurrencyLimiter:
    """
    Concurrency limiter using asyncio.BoundedSemaphore to limit
    the number of simultaneous operations.

    Usage:
        limiter = ConcurrencyLimiter(3)  # Allow max 3 concurrent operations

        async with limiter.context():
            # Your async operation here
            pass
    """

    def __init__(self, max_concurrent: int = 3):
        """
        Initialize the concurrency limiter.

        Args:
            max_concurrent: Maximum number of concurrent operations allowed
        """
        self._semaphore = asyncio.BoundedSemaphore(max_concurrent)
        self._max_concurrent = max_concurrent
        logger.info(f"Concurrency limiter initialized with max {max_concurrent} concurrent operations")

    @property
    def max_concurrent(self) -> int:
        """Get the maximum number of concurrent operations allowed."""
        return self._max_concurrent

    @property
    def current_usage(self) -> int:
        """Get the current number of active operations."""
        return self._max_concurrent - self._semaphore._value

    @property
    def available_slots(self) -> int:
        """Get the number of available slots."""
        return self._semaphore._value

    def context(self):
        """
        Get a context manager for the concurrency limiter.

        Usage:
            async with limiter.context():
                # Your async operation here
                pass
        """
        return self._semaphore

    async def acquire(self):
        """Acquire a slot from the limiter."""
        await self._semaphore.acquire()

    def release(self):
        """Release a slot back to the limiter."""
        self._semaphore.release()


# Global instance for LLM calls
_llm_concurrency_limiter: Optional[ConcurrencyLimiter] = None


def get_llm_concurrency_limiter(max_concurrent: int = None) -> ConcurrencyLimiter:
    """
    Get the global LLM concurrency limiter.

    Args:
        max_concurrent: Maximum number of concurrent LLM calls (default: from config)

    Returns:
        ConcurrencyLimiter instance
    """
    if max_concurrent is None:
        max_concurrent = rate_limit_config.get_max_concurrent_llm_calls()

    global _llm_concurrency_limiter
    if _llm_concurrency_limiter is None:
        _llm_concurrency_limiter = ConcurrencyLimiter(max_concurrent)
    return _llm_concurrency_limiter