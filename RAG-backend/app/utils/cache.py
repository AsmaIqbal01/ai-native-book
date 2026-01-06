"""
Cache utility for embeddings and LLM responses to prevent repeated calls.

Implements an in-memory cache with TTL and size limits to reduce API calls.
"""

import asyncio
import time
import hashlib
from typing import Any, Optional, Dict
from functools import wraps
import logging

logger = logging.getLogger(__name__)


class LRUCache:
    """
    Simple LRU cache with TTL support for embeddings and LLM responses.
    """

    def __init__(self, max_size: int = 1000, default_ttl: int = 3600):
        """
        Initialize the cache.

        Args:
            max_size: Maximum number of items to store
            default_ttl: Default time-to-live in seconds
        """
        self.max_size = max_size
        self.default_ttl = default_ttl
        self._cache: Dict[str, tuple] = {}  # key -> (value, expiry_time, access_time)
        self._access_order = {}  # key -> access_time
        self._lock = asyncio.Lock()

    def _make_key(self, *args, **kwargs) -> str:
        """Create a hash key from function arguments."""
        key_data = str((args, sorted(kwargs.items())))
        return hashlib.md5(key_data.encode()).hexdigest()

    def _is_expired(self, expiry_time: float) -> bool:
        """Check if a cached item has expired."""
        return time.time() > expiry_time

    def _evict_if_needed(self):
        """Remove least recently used items if cache is full."""
        if len(self._cache) <= self.max_size:
            return

        # Sort by access time to find LRU items
        sorted_items = sorted(self._access_order.items(), key=lambda x: x[1])
        items_to_remove = len(self._cache) - self.max_size + 1

        for key, _ in sorted_items[:items_to_remove]:
            if key in self._cache:
                del self._cache[key]
                if key in self._access_order:
                    del self._access_order[key]

    async def get(self, key: str) -> Optional[Any]:
        """Get a value from the cache."""
        async with self._lock:
            if key not in self._cache:
                return None

            value, expiry_time, access_time = self._cache[key]

            if self._is_expired(expiry_time):
                del self._cache[key]
                if key in self._access_order:
                    del self._access_order[key]
                return None

            # Update access time
            new_access_time = time.time()
            self._cache[key] = (value, expiry_time, new_access_time)
            self._access_order[key] = new_access_time

            return value

    async def set(self, key: str, value: Any, ttl: Optional[int] = None) -> None:
        """Set a value in the cache."""
        async with self._lock:
            expiry_time = time.time() + (ttl or self.default_ttl)
            access_time = time.time()

            self._cache[key] = (value, expiry_time, access_time)
            self._access_order[key] = access_time

            self._evict_if_needed()

    async def delete(self, key: str) -> bool:
        """Delete a key from the cache."""
        async with self._lock:
            if key in self._cache:
                del self._cache[key]
                if key in self._access_order:
                    del self._access_order[key]
                return True
            return False

    async def clear(self):
        """Clear the entire cache."""
        async with self._lock:
            self._cache.clear()
            self._access_order.clear()


# Global cache instances
_embeddings_cache = LRUCache(max_size=2000, default_ttl=7200)  # 2 hours for embeddings
_responses_cache = LRUCache(max_size=1000, default_ttl=1800)   # 30 minutes for responses


def cache_embeddings(ttl: Optional[int] = None):
    """
    Decorator to cache embedding results.

    Args:
        ttl: Time-to-live in seconds for the cached result
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # Create cache key from the text being embedded
            text = args[1] if len(args) > 1 else kwargs.get('text', '')
            key = f"embed_{hashlib.md5(text.encode()).hexdigest()}"

            # Try to get from cache first
            cached_result = await _embeddings_cache.get(key)
            if cached_result is not None:
                logger.debug(f"Cache hit for embedding: {key[:8]}...")
                return cached_result

            # Call the actual function
            result = await func(*args, **kwargs)

            # Store in cache
            await _embeddings_cache.set(key, result, ttl)
            logger.debug(f"Cached embedding result: {key[:8]}...")

            return result
        return wrapper
    return decorator


def cache_llm_responses(ttl: Optional[int] = None):
    """
    Decorator to cache LLM responses.

    Args:
        ttl: Time-to-live in seconds for the cached result
    """
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # Create cache key from function arguments
            key = _responses_cache._make_key(*args, **kwargs)
            cache_key = f"llm_{key}"

            # Try to get from cache first
            cached_result = await _responses_cache.get(cache_key)
            if cached_result is not None:
                logger.debug(f"Cache hit for LLM response: {cache_key[:8]}...")
                return cached_result

            # Call the actual function
            result = await func(*args, **kwargs)

            # Store in cache
            await _responses_cache.set(cache_key, result, ttl)
            logger.debug(f"Cached LLM response: {cache_key[:8]}...")

            return result
        return wrapper
    return decorator


def get_embeddings_cache():
    """Get the global embeddings cache instance."""
    return _embeddings_cache


def get_responses_cache():
    """Get the global responses cache instance."""
    return _responses_cache


async def clear_all_caches():
    """Clear all caches."""
    await _embeddings_cache.clear()
    await _responses_cache.clear()
    logger.info("All caches cleared")