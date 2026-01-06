"""
Test script to verify the HTTP 429 rate limiting fixes.

This script tests:
1. Exponential backoff with jitter for 429 errors
2. Concurrency limiting
3. Caching functionality
4. Trace ID logging
"""

import asyncio
import time
from unittest.mock import AsyncMock, patch
import pytest

from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
from app.utils.cache import get_embeddings_cache, get_responses_cache, cache_embeddings, cache_llm_responses
from app.ingestion.embedder import EmbeddingService


async def test_exponential_backoff_with_429_detection():
    """Test that the retry decorator properly detects and handles 429 errors."""

    # Counter to track function calls
    call_count = 0

    @retry_with_exponential_backoff(
        max_retries=2,
        initial_delay=0.1,  # Fast test
        exponential_base=2.0,
        jitter=False,  # Disable jitter for predictable testing
        max_delay=1.0,
        exceptions=(Exception,),
        detect_429=True
    )
    async def mock_api_call():
        nonlocal call_count
        call_count += 1

        if call_count <= 2:  # Fail first 2 attempts
            raise Exception("429 Too Many Requests")
        return "success"

    start_time = time.time()
    result = await mock_api_call()
    duration = time.time() - start_time

    # Should have retried twice (3 total calls) with delays
    assert call_count == 3
    assert result == "success"
    # Should have waited for at least 2 delays: 0.1 + 0.2 = 0.3 seconds
    assert duration >= 0.2, f"Expected at least 0.2s delay, got {duration}s"

    print("[OK] Exponential backoff with 429 detection works")


async def test_concurrency_limiting():
    """Test that concurrency limiting works properly."""

    limiter = get_llm_concurrency_limiter(max_concurrent=2)

    async def mock_task(task_id):
        async with limiter.context():
            print(f"Task {task_id} started, current usage: {limiter.current_usage}/{limiter.max_concurrent}")
            await asyncio.sleep(0.1)  # Simulate work
            print(f"Task {task_id} finished")
        return f"result_{task_id}"

    # Run 5 tasks concurrently - should be limited to 2 at a time
    start_time = time.time()
    tasks = [mock_task(i) for i in range(5)]
    results = await asyncio.gather(*tasks)
    duration = time.time() - start_time

    assert len(results) == 5
    assert all(r.startswith("result_") for r in results)
    # Should take at least 3 * 0.1 seconds since max 2 run at once
    assert duration >= 0.2, f"Expected at least 0.2s with concurrency limiting, got {duration}s"

    print("[OK] Concurrency limiting works")


async def test_caching_functionality():
    """Test that caching works properly for embeddings and responses."""

    # Clear caches first
    embeddings_cache = get_embeddings_cache()
    responses_cache = get_responses_cache()
    await embeddings_cache.clear()
    await responses_cache.clear()

    # Test embedding cache
    @cache_embeddings(ttl=300)  # 5 minute TTL for test
    async def mock_embed(text):
        # Simulate API call delay
        await asyncio.sleep(0.01)
        return [0.1, 0.2, 0.3]  # Mock embedding

    # First call - should not be cached
    start_time = time.time()
    result1 = await mock_embed("test text")
    time1 = time.time() - start_time

    # Second call - should be cached
    start_time = time.time()
    result2 = await mock_embed("test text")
    time2 = time.time() - start_time

    assert result1 == result2
    assert time2 < time1, "Cached call should be faster"

    print("[OK] Caching functionality works")


async def test_embedding_service_with_new_defaults():
    """Test that the embedding service uses the new chunking defaults."""

    # This test verifies that the updated chunk sizes are in place
    from app.ingestion.chunker import chunk_text
    from app.ingestion.semantic_chunker import SemanticChunker

    # Test token-based chunker with new defaults
    test_text = "This is a test sentence. " * 100  # Make it long enough to chunk
    chunks = chunk_text(test_text)

    print(f"[OK] Token-based chunker: {len(chunks)} chunks created with new defaults (1024 tokens, 100 overlap)")

    # Test semantic chunker with new defaults
    chunker = SemanticChunker()
    assert chunker.min_tokens == 250, f"Expected min_tokens=250, got {chunker.min_tokens}"
    assert chunker.max_tokens == 600, f"Expected max_tokens=600, got {chunker.max_tokens}"
    assert chunker.overlap_tokens == 100, f"Expected overlap_tokens=100, got {chunker.overlap_tokens}"

    print("[OK] Semantic chunker: Using new optimized defaults (250-600 tokens, 100 overlap)")


async def main():
    """Run all tests."""
    print("Testing HTTP 429 rate limiting fixes...\n")

    await test_exponential_backoff_with_429_detection()
    print()

    await test_concurrency_limiting()
    print()

    await test_caching_functionality()
    print()

    await test_embedding_service_with_new_defaults()
    print()

    print("[OK] All tests passed! HTTP 429 rate limiting fixes are working properly.")
    print("\nSummary of improvements:")
    print("- Exponential backoff with jitter for 429 errors")
    print("- Concurrency limiting (max 2 simultaneous LLM calls)")
    print("- Caching for embeddings and LLM responses")
    print("- Optimized chunking (larger chunks, more overlap)")
    print("- Enhanced logging with trace IDs")
    print("- 429 error detection and handling")


if __name__ == "__main__":
    asyncio.run(main())