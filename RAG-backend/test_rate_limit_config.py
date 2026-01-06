"""
Test script to verify the rate limiting configuration changes.
"""
import asyncio
import os

from app.config.rate_limit_config import rate_limit_config


async def test_rate_limit_config():
    """Test the rate limit configuration."""
    print("Testing Rate Limit Configuration...")
    print()

    # Test default values
    print(f"Max concurrent LLM calls (default): {rate_limit_config.get_max_concurrent_llm_calls()}")
    print(f"Initial retry delay (default): {rate_limit_config.get_initial_retry_delay()}")
    print(f"Max retry delay (default): {rate_limit_config.get_max_retry_delay()}")
    print(f"Max retries (default): {rate_limit_config.get_max_retries()}")
    print()

    # Set environment variables to test override
    os.environ["LLM_MAX_CONCURRENT"] = "1"
    os.environ["RETRY_INITIAL_DELAY"] = "3.0"
    os.environ["RETRY_MAX_DELAY"] = "90.0"
    os.environ["MAX_RETRIES"] = "4"

    print("After setting environment variables:")
    print(f"Max concurrent LLM calls: {rate_limit_config.get_max_concurrent_llm_calls()}")
    print(f"Initial retry delay: {rate_limit_config.get_initial_retry_delay()}")
    print(f"Max retry delay: {rate_limit_config.get_max_retry_delay()}")
    print(f"Max retries: {rate_limit_config.get_max_retries()}")
    print()

    # Clean up environment variables
    del os.environ["LLM_MAX_CONCURRENT"]
    del os.environ["RETRY_INITIAL_DELAY"]
    del os.environ["RETRY_MAX_DELAY"]
    del os.environ["MAX_RETRIES"]

    print("After cleaning up environment variables (back to defaults):")
    print(f"Max concurrent LLM calls: {rate_limit_config.get_max_concurrent_llm_calls()}")
    print(f"Initial retry delay: {rate_limit_config.get_initial_retry_delay()}")
    print(f"Max retry delay: {rate_limit_config.get_max_retry_delay()}")
    print(f"Max retries: {rate_limit_config.get_max_retries()}")
    print()

    print("[OK] Rate limit configuration tests passed!")


if __name__ == "__main__":
    asyncio.run(test_rate_limit_config())