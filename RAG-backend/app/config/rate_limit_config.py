"""
Rate limiting configuration for the RAG backend.

This module provides configuration for rate limiting and concurrency controls
that can be adjusted based on the LLM provider and usage patterns.
"""

import os
from typing import Optional


class RateLimitConfig:
    """
    Configuration class for rate limiting parameters.
    """

    @staticmethod
    def get_max_concurrent_llm_calls() -> int:
        """
        Get the maximum number of concurrent LLM API calls.

        This can be overridden by the LLM_MAX_CONCURRENT environment variable.
        Default is 2, but can be adjusted based on your API provider limits.
        """
        max_concurrent_str = os.getenv("LLM_MAX_CONCURRENT", "2")
        try:
            max_concurrent = int(max_concurrent_str)
            # Ensure it's at least 1
            return max(max_concurrent, 1)
        except ValueError:
            return 2  # Default value if environment variable is invalid

    @staticmethod
    def get_initial_retry_delay() -> float:
        """
        Get the initial delay for retry attempts.

        This can be overridden by the RETRY_INITIAL_DELAY environment variable.
        Default is 1.0 seconds.
        """
        delay_str = os.getenv("RETRY_INITIAL_DELAY", "1.0")
        try:
            delay = float(delay_str)
            return max(delay, 0.1)  # Minimum 0.1 seconds
        except ValueError:
            return 1.0  # Default value

    @staticmethod
    def get_max_retry_delay() -> float:
        """
        Get the maximum delay for retry attempts.

        This can be overridden by the RETRY_MAX_DELAY environment variable.
        Default is 60.0 seconds.
        """
        delay_str = os.getenv("RETRY_MAX_DELAY", "60.0")
        try:
            delay = float(delay_str)
            return max(delay, 5.0)  # Minimum 5 seconds
        except ValueError:
            return 60.0  # Default value

    @staticmethod
    def get_max_retries() -> int:
        """
        Get the maximum number of retry attempts.

        This can be overridden by the MAX_RETRIES environment variable.
        Default is 3.
        """
        retries_str = os.getenv("MAX_RETRIES", "3")
        try:
            retries = int(retries_str)
            return max(retries, 1)  # At least 1 retry
        except ValueError:
            return 3  # Default value


# Create global instance
rate_limit_config = RateLimitConfig()