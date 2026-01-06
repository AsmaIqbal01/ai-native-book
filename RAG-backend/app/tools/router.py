"""
OpenRouter LLM client configuration.

Provides a pre-configured OpenAI client for OpenRouter API with proper headers
and settings for optimal performance with free-tier models.
Environment variables used:
- OPENROUTER_API_KEY: API key for OpenRouter
- OPENROUTER_BASE_URL: Base URL for OpenRouter API
- OPENROUTER_MODEL: Model to use (optional, defaults to 'mistralai/devstral-2512:free')
- OPENROUTER_SITE_URL: Optional site URL for rankings
- OPENROUTER_SITE_NAME: Optional site name for rankings
"""

import os
from openai import OpenAI, AsyncOpenAI, RateLimitError
from typing import Optional
from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
import asyncio
import time
import logging

logger = logging.getLogger(__name__)


class OpenRouterClient:
    """OpenRouter API client wrapper with proper configuration."""

    def __init__(
        self,
        api_key: Optional[str] = None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
        site_url: Optional[str] = None,
        site_name: Optional[str] = None,
    ):
        """
        Initialize OpenRouter client.

        Args:
            api_key: OpenRouter API key (defaults to OPENROUTER_API_KEY env var)
            base_url: OpenRouter base URL (defaults to OPENROUTER_BASE_URL env var)
            model: Model to use (defaults to settings)
            site_url: Optional site URL for rankings on openrouter.ai
            site_name: Optional site name for rankings on openrouter.ai
        """
        # Load OpenRouter-specific environment variables with clear error messages
        self.api_key = api_key or os.getenv("OPENROUTER_API_KEY")
        if not self.api_key:
            raise ValueError(
                "OPENROUTER_API_KEY environment variable is required for OpenRouter client. "
                "Please set it in your .env file or environment."
            )

        self.base_url = base_url or os.getenv("OPENROUTER_BASE_URL")
        if not self.base_url:
            raise ValueError(
                "OPENROUTER_BASE_URL environment variable is required for OpenRouter client. "
                "Please set it in your .env file or environment."
            )

        self.model = model or os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512:free")
        self.site_url = site_url or os.getenv("OPENROUTER_SITE_URL", "")
        self.site_name = site_name or os.getenv("OPENROUTER_SITE_NAME", "AI Native Book RAG")

        # Synchronous client
        self._sync_client = None
        # Asynchronous client
        self._async_client = None

    @property
    def sync(self) -> OpenAI:
        """Get synchronous OpenAI client configured for OpenRouter."""
        if self._sync_client is None:
            self._sync_client = OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
            )
        return self._sync_client

    @property
    def async_client(self) -> AsyncOpenAI:
        """Get asynchronous OpenAI client configured for OpenRouter."""
        if self._async_client is None:
            self._async_client = AsyncOpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
            )
        return self._async_client

    def get_extra_headers(self) -> dict:
        """
        Get extra headers for OpenRouter API requests.

        These headers are optional but help with rankings on openrouter.ai.
        """
        headers = {}
        if self.site_url:
            headers["HTTP-Referer"] = self.site_url
        if self.site_name:
            headers["X-Title"] = self.site_name
        return headers

    def create_completion(self, messages: list[dict], **kwargs) -> dict:
        """
        Create a chat completion using OpenRouter (synchronous).

        Args:
            messages: List of message dicts with 'role' and 'content'
            **kwargs: Additional arguments passed to chat.completions.create

        Returns:
            Completion response object
        """
        # For synchronous calls, we still want to respect concurrency limits
        # by using a thread lock or similar mechanism, but for now we'll just
        # log that this is a sync call that bypasses concurrency control
        logger.warning("Sync LLM call made - bypassing concurrency control. Use async method when possible.")

        extra_headers = self.get_extra_headers()

        return self.sync.chat.completions.create(
            extra_headers=extra_headers,
            extra_body={},
            model=self.model,
            messages=messages,
            **kwargs
        )

    async def acreate_completion(self, messages: list[dict], **kwargs) -> dict:
        """
        Create a chat completion using OpenRouter (asynchronous).

        Args:
            messages: List of message dicts with 'role' and 'content'
            **kwargs: Additional arguments passed to chat.completions.create

        Returns:
            Completion response object
        """
        # Acquire concurrency slot before making the API call
        limiter = get_llm_concurrency_limiter()
        async with limiter.context():
            logger.debug(
                f"Acquired concurrency slot for {self.model}. "
                f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}"
            )

            # Use retry decorator for the actual API call
            return await self._make_completion_with_retry(
                messages, **kwargs
            )

    @retry_with_exponential_backoff(
        max_retries=3,
        initial_delay=1.0,
        exponential_base=2.0,
        jitter=True,
        max_delay=30.0,
        exceptions=(RateLimitError,)
    )
    async def _make_completion_with_retry(self, messages: list[dict], **kwargs) -> dict:
        """
        Make completion call with retry logic specifically for rate limit errors.

        Args:
            messages: List of message dicts with 'role' and 'content'
            **kwargs: Additional arguments passed to chat.completions.create

        Returns:
            Completion response object

        Raises:
            RateLimitError: If rate limits continue after retries
        """
        extra_headers = self.get_extra_headers()
        start_time = time.time()

        logger.debug(f"Making LLM call to {self.model}")

        response = await self.async_client.chat.completions.create(
            extra_headers=extra_headers,
            extra_body={},
            model=self.model,
            messages=messages,
            **kwargs
        )

        duration = time.time() - start_time
        logger.debug(f"LLM call completed in {duration:.2f}s")

        return response


# Global router instance
_router_instance: Optional[OpenRouterClient] = None


def get_router() -> OpenRouterClient:
    """
    Get or create the global OpenRouter client instance.

    Returns:
        Configured OpenRouterClient instance
    """
    global _router_instance
    if _router_instance is None:
        _router_instance = OpenRouterClient()
    return _router_instance


# Convenience function for quick usage
def create_completion(messages: list[dict], **kwargs):
    """
    Quick access to synchronous completion.

    Example:
        ```python
        from app.tools.router import create_completion

        response = create_completion(
            messages=[
                {"role": "user", "content": "What is the meaning of life?"}
            ],
            temperature=0.7,
            max_tokens=500
        )
        print(response.choices[0].message.content)
        ```
    """
    return get_router().create_completion(messages, **kwargs)


async def acreate_completion(messages: list[dict], **kwargs):
    """
    Quick access to asynchronous completion.

    Example:
        ```python
        from app.tools.router import acreate_completion

        response = await acreate_completion(
            messages=[
                {"role": "user", "content": "What is the meaning of life?"}
            ],
            temperature=0.7,
            max_tokens=500
        )
        print(response.choices[0].message.content)
        ```
    """
    return await get_router().acreate_completion(messages, **kwargs)
