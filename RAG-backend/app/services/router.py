"""
OpenRouter LLM client configuration.

Provides a pre-configured OpenAI client for OpenRouter API with proper headers
and settings for optimal performance with free-tier models.
"""

import os
from openai import OpenAI, AsyncOpenAI
from app.config import settings
from typing import Optional


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
            api_key: OpenRouter API key (defaults to settings)
            base_url: OpenRouter base URL (defaults to settings)
            model: Model to use (defaults to settings)
            site_url: Optional site URL for rankings on openrouter.ai
            site_name: Optional site name for rankings on openrouter.ai
        """
        self.api_key = api_key or settings.llm_api_key
        self.base_url = base_url or settings.llm_base_url
        self.model = model or settings.llm_model
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
        extra_headers = self.get_extra_headers()

        return await self.async_client.chat.completions.create(
            extra_headers=extra_headers,
            extra_body={},
            model=self.model,
            messages=messages,
            **kwargs
        )


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
        from app.services.router import create_completion

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
        from app.services.router import acreate_completion

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
