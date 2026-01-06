"""
Embedding service using Cohere embeddings.

Provides methods for single and batch text embedding with rate limiting and retry logic.
"""

import cohere
from openai import RateLimitError
from app.config import settings
from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
from app.utils.cache import cache_embeddings
from app.utils.provider_logger import get_provider_logger
import logging
import asyncio
import time

logger = logging.getLogger(__name__)

# Define the exception types for cohere - use a safer approach
try:
    from cohere import CohereError
except ImportError:
    # Define a fallback if CohereError is not available
    class CohereError(Exception):
        pass


class EmbeddingService:
    """Service for generating text embeddings using Cohere."""

    def __init__(self):
        # Check if we should use Cohere or OpenAI
        if hasattr(settings, 'cohere_api_key') and settings.cohere_api_key:
            self.client = cohere.AsyncClient(api_key=settings.cohere_api_key)
            self.model = "embed-english-v3.0"  # 1024-dimensional
            self.provider = "cohere"
            logger.info("EmbeddingService initialized with Cohere provider")
        elif settings.openai_api_key:
            # Use OpenAI if available
            from openai import AsyncOpenAI
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = "text-embedding-3-small"  # 768-dimensional
            self.provider = "openai"
            logger.info("EmbeddingService initialized with OpenAI provider")
        else:
            # No embedding provider configured
            raise ValueError(
                "No embedding provider configured. Please set either:\n"
                "  - OPENAI_API_KEY (recommended for Claude users), or\n"
                "  - COHERE_API_KEY (alternative)\n"
                "in your .env file."
            )

    @cache_embeddings(ttl=7200)  # Cache for 2 hours
    @retry_with_exponential_backoff(
        max_retries=3,
        initial_delay=1.0,
        exponential_base=2.0,
        jitter=True,
        max_delay=30.0,
        exceptions=(RateLimitError, CohereError, Exception),
        detect_429=True,
        detect_auth_errors=True,  # Enable authentication error detection
        detect_network_errors=True  # Enable network error detection
    )
    async def embed_text(self, text: str) -> list[float]:
        """
        Generate embedding for a single text with retry logic and concurrency limiting.

        Args:
            text: Text to embed.

        Returns:
            list[float]: Embedding vector (1024-dim for Cohere, 768-dim for OpenAI).
        """
        # Acquire concurrency slot before making the API call
        limiter = get_llm_concurrency_limiter()
        async with limiter.context():
            logger.debug(
                f"Acquired concurrency slot for embedding. "
                f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}",
                extra={'provider': self.provider, 'concurrent_usage': limiter.current_usage}
            )

            start_time = time.time()
            logger.debug(
                f"Making embedding call for text length {len(text)}",
                extra={'provider': self.provider, 'text_length': len(text), 'operation': 'embedding_start'}
            )

            if self.provider == "cohere":
                response = await self.client.embed(
                    texts=[text],
                    model=self.model,
                    input_type="search_document"  # For indexing documents
                )
                embedding_result = response.embeddings[0]
            else:
                response = await self.client.embeddings.create(
                    model=self.model,
                    input=text
                )
                embedding_result = response.data[0].embedding

            duration = time.time() - start_time
            logger.debug(
                f"Embedding call completed in {duration:.2f}s",
                extra={'provider': self.provider, 'operation': 'embedding_complete', 'duration': duration}
            )

            return embedding_result

    @retry_with_exponential_backoff(
        max_retries=3,
        initial_delay=1.0,
        exponential_base=2.0,
        jitter=True,
        max_delay=30.0,
        exceptions=(RateLimitError, CohereError, Exception),
        detect_429=True,
        detect_auth_errors=True,  # Enable authentication error detection
        detect_network_errors=True  # Enable network error detection
    )
    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        """
        Generate embeddings for a batch of texts with retry logic and concurrency limiting.

        Args:
            texts: List of texts to embed.

        Returns:
            list[list[float]]: List of embedding vectors.
        """
        # Acquire concurrency slot before making the API call
        limiter = get_llm_concurrency_limiter()
        async with limiter.context():
            logger.debug(
                f"Acquired concurrency slot for batch embedding. "
                f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}",
                extra={'provider': self.provider, 'concurrent_usage': limiter.current_usage}
            )

            start_time = time.time()
            logger.debug(
                f"Making batch embedding call for {len(texts)} texts",
                extra={'provider': self.provider, 'batch_size': len(texts), 'operation': 'batch_embedding_start'}
            )

            if self.provider == "cohere":
                # Cohere supports up to 96 texts per batch
                if len(texts) > 96:
                    raise ValueError("Batch size must be <= 96 for Cohere")

                response = await self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document"
                )
                result = response.embeddings
            else:
                # OpenAI supports up to 2048 inputs per batch, but we'll be conservative
                if len(texts) > 100:
                    raise ValueError("Batch size must be <= 100")

                response = await self.client.embeddings.create(
                    model=self.model,
                    input=texts
                )
                result = [item.embedding for item in response.data]

            duration = time.time() - start_time
            logger.debug(
                f"Batch embedding call completed in {duration:.2f}s",
                extra={'provider': self.provider, 'operation': 'batch_embedding_complete', 'duration': duration}
            )

            return result
