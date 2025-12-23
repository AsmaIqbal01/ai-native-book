"""
Embedding service using Cohere embeddings.

Provides methods for single and batch text embedding.
"""

import cohere
from app.config import settings


class EmbeddingService:
    """Service for generating text embeddings using Cohere."""

    def __init__(self):
        # Check if we should use Cohere or OpenAI
        if hasattr(settings, 'cohere_api_key') and settings.cohere_api_key:
            self.client = cohere.AsyncClient(api_key=settings.cohere_api_key)
            self.model = "embed-english-v3.0"  # 1024-dimensional
            self.provider = "cohere"
        else:
            # Fallback to OpenAI if Cohere not available
            from openai import AsyncOpenAI
            self.client = AsyncOpenAI(api_key=settings.openai_api_key)
            self.model = "text-embedding-3-small"  # 768-dimensional
            self.provider = "openai"

    async def embed_text(self, text: str) -> list[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed.

        Returns:
            list[float]: Embedding vector (1024-dim for Cohere, 768-dim for OpenAI).
        """
        if self.provider == "cohere":
            response = await self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_document"  # For indexing documents
            )
            return response.embeddings[0]
        else:
            response = await self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding

    async def embed_batch(self, texts: list[str]) -> list[list[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of texts to embed.

        Returns:
            list[list[float]]: List of embedding vectors.
        """
        if self.provider == "cohere":
            # Cohere supports up to 96 texts per batch
            if len(texts) > 96:
                raise ValueError("Batch size must be <= 96 for Cohere")

            response = await self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"
            )
            return response.embeddings
        else:
            # OpenAI supports up to 2048 inputs per batch, but we'll be conservative
            if len(texts) > 100:
                raise ValueError("Batch size must be <= 100")

            response = await self.client.embeddings.create(
                model=self.model,
                input=texts
            )
            return [item.embedding for item in response.data]
