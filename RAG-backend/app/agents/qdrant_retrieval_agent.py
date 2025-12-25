"""
QdrantRetrievalAgent - FR-006

Handles query embedding, vector search, and context building.
Reuses existing EmbeddingService and RetrieverService.
"""

from app.agents.base_agent import BaseAgent, ValidationResult
from app.models.agent_types import RetrievalInput, RetrievalResult
from app.services.embedder import EmbeddingService
from app.services.retriever import RetrieverService
from app.services.context_builder import build_context_normal_rag
import logging

logger = logging.getLogger(__name__)


class QdrantRetrievalAgent(BaseAgent[RetrievalInput, RetrievalResult]):
    """
    Agent responsible for vector retrieval from Qdrant.

    Responsibilities:
    - Embed user query using EmbeddingService (Cohere/OpenAI)
    - Search Qdrant using RetrieverService
    - Apply metadata filters (chapter, section)
    - Build formatted context string
    - Handle graceful failures (empty results OK)

    Reuses shared intelligence:
    - EmbeddingService (app/services/embedder.py)
    - RetrieverService (app/services/retriever.py)
    - build_context_normal_rag (app/services/context_builder.py)
    """

    def __init__(self):
        """Initialize retrieval agent with reusable services."""
        super().__init__()
        self.embedder = EmbeddingService()
        self.retriever = RetrieverService()

    async def validate_input(self, input_data: RetrievalInput) -> ValidationResult:
        """
        Validate retrieval input.

        Args:
            input_data: RetrievalInput with query and filters

        Returns:
            ValidationResult
        """
        # Validate query is not empty
        if not input_data.query or input_data.query.strip() == "":
            return ValidationResult(
                is_valid=False,
                error_message="Query cannot be empty for retrieval"
            )

        # Validate top_k is within bounds
        if input_data.top_k < 1 or input_data.top_k > 20:
            return ValidationResult(
                is_valid=False,
                error_message="top_k must be between 1 and 20"
            )

        return ValidationResult(is_valid=True, normalized_input=input_data)

    async def execute(self, input_data: RetrievalInput) -> RetrievalResult:
        """
        Execute vector retrieval pipeline.

        Pipeline:
        1. Embed query (Cohere/OpenAI)
        2. Search Qdrant with filters
        3. Build context string
        4. Extract chunk metadata

        Args:
            input_data: RetrievalInput with validated query

        Returns:
            RetrievalResult with chunks, IDs, and formatted context

        Raises:
            Exception: If embedding or retrieval fails (caught by ErrorRecoveryAgent)
        """
        logger.info(
            "Starting vector retrieval",
            extra={
                "query_preview": input_data.query[:100],
                "chapter_filter": input_data.chapter,
                "top_k": input_data.top_k
            }
        )

        try:
            # Step 1: Embed query
            query_embedding = await self.embedder.embed_text(input_data.query)
            logger.info("Query embedded successfully", extra={"vector_dim": len(query_embedding)})

            # Step 2: Search Qdrant
            search_results = await self.retriever.search(
                query_vector=query_embedding,
                chapter=input_data.chapter,
                section=input_data.section,
                top_k=input_data.top_k
            )

            # Step 3: Extract chunks and IDs
            chunks = [result["payload"] for result in search_results]
            chunk_ids = [result["id"] for result in search_results]

            # Step 4: Build context string
            if chunks:
                context = build_context_normal_rag(chunks)
            else:
                context = ""
                logger.warning("No chunks retrieved from Qdrant", extra={"query": input_data.query[:100]})

            logger.info(
                "Retrieval completed",
                extra={
                    "chunks_found": len(chunks),
                    "context_length": len(context)
                }
            )

            return RetrievalResult(
                chunks=chunks,
                chunk_ids=chunk_ids,
                context=context,
                metadata={
                    "chunks_retrieved": len(chunks),
                    "embedding_dim": len(query_embedding),
                    "chapter_filter": input_data.chapter,
                    "section_filter": input_data.section,
                    "top_k_requested": input_data.top_k
                }
            )

        except ConnectionError as e:
            # Qdrant connection failed
            logger.error("Qdrant connection failed", extra={"error": str(e)})
            raise  # Re-raise to be caught by ErrorRecoveryAgent

        except Exception as e:
            # Unexpected error during retrieval
            logger.error(
                "Retrieval failed unexpectedly",
                extra={
                    "error": str(e),
                    "error_type": type(e).__name__
                }
            )
            raise  # Re-raise to be caught by ErrorRecoveryAgent
