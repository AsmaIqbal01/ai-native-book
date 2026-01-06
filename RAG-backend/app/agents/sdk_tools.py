"""
SDK-native tools for RAG agents.
Wraps existing services as @function_tool decorated functions.

This module provides OpenAI Agents SDK-compliant tools that:
1. Wrap existing services (EmbeddingService, RetrieverService, etc.)
2. Preserve token efficiency mechanisms
3. Maintain backward compatibility
4. Enable tool reuse across multiple agents
"""

from typing import Annotated, Optional
from pydantic import BaseModel, Field
from agents import function_tool

from app.ingestion.embedder import EmbeddingService
from app.retrieval.retriever import RetrieverService
from app.tools.context_builder import build_context_normal_rag, DEFAULT_MAX_CONTEXT_TOKENS
from app.utils.logging import get_logger
from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
from app.utils.cache import cache_llm_responses
from app.utils.api_key_validator import validate_provider_configuration
import asyncio
import time

logger = get_logger(__name__)


# ============================================================================
# Structured Output Models
# ============================================================================

class RetrievalResult(BaseModel):
    """Structured output from embed_and_retrieve tool."""
    context: str = Field(description="Formatted context from retrieved chunks")
    chunks_retrieved: int = Field(description="Number of chunks retrieved")
    chunk_ids: list[str] = Field(description="IDs of retrieved chunks")
    metadata: dict = Field(description="Additional metadata")


class AnswerResult(BaseModel):
    """Structured output from synthesize_answer tool."""
    answer: str = Field(description="Generated answer")
    provider_used: str = Field(description="LLM provider used")
    metadata: dict = Field(description="Synthesis metadata")


class ValidationResult(BaseModel):
    """Structured output from validate_query tool."""
    is_valid: bool
    normalized_query: str
    error_message: Optional[str] = None


# ============================================================================
# Tool 1: Embed and Retrieve (Wraps EmbeddingService + RetrieverService)
# ============================================================================

@function_tool
async def embed_and_retrieve(
    query: Annotated[str, "User's question to embed and search for"],
    chapter: Annotated[Optional[int], "Optional chapter filter"] = None,
    top_k: Annotated[int, "Number of chunks to retrieve (default: 5)"] = 5,
) -> RetrievalResult:
    """
    Embed user query and retrieve relevant chunks from Qdrant vector database.

    This tool:
    1. Generates embedding for the query (Cohere/OpenAI)
    2. Searches Qdrant with optional metadata filters
    3. Builds formatted context with token limits (max 8000 tokens)
    4. Deduplicates chunks to avoid redundancy

    Returns structured context ready for answer synthesis.
    """
    logger.info("embed_and_retrieve tool called", {
        "query_preview": query[:100],
        "chapter": chapter,
        "top_k": top_k
    })

    # Reuse existing services (preserves all business logic)
    embedder = EmbeddingService()
    retriever = RetrieverService()

    # Step 1: Embed query
    query_embedding = await embedder.embed_text(query)

    # Step 2: Search Qdrant
    search_results = await retriever.search(
        query_vector=query_embedding,
        chapter=chapter,
        section=None,
        top_k=top_k
    )

    # Step 3: Extract chunks
    chunks = [result["payload"] for result in search_results]
    chunk_ids = [result["id"] for result in search_results]

    # Step 4: Build context with token limits (PRESERVES TOKEN EFFICIENCY)
    context = build_context_normal_rag(
        chunks,
        max_tokens=DEFAULT_MAX_CONTEXT_TOKENS  # 8000 tokens
    )

    logger.info("Retrieval completed", {
        "chunks_retrieved": len(chunks),
        "context_length": len(context)
    })

    return RetrievalResult(
        context=context,
        chunks_retrieved=len(chunks),
        chunk_ids=chunk_ids,
        metadata={
            "chapter_filter": chapter,
            "top_k_requested": top_k,
            "embedding_dim": len(query_embedding)
        }
    )


# ============================================================================
# Tool 2: Synthesize Answer (Wraps AnswerSynthesisAgent logic)
# ============================================================================

@cache_llm_responses(ttl=1800)  # Cache for 30 minutes
@function_tool
async def synthesize_answer(
    context: Annotated[str, "Retrieved or selected context"],
    query: Annotated[str, "User's question"],
    mode: Annotated[str, "Query mode: normal_rag or selected_text_only"] = "normal_rag",
) -> AnswerResult:
    """
    Generate context-bound answer using multi-LLM failover.

    This tool:
    1. Loads system prompt from specs/agent.system.md
    2. Builds mode-specific user prompt (normal_rag vs selected_text_only)
    3. Tries multiple LLM providers with automatic failover
    4. Returns answer with provider metadata

    Reuses AnswerSynthesisAgent logic for backward compatibility.
    """
    from pathlib import Path
    from openai import AsyncOpenAI, RateLimitError
    from app.config import settings

    logger.info("synthesize_answer tool called", {
        "query_preview": query[:100],
        "mode": mode,
        "context_length": len(context)
    })

    # Load system prompt (REUSES existing prompt)
    project_root = Path(__file__).parent.parent.parent
    system_prompt_path = project_root / "specs" / "agent.system.md"

    if not system_prompt_path.exists():
        raise FileNotFoundError(f"System prompt not found at {system_prompt_path}")

    with open(system_prompt_path, "r", encoding="utf-8") as f:
        system_prompt = f.read()

    # Build user prompt based on mode
    if mode == "selected_text_only":
        user_prompt = f"""[SELECTED TEXT ONLY]

{context}

---

**User Question:** {query}"""
    else:
        user_prompt = f"""Retrieved Context:
------------------
{context}

---

**User Question:** {query}"""

    # Build provider list (REUSES failover logic)
    providers = []
    if settings.llm_api_key:
        # Validate that the API key matches the provider
        is_valid, error_msg = validate_provider_configuration(
            settings.llm_api_key,
            settings.llm_provider,
            settings.llm_base_url
        )
        if not is_valid:
            logger.error(f"Primary provider configuration invalid in SDK tool: {error_msg}")
            raise ValueError(f"Primary provider configuration error in SDK tool: {error_msg}")

        providers.append({
            "name": settings.llm_provider,
            "client": AsyncOpenAI(
                api_key=settings.llm_api_key,
                base_url=settings.llm_base_url,
            ),
            "model": settings.llm_model,
        })

    if settings.llm_api_key_fallback_1:
        # Validate that the API key matches the provider
        is_valid, error_msg = validate_provider_configuration(
            settings.llm_api_key_fallback_1,
            settings.llm_provider_fallback_1,
            settings.llm_base_url_fallback_1
        )
        if not is_valid:
            logger.error(f"Fallback provider 1 configuration invalid in SDK tool: {error_msg}")
            raise ValueError(f"Fallback provider 1 configuration error in SDK tool: {error_msg}")

        providers.append({
            "name": settings.llm_provider_fallback_1,
            "client": AsyncOpenAI(
                api_key=settings.llm_api_key_fallback_1,
                base_url=settings.llm_base_url_fallback_1,
            ),
            "model": settings.llm_model_fallback_1,
        })

    if settings.llm_api_key_fallback_2:
        # Validate that the API key matches the provider
        is_valid, error_msg = validate_provider_configuration(
            settings.llm_api_key_fallback_2,
            settings.llm_provider_fallback_2,
            settings.llm_base_url_fallback_2
        )
        if not is_valid:
            logger.error(f"Fallback provider 2 configuration invalid in SDK tool: {error_msg}")
            raise ValueError(f"Fallback provider 2 configuration error in SDK tool: {error_msg}")

        providers.append({
            "name": settings.llm_provider_fallback_2,
            "client": AsyncOpenAI(
                api_key=settings.llm_api_key_fallback_2,
                base_url=settings.llm_base_url_fallback_2,
            ),
            "model": settings.llm_model_fallback_2,
        })

    # Try each provider with failover
    last_error = None
    for i, provider in enumerate(providers):
        try:
            logger.info(f"Attempting LLM call with {provider['name']} ({i+1}/{len(providers)})")

            # Acquire concurrency slot before making the API call
            limiter = get_llm_concurrency_limiter()
            async with limiter.context():
                logger.debug(
                    f"Acquired concurrency slot for {provider['name']}. "
                    f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}"
                )

                # Use retry decorator for the actual API call
                response = await _make_llm_call_with_retry(
                    provider, system_prompt, user_prompt
                )

                answer = response.choices[0].message.content
                logger.info(f"Successfully generated response using {provider['name']}")

                return AnswerResult(
                    answer=answer,
                    provider_used=provider["name"],
                    metadata={
                        "mode": mode,
                        "context_length": len(context),
                        "temperature": 0.0
                    }
                )

        except RateLimitError as e:
            last_error = e
            logger.warning(f"Rate limit on {provider['name']}, trying next...")
            # Add a small delay before trying the next provider to avoid immediate retry loops
            await asyncio.sleep(0.5)
            continue
        except Exception as e:
            last_error = e
            logger.error(f"Error with {provider['name']}: {e}")
            # Add a small delay before trying the next provider to avoid immediate retry loops
            await asyncio.sleep(0.5)
            continue

    # All providers failed
    raise Exception(f"All LLM providers failed. Last error: {str(last_error)}")


@retry_with_exponential_backoff(
    max_retries=3,
    initial_delay=1.0,
    exponential_base=2.0,
    jitter=True,
    max_delay=30.0,
    exceptions=(RateLimitError,),
    detect_429=True
)
async def _make_llm_call_with_retry(provider: dict, system_prompt: str, user_prompt: str):
    """
    Make LLM call with retry logic specifically for rate limit errors.

    Args:
        provider: Provider configuration dict
        system_prompt: System prompt to use
        user_prompt: Formatted user prompt

    Returns:
        LLM response object

    Raises:
        RateLimitError: If rate limits continue after retries
    """
    start_time = time.time()

    logger.debug(f"Making LLM call to {provider['name']}")

    response = await provider["client"].chat.completions.create(
        model=provider["model"],
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ],
        temperature=0.0,  # Deterministic for factual Q&A
        max_tokens=1000,
    )

    duration = time.time() - start_time
    logger.debug(f"LLM call completed in {duration:.2f}s")

    return response


# ============================================================================
# Tool 3: Validate Query (Optional - for explicit validation)
# ============================================================================

@function_tool
def validate_query(
    query: Annotated[str, "User's question to validate"]
) -> ValidationResult:
    """
    Validate and normalize user query.

    Checks:
    - Length (1-10,000 chars)
    - Normalizes whitespace

    Returns structured validation result.
    """
    import re

    query_stripped = query.strip()

    if len(query_stripped) == 0:
        return ValidationResult(
            is_valid=False,
            normalized_query="",
            error_message="Question cannot be empty"
        )

    if len(query_stripped) > 10000:
        return ValidationResult(
            is_valid=False,
            normalized_query="",
            error_message="Question exceeds maximum length of 10,000 characters"
        )

    # Normalize whitespace
    normalized = re.sub(r'\s+', ' ', query_stripped)

    return ValidationResult(
        is_valid=True,
        normalized_query=normalized,
        error_message=None
    )
