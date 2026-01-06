"""
AnswerSynthesisAgent - FR-007

Generates context-bound answers using OpenRouter with Mistral model.
This fixes the 401 authentication error by using OpenRouter exclusively instead of mixed provider configuration.
"""

from pathlib import Path
from openai import AsyncOpenAI, RateLimitError
from app.agents.base_agent import BaseAgent, ValidationResult
from app.models.agent_types import SynthesisInput, SynthesisResult
from app.config import settings
from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
from app.utils.api_key_validator import validate_provider_configuration
from app.utils.provider_logger import get_provider_logger
import logging
import asyncio
import time

logger = logging.getLogger(__name__)


class AnswerSynthesisAgent(BaseAgent[SynthesisInput, SynthesisResult]):
    """
    Agent responsible for answer generation with multi-LLM failover.

    Responsibilities:
    - Load system prompt (specs/agent.system.md)
    - Build user prompt with context
    - Try multiple LLM providers with automatic failover
    - Enforce "no hallucination" via system prompt
    - Generate citations from chunks

    Preserves logic from original RAGAgent:
    - Multi-provider failover (Primary → Fallback 1 → Fallback 2)
    - System prompt loading from file
    - User prompt building (mode-specific formatting)
    - Temperature=0.0 for factual Q&A
    """

    def __init__(self):
        """Initialize synthesis agent with system prompt and OpenRouter LLM provider."""
        super().__init__()

        # Load system prompt from specs/agent.system.md
        # Preserve exact path resolution from original RAGAgent
        project_root = Path(__file__).parent.parent.parent
        system_prompt_path = project_root / "specs" / "agent.system.md"

        if not system_prompt_path.exists():
            raise FileNotFoundError(
                f"Agent system prompt not found at {system_prompt_path}"
            )

        with open(system_prompt_path, "r", encoding="utf-8") as f:
            self.system_prompt = f.read()

        # Build single OpenRouter provider configuration using settings
        self.providers = self._build_openrouter_provider()
        logger.info(f"AnswerSynthesisAgent initialized with OpenRouter provider using Mistral model")

    def _build_openrouter_provider(self):
        """
        Build OpenRouter provider configuration with Mistral model.

        This method specifically configures OpenRouter to avoid API key mismatches
        that cause 401 authentication errors when using mixed provider configurations.
        """
        # Validate that the primary LLM API key is available (this will be the OpenRouter key)
        if not settings.llm_api_key:
            raise ValueError(
                "LLM_API_KEY environment variable is required for OpenRouter. "
                "Please set OPENROUTER_API_KEY in your .env file and ensure LLM_API_KEY references it."
            )

        # Use comprehensive validation to ensure API key matches provider
        is_valid, error_msg = validate_provider_configuration(
            settings.llm_api_key,
            settings.llm_provider,
            settings.llm_base_url
        )
        if not is_valid:
            logger.error(f"Primary provider configuration invalid: {error_msg}")
            raise ValueError(f"Primary provider configuration error: {error_msg}")

        provider = {
            "name": "openrouter-mistral",
            "client": AsyncOpenAI(
                api_key=settings.llm_api_key,
                base_url=settings.llm_base_url,
            ),
            "model": settings.llm_model,
        }

        return [provider]  # Return as list to maintain compatibility with existing execution logic

    async def validate_input(self, input_data: SynthesisInput) -> ValidationResult:
        """
        Validate synthesis input.

        Args:
            input_data: SynthesisInput with context and query

        Returns:
            ValidationResult
        """
        if not input_data.context or input_data.context.strip() == "":
            return ValidationResult(
                is_valid=False,
                error_message="Context cannot be empty for answer synthesis"
            )

        if not input_data.query or input_data.query.strip() == "":
            return ValidationResult(
                is_valid=False,
                error_message="Query cannot be empty for answer synthesis"
            )

        return ValidationResult(is_valid=True, normalized_input=input_data)

    async def execute(self, input_data: SynthesisInput) -> SynthesisResult:
        """
        Generate answer with multi-LLM failover and rate limiting protection.

        Args:
            input_data: SynthesisInput with context, query, mode

        Returns:
            SynthesisResult with answer, provider, citations

        Raises:
            Exception: If all providers fail (caught by ErrorRecoveryAgent)
        """
        # Build user prompt (mode-specific formatting)
        user_prompt = self._build_user_prompt(
            input_data.query,
            input_data.context,
            input_data.mode
        )

        last_error = None

        # Try each provider in sequence
        for i, provider in enumerate(self.providers):
            try:
                logger.info(
                    f"Attempting LLM call with provider: {provider['name']} ({i+1}/{len(self.providers)})",
                    extra={'provider': provider['name'], 'operation': 'llm_call_attempt'}
                )

                # Acquire concurrency slot before making the API call
                limiter = get_llm_concurrency_limiter()
                async with limiter.context():
                    logger.debug(
                        f"Acquired concurrency slot for {provider['name']}. "
                        f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}",
                        extra={'provider': provider['name'], 'concurrent_usage': limiter.current_usage}
                    )

                    # Use retry decorator for the actual API call
                    response = await self._make_llm_call_with_retry(
                        provider, user_prompt
                    )

                    answer = response.choices[0].message.content
                    logger.info(
                        f"Successfully generated response using {provider['name']}",
                        extra={'provider': provider['name'], 'operation': 'llm_call_success'}
                    )

                    # Generate citations from chunks
                    citations = self._generate_citations(input_data.chunks)

                    return SynthesisResult(
                        answer=answer,
                        provider_used=provider["name"],
                        citations=citations,
                        metadata={
                            "mode": input_data.mode,
                            "context_length": len(input_data.context),
                            "chunks_used": len(input_data.chunks)
                        }
                    )

            except RateLimitError as e:
                error_str = str(e).lower()
                last_error = e

                # Check if it's quota exhaustion or rate limit
                if "quota" in error_str or "insufficient" in error_str:
                    logger.warning(
                        f"Provider {provider['name']} has insufficient quota. "
                        f"Failing over to next provider...",
                        extra={'provider': provider['name'], 'error_type': 'quota_exceeded'}
                    )
                    continue  # Try next provider
                else:
                    logger.warning(
                        f"Rate limit hit on {provider['name']}, trying next provider...",
                        extra={'provider': provider['name'], 'error_type': 'rate_limit'}
                    )
                    # Add a small delay before trying the next provider to avoid immediate retry loops
                    await asyncio.sleep(0.5)
                    continue

            except Exception as e:
                last_error = e
                logger.error(
                    f"Error with provider {provider['name']}: {str(e)}. "
                    f"Trying next provider...",
                    extra={'provider': provider['name'], 'error_type': type(e).__name__, 'error_message': str(e)}
                )
                # Add a small delay before trying the next provider to avoid immediate retry loops
                await asyncio.sleep(0.5)
                continue  # Try next provider

        # All providers failed (in our case, OpenRouter failed)
        logger.error(
            f"OpenRouter LLM provider failed: {str(last_error)}",
            extra={'provider': 'openrouter-mistral', 'error_type': type(last_error).__name__}
        )
        raise Exception(f"OpenRouter LLM provider failed. Error: {str(last_error)}")

    @retry_with_exponential_backoff(
        max_retries=3,
        initial_delay=1.0,
        exponential_base=2.0,
        jitter=True,
        max_delay=30.0,
        exceptions=(RateLimitError, Exception),  # Include more exception types
        detect_429=True,
        detect_auth_errors=True,  # Enable authentication error detection
        detect_network_errors=True  # Enable network error detection
    )
    async def _make_llm_call_with_retry(self, provider: dict, user_prompt: str):
        """
        Make LLM call with retry logic specifically for rate limit errors.

        Args:
            provider: Provider configuration dict
            user_prompt: Formatted user prompt

        Returns:
            LLM response object

        Raises:
            RateLimitError: If rate limits continue after retries
        """
        start_time = time.time()

        logger.debug(
            f"Making LLM call to {provider['name']}",
            extra={'provider': provider['name'], 'operation': 'llm_request_start'}
        )

        response = await provider["client"].chat.completions.create(
            model=provider["model"],
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.0,  # Deterministic for factual Q&A
            max_tokens=1000,
        )

        duration = time.time() - start_time
        logger.debug(
            f"LLM call completed in {duration:.2f}s",
            extra={'provider': provider['name'], 'operation': 'llm_request_complete', 'duration': duration}
        )

        return response

    def _build_user_prompt(self, user_question: str, context: str, mode: str) -> str:
        """
        Build user prompt with context and question.

        Preserves exact logic from original RAGAgent._build_user_prompt().

        Args:
            user_question: User's question
            context: Retrieved or selected context
            mode: Query mode

        Returns:
            Formatted user prompt
        """
        if mode == "selected_text_only":
            # Selected text mode - mark context as selected text only
            return f"""[SELECTED TEXT ONLY]

{context}

---

**User Question:** {user_question}"""
        else:
            # Normal RAG mode - provide retrieved context
            return f"""Retrieved Context:
------------------
{context}

---

**User Question:** {user_question}"""

    def _generate_citations(self, chunks: list) -> list:
        """
        Generate citations from retrieved chunks.

        Args:
            chunks: List of chunk payloads

        Returns:
            List of citation dicts with chapter, section, page
        """
        citations = []

        for chunk in chunks:
            citations.append({
                "chapter": chunk.get("chapter"),
                "section": chunk.get("section"),
                "page": chunk.get("page"),
                "chunk_text": chunk.get("chunk_text", "")[:200],  # Truncate for response
            })

        return citations
