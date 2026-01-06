"""
RAG Agent implementation using OpenAI-compatible LLM.

Loads system instructions from specs/agent.system.md and provides
a constitution-bound agent for answering questions.
"""

from pathlib import Path
from openai import AsyncOpenAI, RateLimitError
from app.config import settings
from app.utils.retry import retry_with_exponential_backoff
from app.utils.concurrency import get_llm_concurrency_limiter
from app.utils.api_key_validator import validate_provider_configuration
import logging
import asyncio
import time

logger = logging.getLogger(__name__)


class RAGAgent:
    """Constitution-bound RAG agent for book Q&A with multi-LLM failover."""

    def __init__(self):
        """Initialize RAG agent with system instructions and multiple LLM providers."""
        # Load system instructions from specs/agent.system.md
        # Use Path(__file__).parent to get the file's directory, then navigate to project root
        project_root = Path(__file__).parent.parent.parent
        system_prompt_path = project_root / "specs" / "agent.system.md"
        if not system_prompt_path.exists():
            raise FileNotFoundError(
                f"Agent system prompt not found at {system_prompt_path}"
            )

        with open(system_prompt_path, "r", encoding="utf-8") as f:
            self.system_prompt = f.read()

        # Build list of available LLM providers for failover
        self.providers = self._build_provider_list()
        logger.info(f"Initialized RAG agent with {len(self.providers)} LLM provider(s)")

    def _build_provider_list(self):
        """Build list of available LLM providers from configuration."""
        providers = []

        # Primary provider
        if settings.llm_api_key:
            # Validate that the API key matches the provider
            is_valid, error_msg = validate_provider_configuration(
                settings.llm_api_key,
                settings.llm_provider,
                settings.llm_base_url
            )
            if not is_valid:
                logger.error(f"Primary provider configuration invalid: {error_msg}")
                raise ValueError(f"Primary provider configuration error: {error_msg}")

            providers.append({
                "name": settings.llm_provider,
                "client": AsyncOpenAI(
                    api_key=settings.llm_api_key,
                    base_url=settings.llm_base_url,
                ),
                "model": settings.llm_model,
            })

        # Fallback provider 1
        if settings.llm_api_key_fallback_1:
            # Validate that the API key matches the provider
            is_valid, error_msg = validate_provider_configuration(
                settings.llm_api_key_fallback_1,
                settings.llm_provider_fallback_1,
                settings.llm_base_url_fallback_1
            )
            if not is_valid:
                logger.error(f"Fallback provider 1 configuration invalid: {error_msg}")
                raise ValueError(f"Fallback provider 1 configuration error: {error_msg}")

            providers.append({
                "name": settings.llm_provider_fallback_1,
                "client": AsyncOpenAI(
                    api_key=settings.llm_api_key_fallback_1,
                    base_url=settings.llm_base_url_fallback_1,
                ),
                "model": settings.llm_model_fallback_1,
            })

        # Fallback provider 2
        if settings.llm_api_key_fallback_2:
            # Validate that the API key matches the provider
            is_valid, error_msg = validate_provider_configuration(
                settings.llm_api_key_fallback_2,
                settings.llm_provider_fallback_2,
                settings.llm_base_url_fallback_2
            )
            if not is_valid:
                logger.error(f"Fallback provider 2 configuration invalid: {error_msg}")
                raise ValueError(f"Fallback provider 2 configuration error: {error_msg}")

            providers.append({
                "name": settings.llm_provider_fallback_2,
                "client": AsyncOpenAI(
                    api_key=settings.llm_api_key_fallback_2,
                    base_url=settings.llm_base_url_fallback_2,
                ),
                "model": settings.llm_model_fallback_2,
            })

        return providers

    async def run_query(
        self, user_question: str, context: str, mode: str = "normal_rag"
    ) -> dict:
        """
        Run RAG query with automatic LLM failover and rate limiting protection.

        Args:
            user_question: The user's question.
            context: Retrieved context (either from Qdrant or selected text).
            mode: Query mode ("normal_rag" or "selected_text_only").

        Returns:
            dict: {"answer": str, "provider_used": str}

        Raises:
            Exception: If all providers fail.
        """
        # Build the complete prompt with system instructions, context, and question
        user_prompt = self._build_user_prompt(user_question, context, mode)

        last_error = None

        # Try each provider in sequence
        for i, provider in enumerate(self.providers):
            try:
                logger.info(f"Attempting LLM call with provider: {provider['name']} ({i+1}/{len(self.providers)})")

                # Acquire concurrency slot before making the API call
                limiter = get_llm_concurrency_limiter()
                async with limiter.context():
                    logger.debug(
                        f"Acquired concurrency slot for {provider['name']}. "
                        f"Current usage: {limiter.current_usage}/{limiter.max_concurrent}"
                    )

                    # Use retry decorator for the actual API call
                    response = await self._make_llm_call_with_retry(
                        provider, user_prompt
                    )

                    answer = response.choices[0].message.content
                    logger.info(f"Successfully generated response using {provider['name']}")

                    return {
                        "answer": answer,
                        "provider_used": provider["name"],
                    }

            except RateLimitError as e:
                error_str = str(e).lower()
                last_error = e

                # Check if it's a quota error (not just rate limit)
                if "quota" in error_str or "insufficient" in error_str:
                    logger.warning(
                        f"Provider {provider['name']} has insufficient quota. "
                        f"Failing over to next provider..."
                    )
                    # Add a small delay before trying the next provider to avoid immediate retry loops
                    await asyncio.sleep(0.5)
                    continue  # Try next provider
                else:
                    # It's a rate limit (too many requests), not quota exhaustion
                    logger.warning(f"Rate limit hit on {provider['name']}, trying next provider...")
                    # Add a small delay before trying the next provider to avoid immediate retry loops
                    await asyncio.sleep(0.5)
                    continue

            except Exception as e:
                last_error = e
                logger.error(
                    f"Error with provider {provider['name']}: {str(e)}. "
                    f"Trying next provider..."
                )
                # Add a small delay before trying the next provider to avoid immediate retry loops
                await asyncio.sleep(0.5)
                continue  # Try next provider

        # All providers failed
        logger.error(f"All {len(self.providers)} LLM provider(s) failed")
        raise Exception(f"All LLM providers failed. Last error: {str(last_error)}")

    @retry_with_exponential_backoff(
        max_retries=3,
        initial_delay=1.0,
        exponential_base=2.0,
        jitter=True,
        max_delay=30.0,
        exceptions=(RateLimitError,)
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

        logger.debug(f"Making LLM call to {provider['name']}")

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
        logger.debug(f"LLM call completed in {duration:.2f}s")

        return response

    def _build_user_prompt(
        self, user_question: str, context: str, mode: str
    ) -> str:
        """
        Build the user prompt with context and question.

        Args:
            user_question: The user's question.
            context: Retrieved context.
            mode: Query mode.

        Returns:
            str: Formatted user prompt.
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


# Global agent instance
_agent_instance = None


def get_agent() -> RAGAgent:
    """Get or create the global RAG agent instance."""
    global _agent_instance
    if _agent_instance is None:
        _agent_instance = RAGAgent()
    return _agent_instance
