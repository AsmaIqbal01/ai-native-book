"""
AnswerSynthesisAgent - FR-007

Generates context-bound answers using multi-LLM failover.
Preserves all logic from original RAGAgent for backward compatibility.
"""

from pathlib import Path
from openai import AsyncOpenAI, RateLimitError
from app.agents.base_agent import BaseAgent, ValidationResult
from app.models.agent_types import SynthesisInput, SynthesisResult
from app.config import settings
import logging

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
        """Initialize synthesis agent with system prompt and LLM providers."""
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

        # Build provider list for failover
        self.providers = self._build_provider_list()
        logger.info(f"AnswerSynthesisAgent initialized with {len(self.providers)} LLM provider(s)")

    def _build_provider_list(self):
        """
        Build list of available LLM providers.

        Preserves exact logic from original RAGAgent._build_provider_list().
        Returns list of provider dicts with name, client, and model.
        """
        providers = []

        # Primary provider
        if settings.llm_api_key:
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
            providers.append({
                "name": settings.llm_provider_fallback_2,
                "client": AsyncOpenAI(
                    api_key=settings.llm_api_key_fallback_2,
                    base_url=settings.llm_base_url_fallback_2,
                ),
                "model": settings.llm_model_fallback_2,
            })

        return providers

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
        Generate answer with multi-LLM failover.

        Preserves exact failover logic from original RAGAgent.run_query().

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

        # Try each provider in sequence (exact logic from original)
        for i, provider in enumerate(self.providers):
            try:
                logger.info(
                    f"Attempting LLM call with provider: {provider['name']} ({i+1}/{len(self.providers)})"
                )

                # Call LLM via OpenAI-compatible interface
                response = await provider["client"].chat.completions.create(
                    model=provider["model"],
                    messages=[
                        {"role": "system", "content": self.system_prompt},
                        {"role": "user", "content": user_prompt},
                    ],
                    temperature=0.0,  # Deterministic for factual Q&A
                    max_tokens=1000,
                )

                answer = response.choices[0].message.content
                logger.info(f"Successfully generated response using {provider['name']}")

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
                        f"Failing over to next provider..."
                    )
                    continue  # Try next provider
                else:
                    logger.warning(f"Rate limit hit on {provider['name']}, trying next provider...")
                    continue

            except Exception as e:
                last_error = e
                logger.error(
                    f"Error with provider {provider['name']}: {str(e)}. "
                    f"Trying next provider..."
                )
                continue  # Try next provider

        # All providers failed
        logger.error(f"All {len(self.providers)} LLM provider(s) failed")
        raise Exception(f"All LLM providers failed. Last error: {str(last_error)}")

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
