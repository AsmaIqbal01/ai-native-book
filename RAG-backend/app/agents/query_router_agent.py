"""
QueryRouterAgent - FR-005

Validates input, detects greetings/out-of-scope queries, and normalizes text.
Routes queries to appropriate handlers (RAG vs greeting vs rejection).
"""

from app.agents.base_agent import BaseAgent, ValidationResult
from app.models.agent_types import RouterInput, RouterResult
from app.utils.mode_detector import detect_mode
import re
import logging

logger = logging.getLogger(__name__)


class QueryRouterAgent(BaseAgent[RouterInput, RouterResult]):
    """
    Agent responsible for query routing and validation.

    Responsibilities:
    - Validate question length (1-10,000 chars)
    - Detect greetings ("hello", "hi", "hey")
    - Normalize text (strip, lowercase for analysis)
    - Detect out-of-scope queries (weather, politics, etc.)
    - Determine mode (normal_rag vs selected_text_only)
    """

    # Greeting patterns
    GREETING_PATTERNS = [
        r"^hi\b",
        r"^hello\b",
        r"^hey\b",
        r"^greetings\b",
        r"^good morning\b",
        r"^good afternoon\b",
        r"^good evening\b",
    ]

    # Out-of-scope keywords (topics not in documentation)
    OUT_OF_SCOPE_KEYWORDS = [
        "weather",
        "stock market",
        "politics",
        "sports",
        "recipe",
        "movie",
        "music",
        "celebrity",
    ]

    async def validate_input(self, input_data: RouterInput) -> ValidationResult:
        """
        Validate query input.

        Args:
            input_data: RouterInput with question and optional filters

        Returns:
            ValidationResult with validation status
        """
        question = input_data.question.strip()

        # Check length
        if len(question) == 0:
            return ValidationResult(
                is_valid=False,
                error_message="Question cannot be empty"
            )

        if len(question) > 10000:
            return ValidationResult(
                is_valid=False,
                error_message="Question exceeds maximum length of 10,000 characters"
            )

        return ValidationResult(is_valid=True, normalized_input=input_data)

    async def execute(self, input_data: RouterInput) -> RouterResult:
        """
        Route query to appropriate handler.

        Args:
            input_data: Validated RouterInput

        Returns:
            RouterResult with mode, normalized query, and validation status
        """
        question = input_data.question.strip()
        normalized_query = self._normalize_text(question)

        # Step 1: Detect greetings
        if self._is_greeting(normalized_query):
            logger.info("Greeting detected", extra={"query": question[:100]})
            return RouterResult(
                is_valid=True,
                mode="greeting",
                normalized_query=normalized_query,
                metadata={
                    "greeting_detected": True,
                    "original_query": question
                }
            )

        # Step 2: Detect out-of-scope queries
        if self._is_out_of_scope(normalized_query):
            logger.info("Out-of-scope query detected", extra={"query": question[:100]})
            return RouterResult(
                is_valid=False,
                mode="out_of_scope",
                normalized_query=normalized_query,
                error="This question is outside the scope of the documentation. Please ask about robotics, ROS2, or related topics.",
                metadata={
                    "out_of_scope": True
                }
            )

        # Step 3: Detect mode (normal_rag vs selected_text_only)
        mode = detect_mode(input_data.selected_text)

        logger.info(
            "Query routed successfully",
            extra={
                "mode": mode,
                "has_chapter_filter": input_data.chapter is not None,
                "query_preview": question[:100]
            }
        )

        return RouterResult(
            is_valid=True,
            mode=mode,
            normalized_query=normalized_query,
            metadata={
                "original_query": question,
                "chapter_filter": input_data.chapter,
            }
        )

    def _normalize_text(self, text: str) -> str:
        """
        Normalize text for analysis.

        Args:
            text: Raw input text

        Returns:
            Normalized text (lowercased, extra spaces removed)
        """
        # Remove extra whitespace
        normalized = re.sub(r'\s+', ' ', text.strip())
        return normalized

    def _is_greeting(self, text: str) -> bool:
        """
        Check if text is a greeting.

        Args:
            text: Normalized text

        Returns:
            True if greeting detected
        """
        text_lower = text.lower()

        # Check against greeting patterns
        for pattern in self.GREETING_PATTERNS:
            if re.search(pattern, text_lower):
                # Ensure it's not a complex question starting with greeting
                # "Hello, how do I..." should not be classified as greeting
                if len(text.split()) <= 3:  # Pure greetings are typically 1-3 words
                    return True

        return False

    def _is_out_of_scope(self, text: str) -> bool:
        """
        Check if query is out of scope.

        Args:
            text: Normalized text

        Returns:
            True if out-of-scope keywords detected
        """
        text_lower = text.lower()

        # Check for out-of-scope keywords
        for keyword in self.OUT_OF_SCOPE_KEYWORDS:
            if keyword in text_lower:
                return True

        return False
