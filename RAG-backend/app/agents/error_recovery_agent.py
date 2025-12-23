"""
ErrorRecoveryAgent - FR-008

Handles all error classification, conversion to user-safe messages,
and HTTP status code mapping. Preserves technical details for server-side logging.
"""

from app.agents.base_agent import BaseAgent, ValidationResult
from app.models.agent_types import ErrorClassification, ErrorRecoveryInput
from openai import RateLimitError
from pydantic import ValidationError
import asyncio
import logging

logger = logging.getLogger(__name__)


class ErrorRecoveryAgent(BaseAgent[ErrorRecoveryInput, ErrorClassification]):
    """
    Agent responsible for error classification and recovery.

    Classifies errors into:
    - connection: Qdrant/LLM service unavailable
    - timeout: Request exceeded time limit
    - validation: Invalid input parameters
    - quota: LLM quota exhausted
    - rate_limit: Too many requests
    - unknown: Unexpected errors

    Converts technical errors to user-friendly messages while
    preserving full details for server-side logging.
    """

    async def validate_input(self, input_data: ErrorRecoveryInput) -> ValidationResult:
        """Validate error recovery input (always valid)."""
        return ValidationResult(is_valid=True, normalized_input=input_data)

    async def execute(self, input_data: ErrorRecoveryInput) -> ErrorClassification:
        """
        Classify error and generate user-safe message.

        Args:
            input_data: ErrorRecoveryInput with exception and context

        Returns:
            ErrorClassification with status code, user message, and type
        """
        exception = input_data.exception
        context = input_data.context

        # Get technical error details
        technical_details = f"{type(exception).__name__}: {str(exception)}"

        # Classify error type and map to HTTP status + message
        classification = self._classify_error(exception, technical_details)

        # Log error with context
        logger.error(
            "Error classified by ErrorRecoveryAgent",
            extra={
                "error_type": classification.error_type,
                "status_code": classification.status_code,
                "context": context,
                "technical_details": technical_details
            }
        )

        return classification

    def _classify_error(self, exception: Exception, technical_details: str) -> ErrorClassification:
        """
        Classify exception into error taxonomy.

        Args:
            exception: The caught exception
            technical_details: Technical error string

        Returns:
            ErrorClassification with mapped status code and message
        """
        error_str = str(exception).lower()

        # Connection errors (Qdrant/LLM unavailable)
        if isinstance(exception, ConnectionError):
            return ErrorClassification(
                error_type="connection",
                status_code=503,
                user_message="The service is temporarily unavailable. Please try again later.",
                technical_details=technical_details,
                should_retry=True
            )

        # Timeout errors
        if isinstance(exception, asyncio.TimeoutError):
            return ErrorClassification(
                error_type="timeout",
                status_code=504,
                user_message="The request timed out. Please try again.",
                technical_details=technical_details,
                should_retry=True
            )

        # Validation errors
        if isinstance(exception, (ValidationError, ValueError)):
            return ErrorClassification(
                error_type="validation",
                status_code=400,
                user_message=f"Invalid input: {str(exception)}",
                technical_details=technical_details,
                should_retry=False
            )

        # Rate limit errors (quota vs rate)
        if isinstance(exception, RateLimitError):
            # Check if it's quota exhaustion or rate limiting
            if "quota" in error_str or "insufficient" in error_str:
                return ErrorClassification(
                    error_type="quota",
                    status_code=429,
                    user_message="LLM quota exceeded. Please try again later.",
                    technical_details=technical_details,
                    should_retry=True
                )
            else:
                return ErrorClassification(
                    error_type="rate_limit",
                    status_code=429,
                    user_message="Too many requests. Please try again in a moment.",
                    technical_details=technical_details,
                    should_retry=True
                )

        # Check error string for rate limit indicators
        if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
            return ErrorClassification(
                error_type="rate_limit",
                status_code=429,
                user_message="Rate limit exceeded. Please try again later.",
                technical_details=technical_details,
                should_retry=True
            )

        # Unknown/unexpected errors
        return ErrorClassification(
            error_type="unknown",
            status_code=500,
            user_message="An unexpected error occurred. Please try again or contact support.",
            technical_details=technical_details,
            should_retry=False
        )
