"""
Unit tests for ErrorRecoveryAgent (FR-008)

Tests error classification, user message generation, and retry logic.
"""

import pytest
import asyncio
from unittest.mock import Mock
from openai import RateLimitError
from pydantic import ValidationError

from app.agents.error_recovery_agent import ErrorRecoveryAgent
from app.models.agent_types import ErrorRecoveryInput, ErrorClassification


# ============================================================================
# Error Classification Tests
# ============================================================================

@pytest.mark.asyncio
async def test_classify_connection_error(sample_connection_error):
    """Test ConnectionError classified as connection error with 503."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=sample_connection_error,
        context={"agent": "QdrantRetrievalAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "connection"
    assert result.status_code == 503
    assert result.should_retry is True
    assert "unavailable" in result.user_message.lower()
    assert "Qdrant" in result.technical_details


@pytest.mark.asyncio
async def test_classify_timeout_error(sample_timeout_error):
    """Test TimeoutError classified as timeout error with 504."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=sample_timeout_error,
        context={"agent": "QdrantRetrievalAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "timeout"
    assert result.status_code == 504
    assert result.should_retry is True
    assert "timed out" in result.user_message.lower()
    assert "30 seconds" in result.technical_details


@pytest.mark.asyncio
async def test_classify_validation_error():
    """Test ValidationError classified as validation error with 400."""
    # Create a real ValidationError
    try:
        from app.models.agent_types import RouterInput
        RouterInput(question="", selected_text=None, chapter=None)
    except ValidationError as e:
        validation_error = e

    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=validation_error,
        context={"agent": "QueryRouterAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "validation"
    assert result.status_code == 400
    assert result.should_retry is False
    assert "invalid" in result.user_message.lower()


@pytest.mark.asyncio
async def test_classify_rate_limit_error(sample_rate_limit_error):
    """Test RateLimitError classified as rate_limit error with 429."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=sample_rate_limit_error,
        context={"agent": "AnswerSynthesisAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "rate_limit"
    assert result.status_code == 429
    assert result.should_retry is True
    # Check for "too many requests" or "rate limit" in message
    assert ("too many requests" in result.user_message.lower() or
            "rate limit" in result.user_message.lower())


@pytest.mark.asyncio
async def test_classify_quota_error(sample_quota_error):
    """Test quota exhaustion classified as quota error with 429."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=sample_quota_error,
        context={"agent": "AnswerSynthesisAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "quota"
    assert result.status_code == 429
    # Note: Agent allows retry for quota errors (quota may renew)
    assert result.should_retry is True
    assert "quota" in result.user_message.lower()


@pytest.mark.asyncio
async def test_classify_generic_exception():
    """Test generic Exception classified as unknown error with 500."""
    agent = ErrorRecoveryAgent()
    generic_error = Exception("Something unexpected happened")
    error_input = ErrorRecoveryInput(
        exception=generic_error,
        context={"agent": "RAGChatAgent"},
    )

    result = await agent.execute(error_input)

    assert result.error_type == "unknown"
    assert result.status_code == 500
    assert result.should_retry is False
    assert "unexpected error" in result.user_message.lower()


# ============================================================================
# Retry Logic Tests
# ============================================================================

@pytest.mark.asyncio
async def test_connection_error_should_retry():
    """Test connection errors marked as retriable."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=ConnectionError("Network unreachable"),
        context={},
    )

    result = await agent.execute(error_input)

    assert result.should_retry is True


@pytest.mark.asyncio
async def test_timeout_error_should_retry():
    """Test timeout errors marked as retriable."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=asyncio.TimeoutError(),
        context={},
    )

    result = await agent.execute(error_input)

    assert result.should_retry is True


@pytest.mark.asyncio
async def test_validation_error_should_not_retry():
    """Test validation errors marked as non-retriable."""
    agent = ErrorRecoveryAgent()

    # Create a real ValidationError by violating top_k constraint
    validation_error = None
    try:
        from app.models.agent_types import RetrievalInput
        # top_k must be >= 1 and <= 20, so 100 will fail validation
        RetrievalInput(query="test", mode="normal_rag", top_k=100)
    except ValidationError as e:
        validation_error = e

    assert validation_error is not None, "ValidationError should be raised"

    error_input = ErrorRecoveryInput(
        exception=validation_error,
        context={},
    )

    result = await agent.execute(error_input)

    assert result.should_retry is False
    assert result.error_type == "validation"


@pytest.mark.asyncio
async def test_quota_error_can_retry():
    """Test quota errors marked as retriable (quota may renew)."""
    agent = ErrorRecoveryAgent()
    quota_error = RateLimitError(
        message="Insufficient quota",
        response=Mock(status_code=429),
        body={"error": {"message": "quota exceeded"}},
    )

    error_input = ErrorRecoveryInput(
        exception=quota_error,
        context={},
    )

    result = await agent.execute(error_input)

    # Agent allows retry for quota errors (quota may renew later)
    assert result.should_retry is True
    assert result.error_type == "quota"


# ============================================================================
# User Message Sanitization Tests
# ============================================================================

@pytest.mark.asyncio
async def test_user_message_no_sensitive_data():
    """Test user messages don't leak sensitive technical details."""
    agent = ErrorRecoveryAgent()
    error_with_secrets = Exception(
        "API key sk-1234567890 failed at endpoint https://api.openai.com/v1/chat"
    )

    error_input = ErrorRecoveryInput(
        exception=error_with_secrets,
        context={},
    )

    result = await agent.execute(error_input)

    # User message should be generic
    assert "sk-" not in result.user_message
    assert "api.openai.com" not in result.user_message
    assert "unexpected error" in result.user_message.lower()

    # Technical details should contain full info
    assert "sk-1234567890" in result.technical_details
    assert "api.openai.com" in result.technical_details


@pytest.mark.asyncio
async def test_user_message_friendly_language():
    """Test user messages use friendly, non-technical language."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=ConnectionError("Socket connection refused at 127.0.0.1:6333"),
        context={},
    )

    result = await agent.execute(error_input)

    # Should use friendly language
    assert "unavailable" in result.user_message.lower() or "try again" in result.user_message.lower()

    # Should not use technical jargon in user message
    assert "socket" not in result.user_message.lower()
    assert "127.0.0.1" not in result.user_message


@pytest.mark.asyncio
async def test_technical_details_contain_exception_info():
    """Test technical_details contains exception type and message."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=ValueError("Invalid embedding dimension: expected 1536, got 512"),
        context={"agent": "EmbeddingService", "operation": "embed_text"},
    )

    result = await agent.execute(error_input)

    # Technical details should include:
    # - Exception type
    # - Exception message
    assert "ValueError" in result.technical_details
    assert "embedding dimension" in result.technical_details
    assert "1536" in result.technical_details


# ============================================================================
# Context Propagation Tests
# ============================================================================

@pytest.mark.asyncio
async def test_context_available_for_logging():
    """Test context is provided to error recovery agent (for logging/monitoring)."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=Exception("Test error"),
        context={
            "agent": "QdrantRetrievalAgent",
            "operation": "search",
            "query": "What is ROS 2?",
            "chapter": "Chapter 1",
        },
    )

    result = await agent.execute(error_input)

    # Result should contain exception info
    assert "Exception" in result.technical_details
    assert "Test error" in result.technical_details
    # Context is logged (checked via logger output, not in technical_details field)


# ============================================================================
# Edge Cases
# ============================================================================

@pytest.mark.asyncio
async def test_error_with_empty_context():
    """Test error handling with empty context."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=Exception("Test error"),
        context={},
    )

    result = await agent.execute(error_input)

    assert isinstance(result, ErrorClassification)
    assert result.error_type == "unknown"
    assert result.status_code == 500


@pytest.mark.asyncio
async def test_error_with_none_message():
    """Test error handling when exception has no message."""
    agent = ErrorRecoveryAgent()

    class CustomError(Exception):
        pass

    error_input = ErrorRecoveryInput(
        exception=CustomError(),
        context={},
    )

    result = await agent.execute(error_input)

    assert isinstance(result, ErrorClassification)
    assert result.user_message  # Should have a default message


@pytest.mark.asyncio
async def test_rate_limit_vs_quota_distinction():
    """Test distinction between rate limit and quota errors."""
    agent = ErrorRecoveryAgent()

    # Rate limit error (should retry)
    rate_limit = RateLimitError(
        message="Rate limit exceeded",
        response=Mock(status_code=429),
        body={"error": {"message": "rate limit exceeded"}},
    )

    rate_limit_input = ErrorRecoveryInput(exception=rate_limit, context={})
    rate_limit_result = await agent.execute(rate_limit_input)

    # Quota error (should also retry - quota may renew)
    quota = RateLimitError(
        message="Insufficient quota",
        response=Mock(status_code=429),
        body={"error": {"message": "insufficient quota"}},
    )

    quota_input = ErrorRecoveryInput(exception=quota, context={})
    quota_result = await agent.execute(quota_input)

    # Both should retry, but be classified differently
    assert rate_limit_result.error_type == "rate_limit"
    assert rate_limit_result.should_retry is True

    assert quota_result.error_type == "quota"
    assert quota_result.should_retry is True  # Agent allows retry for both


# ============================================================================
# Integration with Validation Tests
# ============================================================================

@pytest.mark.asyncio
async def test_validate_input_always_valid():
    """Test ErrorRecoveryAgent accepts any ErrorRecoveryInput (no validation failure)."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=Exception("Test"),
        context={},
    )

    validation_result = await agent.validate_input(error_input)

    assert validation_result.is_valid is True


@pytest.mark.asyncio
async def test_run_method_executes_successfully():
    """Test run() method successfully validates and executes."""
    agent = ErrorRecoveryAgent()
    error_input = ErrorRecoveryInput(
        exception=ConnectionError("Test connection error"),
        context={},
    )

    result = await agent.run(error_input)

    assert isinstance(result, ErrorClassification)
    assert result.error_type == "connection"
    assert result.status_code == 503
