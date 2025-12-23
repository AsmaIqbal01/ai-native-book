"""
Pytest fixtures and configuration for RAG backend tests.

Provides reusable mocks and fixtures for testing multi-agent system.
"""

import pytest
from unittest.mock import AsyncMock, Mock, MagicMock, patch
from typing import Dict, Any, List
from openai import AsyncOpenAI
from openai.types.chat import ChatCompletion, ChatCompletionMessage
from openai.types.chat.chat_completion import Choice

# ============================================================================
# Mock Data Fixtures
# ============================================================================

@pytest.fixture
def sample_question():
    """Sample user question for testing."""
    return "What is ROS 2 and how does it differ from ROS 1?"


@pytest.fixture
def sample_selected_text():
    """Sample selected text for testing."""
    return """
    ROS 2 (Robot Operating System 2) is a complete redesign of ROS 1,
    addressing limitations in security, real-time performance, and multi-robot
    support. Unlike ROS 1, ROS 2 uses DDS for communication.
    """


@pytest.fixture
def sample_chunks() -> List[Dict[str, Any]]:
    """Sample Qdrant chunks for testing."""
    return [
        {
            "chunk_id": "chunk-001",
            "chunk_text": "ROS 2 is the next generation of ROS, designed for production robots.",
            "chapter": "Chapter 1",
            "section": "Introduction to ROS 2",
            "page": 5,
            "embedding": [0.1] * 1536,
        },
        {
            "chunk_id": "chunk-002",
            "chunk_text": "Unlike ROS 1, ROS 2 provides real-time communication using DDS.",
            "chapter": "Chapter 1",
            "section": "ROS 2 Architecture",
            "page": 12,
            "embedding": [0.2] * 1536,
        },
        {
            "chunk_id": "chunk-003",
            "chunk_text": "ROS 2 supports multiple robot coordination through DDS discovery.",
            "chapter": "Chapter 2",
            "section": "Multi-Robot Systems",
            "page": 34,
            "embedding": [0.3] * 1536,
        },
    ]


@pytest.fixture
def sample_qdrant_search_results(sample_chunks) -> List[Dict[str, Any]]:
    """Sample Qdrant search results for testing."""
    return [
        {
            "id": chunk["chunk_id"],
            "score": 0.95 - (i * 0.1),
            "payload": chunk,
        }
        for i, chunk in enumerate(sample_chunks)
    ]


@pytest.fixture
def sample_context():
    """Sample context string for testing."""
    return """
Chunk 1 (Chapter 1, Section: Introduction to ROS 2, Page: 5):
ROS 2 is the next generation of ROS, designed for production robots.

Chunk 2 (Chapter 1, Section: ROS 2 Architecture, Page: 12):
Unlike ROS 1, ROS 2 provides real-time communication using DDS.

Chunk 3 (Chapter 2, Section: Multi-Robot Systems, Page: 34):
ROS 2 supports multiple robot coordination through DDS discovery.
    """.strip()


@pytest.fixture
def sample_llm_response():
    """Sample LLM response for testing."""
    return """
ROS 2 (Robot Operating System 2) is the next generation of the ROS framework,
designed to address limitations in ROS 1. Key differences include:

1. **Communication**: ROS 2 uses DDS (Data Distribution Service) instead of
   custom protocols, providing better real-time performance.
2. **Multi-robot support**: Native support for multi-robot coordination.
3. **Security**: Built-in authentication and encryption.
4. **Production-ready**: Designed for commercial robot deployments.

Based on the documentation, ROS 2 is a complete redesign focused on
industrial and commercial applications.
    """.strip()


# ============================================================================
# Service Mocks
# ============================================================================

@pytest.fixture
def mock_embedding_service():
    """Mock EmbeddingService for testing."""
    mock = AsyncMock()
    mock.embed_text = AsyncMock(return_value=[0.1] * 1536)
    return mock


@pytest.fixture
def mock_retriever_service(sample_qdrant_search_results):
    """Mock RetrieverService for testing."""
    mock = AsyncMock()
    mock.search = AsyncMock(return_value=sample_qdrant_search_results)
    return mock


@pytest.fixture
def mock_context_builder(sample_context):
    """Mock context builder functions."""
    return Mock(return_value=sample_context)


# ============================================================================
# OpenAI Client Mocks
# ============================================================================

@pytest.fixture
def mock_openai_response(sample_llm_response):
    """Mock OpenAI ChatCompletion response."""
    return ChatCompletion(
        id="chatcmpl-test-123",
        object="chat.completion",
        created=1234567890,
        model="gpt-4",
        choices=[
            Choice(
                index=0,
                message=ChatCompletionMessage(
                    role="assistant",
                    content=sample_llm_response,
                ),
                finish_reason="stop",
            )
        ],
        usage={
            "prompt_tokens": 150,
            "completion_tokens": 120,
            "total_tokens": 270,
        },
    )


@pytest.fixture
def mock_openai_client(mock_openai_response):
    """Mock AsyncOpenAI client for testing."""
    mock_client = AsyncMock(spec=AsyncOpenAI)
    mock_client.chat = AsyncMock()
    mock_client.chat.completions = AsyncMock()
    mock_client.chat.completions.create = AsyncMock(return_value=mock_openai_response)
    return mock_client


@pytest.fixture
def mock_openai_client_with_failover(sample_llm_response):
    """Mock AsyncOpenAI client that fails on first call, succeeds on second."""
    mock_client = AsyncMock(spec=AsyncOpenAI)
    mock_client.chat = AsyncMock()
    mock_client.chat.completions = AsyncMock()

    # First call fails, second succeeds
    mock_client.chat.completions.create = AsyncMock(
        side_effect=[
            Exception("Primary provider failed"),
            ChatCompletion(
                id="chatcmpl-test-456",
                object="chat.completion",
                created=1234567890,
                model="gpt-3.5-turbo",
                choices=[
                    Choice(
                        index=0,
                        message=ChatCompletionMessage(
                            role="assistant",
                            content=sample_llm_response,
                        ),
                        finish_reason="stop",
                    )
                ],
                usage={"prompt_tokens": 150, "completion_tokens": 120, "total_tokens": 270},
            ),
        ]
    )
    return mock_client


# ============================================================================
# Agent Input/Output Fixtures
# ============================================================================

@pytest.fixture
def sample_router_input(sample_question):
    """Sample RouterInput for testing."""
    from app.models.agent_types import RouterInput
    return RouterInput(
        question=sample_question,
        selected_text=None,
        chapter=None,
    )


@pytest.fixture
def sample_router_result():
    """Sample RouterResult for testing."""
    from app.models.agent_types import RouterResult
    return RouterResult(
        is_valid=True,
        mode="normal_rag",
        normalized_query="What is ROS 2 and how does it differ from ROS 1?",
        error=None,
        metadata={},
    )


@pytest.fixture
def sample_retrieval_input():
    """Sample RetrievalInput for testing."""
    from app.models.agent_types import RetrievalInput
    return RetrievalInput(
        query="What is ROS 2 and how does it differ from ROS 1?",
        mode="normal_rag",
        chapter=None,
        section=None,
        top_k=5,
    )


@pytest.fixture
def sample_retrieval_result(sample_chunks, sample_context):
    """Sample RetrievalResult for testing."""
    from app.models.agent_types import RetrievalResult
    return RetrievalResult(
        chunks=sample_chunks,
        chunk_ids=["chunk-001", "chunk-002", "chunk-003"],
        context=sample_context,
        metadata={"chunks_retrieved": 3},
        error=None,
    )


@pytest.fixture
def sample_synthesis_input(sample_context):
    """Sample SynthesisInput for testing."""
    from app.models.agent_types import SynthesisInput
    return SynthesisInput(
        context=sample_context,
        query="What is ROS 2 and how does it differ from ROS 1?",
        mode="normal_rag",
        chunks=[],
    )


@pytest.fixture
def sample_synthesis_result(sample_llm_response):
    """Sample SynthesisResult for testing."""
    from app.models.agent_types import SynthesisResult
    return SynthesisResult(
        answer=sample_llm_response,
        provider_used="OpenAI",
        citations=[],
        metadata={"mode": "normal_rag", "context_length": 300},
        error=None,
    )


# ============================================================================
# Error Fixtures
# ============================================================================

@pytest.fixture
def sample_connection_error():
    """Sample ConnectionError for testing."""
    return ConnectionError("Failed to connect to Qdrant at localhost:6333")


@pytest.fixture
def sample_timeout_error():
    """Sample TimeoutError for testing."""
    import asyncio
    return asyncio.TimeoutError("Request timed out after 30 seconds")


@pytest.fixture
def sample_validation_error():
    """Sample ValidationError for testing."""
    from pydantic import ValidationError
    try:
        # Force a validation error
        from app.models.agent_types import RouterInput
        RouterInput(question="", selected_text=None, chapter=None)
    except ValidationError as e:
        return e


@pytest.fixture
def sample_rate_limit_error():
    """Sample RateLimitError for testing."""
    from openai import RateLimitError
    return RateLimitError(
        message="Rate limit exceeded. Please try again later.",
        response=Mock(status_code=429),
        body={"error": {"message": "Rate limit exceeded"}},
    )


@pytest.fixture
def sample_quota_error():
    """Sample quota exhaustion error for testing."""
    from openai import RateLimitError
    return RateLimitError(
        message="You have insufficient quota for this request.",
        response=Mock(status_code=429),
        body={"error": {"message": "insufficient quota"}},
    )


# ============================================================================
# Agent Mocks (for orchestrator testing)
# ============================================================================

@pytest.fixture
def mock_query_router_agent(sample_router_result):
    """Mock QueryRouterAgent for orchestrator testing."""
    mock = AsyncMock()
    mock.run = AsyncMock(return_value=sample_router_result)
    mock.validate_input = AsyncMock(return_value=Mock(is_valid=True))
    mock.execute = AsyncMock(return_value=sample_router_result)
    return mock


@pytest.fixture
def mock_qdrant_retrieval_agent(sample_retrieval_result):
    """Mock QdrantRetrievalAgent for orchestrator testing."""
    mock = AsyncMock()
    mock.run = AsyncMock(return_value=sample_retrieval_result)
    mock.validate_input = AsyncMock(return_value=Mock(is_valid=True))
    mock.execute = AsyncMock(return_value=sample_retrieval_result)
    return mock


@pytest.fixture
def mock_answer_synthesis_agent(sample_synthesis_result):
    """Mock AnswerSynthesisAgent for orchestrator testing."""
    mock = AsyncMock()
    mock.run = AsyncMock(return_value=sample_synthesis_result)
    mock.validate_input = AsyncMock(return_value=Mock(is_valid=True))
    mock.execute = AsyncMock(return_value=sample_synthesis_result)
    return mock


@pytest.fixture
def mock_error_recovery_agent():
    """Mock ErrorRecoveryAgent for orchestrator testing."""
    from app.models.agent_types import ErrorClassification

    mock = AsyncMock()
    mock.execute = AsyncMock(
        return_value=ErrorClassification(
            error_type="unknown",
            status_code=500,
            user_message="An unexpected error occurred. Please try again.",
            technical_details="Exception details here",
            should_retry=False,
        )
    )
    return mock


# ============================================================================
# System Prompt Mock
# ============================================================================

@pytest.fixture
def mock_system_prompt():
    """Mock system prompt content for testing."""
    return """
You are an AI assistant for the AI-Native Robotics textbook.

**Rules**:
1. Answer ONLY from provided context
2. If information not in context, say "I don't have information about that"
3. Cite sources (chapter, section, page)
4. Be concise and accurate

**NO HALLUCINATION**: Do not invent information not in the context.
    """.strip()


@pytest.fixture
def mock_system_prompt_file(mock_system_prompt, tmp_path):
    """Create temporary system prompt file for testing."""
    prompt_file = tmp_path / "agent.system.md"
    prompt_file.write_text(mock_system_prompt, encoding="utf-8")
    return prompt_file


# ============================================================================
# Async Test Helpers
# ============================================================================

@pytest.fixture
def event_loop():
    """Create event loop for async tests."""
    import asyncio
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


# ============================================================================
# FastAPI TestClient
# ============================================================================

@pytest.fixture
def test_client():
    """FastAPI TestClient for integration tests."""
    from fastapi.testclient import TestClient
    from app.main import app
    return TestClient(app)
