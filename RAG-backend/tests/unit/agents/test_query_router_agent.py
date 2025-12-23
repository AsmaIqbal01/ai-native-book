"""
Unit tests for QueryRouterAgent (FR-005)

Tests input validation, greeting detection, out-of-scope detection,
query normalization, and mode detection.
"""

import pytest
from app.agents.query_router_agent import QueryRouterAgent
from app.models.agent_types import RouterInput, RouterResult


# ============================================================================
# Input Validation Tests
# ============================================================================

@pytest.mark.asyncio
async def test_validate_empty_question_rejected():
    """Test empty question is rejected during validation."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="",
        selected_text=None,
        chapter=None,
    )

    validation_result = await agent.validate_input(router_input)

    assert validation_result.is_valid is False
    assert "cannot be empty" in validation_result.error_message.lower()


@pytest.mark.asyncio
async def test_validate_whitespace_only_question_rejected():
    """Test whitespace-only question is rejected during validation."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="   \n\t  ",
        selected_text=None,
        chapter=None,
    )

    validation_result = await agent.validate_input(router_input)

    assert validation_result.is_valid is False
    assert "cannot be empty" in validation_result.error_message.lower()


@pytest.mark.asyncio
async def test_validate_valid_question_passes():
    """Test valid question passes validation."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text=None,
        chapter=None,
    )

    validation_result = await agent.validate_input(router_input)

    assert validation_result.is_valid is True
    assert validation_result.error_message is None


# ============================================================================
# Greeting Detection Tests
# ============================================================================

@pytest.mark.asyncio
async def test_detect_greeting_hi():
    """Test 'hi' detected as greeting."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="hi",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "greeting"
    assert result.is_valid is True


@pytest.mark.asyncio
async def test_detect_greeting_hello():
    """Test 'hello' detected as greeting."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="hello",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "greeting"
    assert result.is_valid is True


@pytest.mark.asyncio
async def test_detect_greeting_hello_there():
    """Test 'hello there' (2 words) detected as greeting."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="hello there",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "greeting"


@pytest.mark.asyncio
async def test_detect_greeting_hey_whats_up():
    """Test 'hey what's up' (3 words) detected as greeting."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="hey what's up",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "greeting"


@pytest.mark.asyncio
async def test_greeting_not_detected_long_sentence():
    """Test greeting pattern not detected in long sentences (>3 words)."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="hey can you help me understand ROS 2",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    # Should NOT be greeting because >3 words
    assert result.mode != "greeting"
    assert result.mode == "normal_rag"


@pytest.mark.asyncio
async def test_question_not_greeting():
    """Test regular question not detected as greeting."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2 and how does it work?",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode != "greeting"


# ============================================================================
# Out-of-Scope Detection Tests
# ============================================================================

@pytest.mark.asyncio
async def test_detect_out_of_scope_weather():
    """Test weather question detected as out-of-scope."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="what's the weather today",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "out_of_scope"
    assert result.is_valid is False
    assert "outside the scope" in result.error.lower()


@pytest.mark.asyncio
async def test_detect_out_of_scope_politics():
    """Test politics question detected as out-of-scope."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="who won the election last year",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "out_of_scope"
    assert result.is_valid is False


@pytest.mark.asyncio
async def test_detect_out_of_scope_stock_market():
    """Test stock market question detected as out-of-scope."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="what is the stock price of Tesla",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "out_of_scope"
    assert result.is_valid is False


@pytest.mark.asyncio
async def test_robotics_question_in_scope():
    """Test robotics question is in-scope."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="explain ROS 2 navigation stack",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode != "out_of_scope"
    assert result.is_valid is True


# ============================================================================
# Query Normalization Tests
# ============================================================================

@pytest.mark.asyncio
async def test_query_normalization_whitespace_trimmed():
    """Test query whitespace is trimmed in normalized_query."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="  What is ROS 2?  \n",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.normalized_query == "What is ROS 2?"
    assert result.normalized_query.strip() == result.normalized_query


@pytest.mark.asyncio
async def test_query_normalization_preserves_content():
    """Test query content is preserved during normalization."""
    agent = QueryRouterAgent()
    original_question = "What is the difference between ROS 1 and ROS 2?"
    router_input = RouterInput(
        question=original_question,
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.normalized_query == original_question


# ============================================================================
# Mode Detection Tests
# ============================================================================

@pytest.mark.asyncio
async def test_mode_normal_rag_when_no_selected_text():
    """Test normal_rag mode when no selected_text provided."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "normal_rag"


@pytest.mark.asyncio
async def test_mode_normal_rag_when_empty_selected_text():
    """Test normal_rag mode when selected_text is empty string."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text="",
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "normal_rag"


@pytest.mark.asyncio
async def test_mode_selected_text_only_when_text_provided(sample_selected_text):
    """Test selected_text_only mode when selected text provided."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="Explain this text",
        selected_text=sample_selected_text,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "selected_text_only"


# ============================================================================
# Metadata Propagation Tests
# ============================================================================

@pytest.mark.asyncio
async def test_metadata_includes_chapter_filter():
    """Test metadata includes chapter filter when provided."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text=None,
        chapter="Chapter 1",
    )

    result = await agent.execute(router_input)

    assert result.metadata.get("chapter") == "Chapter 1"


@pytest.mark.asyncio
async def test_result_structure():
    """Test RouterResult has all required fields."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert isinstance(result, RouterResult)
    assert hasattr(result, "is_valid")
    assert hasattr(result, "mode")
    assert hasattr(result, "normalized_query")
    assert hasattr(result, "error")
    assert hasattr(result, "metadata")


# ============================================================================
# Edge Cases
# ============================================================================

@pytest.mark.asyncio
async def test_greeting_case_insensitive():
    """Test greeting detection is case-insensitive."""
    agent = QueryRouterAgent()

    # Test uppercase
    router_input_upper = RouterInput(
        question="HI",
        selected_text=None,
        chapter=None,
    )
    result_upper = await agent.execute(router_input_upper)
    assert result_upper.mode == "greeting"

    # Test mixed case
    router_input_mixed = RouterInput(
        question="HeLLo",
        selected_text=None,
        chapter=None,
    )
    result_mixed = await agent.execute(router_input_mixed)
    assert result_mixed.mode == "greeting"


@pytest.mark.asyncio
async def test_out_of_scope_case_insensitive():
    """Test out-of-scope detection is case-insensitive."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="WHAT IS THE WEATHER TODAY",
        selected_text=None,
        chapter=None,
    )

    result = await agent.execute(router_input)

    assert result.mode == "out_of_scope"


@pytest.mark.asyncio
async def test_run_method_validates_and_executes():
    """Test run() method successfully validates and executes."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="What is ROS 2?",
        selected_text=None,
        chapter=None,
    )

    result = await agent.run(router_input)

    assert isinstance(result, RouterResult)
    assert result.is_valid is True
    assert result.mode == "normal_rag"


@pytest.mark.asyncio
async def test_run_method_raises_on_invalid_input():
    """Test run() method raises ValueError for invalid input."""
    agent = QueryRouterAgent()
    router_input = RouterInput(
        question="",
        selected_text=None,
        chapter=None,
    )

    with pytest.raises(ValueError, match="cannot be empty"):
        await agent.run(router_input)
