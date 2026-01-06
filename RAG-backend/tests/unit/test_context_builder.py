"""
Unit tests for context_builder.py

Tests context formatting for both normal_rag and selected_text_only modes.
"""

import pytest
from app.tools.context_builder import build_context_normal_rag, build_context_selected_text


def test_build_context_normal_rag_with_single_chunk():
    """Should format single chunk correctly."""
    chunks = [
        {
            "chunk_text": "ROS 2 is a framework for robot software.",
            "chapter": 2,
            "section": "Introduction",
            "page": 42,
        }
    ]

    context = build_context_normal_rag(chunks)

    assert "ROS 2 is a framework" in context
    assert "Chapter: 2" in context
    assert "Section: Introduction" in context
    assert "Page: 42" in context


def test_build_context_normal_rag_with_multiple_chunks():
    """Should format multiple chunks with separator."""
    chunks = [
        {
            "chunk_text": "First chunk content.",
            "chapter": 1,
            "section": "Intro",
            "page": 10,
        },
        {
            "chunk_text": "Second chunk content.",
            "chapter": 1,
            "section": "Intro",
            "page": 11,
        },
    ]

    context = build_context_normal_rag(chunks)

    assert "First chunk content" in context
    assert "Second chunk content" in context
    assert "---" in context  # Separator between chunks
    assert context.count("Chapter:") == 2


def test_build_context_normal_rag_with_empty_list():
    """Should return 'no passages' message when chunks empty."""
    chunks = []
    context = build_context_normal_rag(chunks)

    assert context == "[No relevant passages found]"


def test_build_context_normal_rag_with_missing_metadata():
    """Should handle missing metadata fields gracefully."""
    chunks = [
        {
            "chunk_text": "Content without all metadata.",
            "chapter": 3,
            # Missing section and page
        }
    ]

    context = build_context_normal_rag(chunks)

    assert "Content without all metadata" in context
    assert "Chapter: 3" in context
    assert "Section: N/A" in context
    assert "Page: N/A" in context


def test_build_context_selected_text_simple():
    """Should return selected text as-is with whitespace stripped."""
    selected_text = "  This is user-selected text.  "
    context = build_context_selected_text(selected_text)

    assert context == "This is user-selected text."


def test_build_context_selected_text_multiline():
    """Should preserve multiline structure."""
    selected_text = """
Line 1
Line 2
Line 3
    """

    context = build_context_selected_text(selected_text)

    assert "Line 1" in context
    assert "Line 2" in context
    assert "Line 3" in context


def test_build_context_selected_text_with_code():
    """Should preserve code formatting."""
    code = """
def hello():
    return "world"
    """

    context = build_context_selected_text(code)

    assert "def hello():" in context
    assert 'return "world"' in context
