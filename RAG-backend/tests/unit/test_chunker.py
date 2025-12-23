"""
Unit tests for text chunking service.
"""

import pytest
from app.services.chunker import chunk_text, count_tokens


def test_chunk_text_short_content():
    """Test that short content returns single chunk."""
    text = "This is a short text that fits in one chunk."
    chunks = chunk_text(text, chunk_size=512, overlap=50)

    assert len(chunks) == 1
    assert chunks[0] == text


def test_chunk_text_with_overlap():
    """Test that chunking with overlap works correctly."""
    # Create text that's definitely longer than chunk_size
    text = " ".join(["word"] * 1000)  # ~1000 tokens
    chunks = chunk_text(text, chunk_size=100, overlap=10)

    # Should have multiple chunks
    assert len(chunks) > 1

    # Each chunk should be approximately 100 tokens
    for chunk in chunks[:-1]:  # Exclude last chunk which may be shorter
        token_count = count_tokens(chunk)
        assert token_count <= 110  # Allow some margin


def test_count_tokens():
    """Test token counting."""
    text = "This is a test sentence."
    token_count = count_tokens(text)

    assert token_count > 0
    assert isinstance(token_count, int)


def test_chunk_text_empty():
    """Test that empty content returns single empty chunk."""
    text = ""
    chunks = chunk_text(text, chunk_size=512, overlap=50)

    assert len(chunks) == 1
    assert chunks[0] == ""


def test_chunk_text_exact_size():
    """Test chunking when content is exactly chunk_size."""
    # Create text of exactly 512 tokens
    words = ["word"] * 512
    text = " ".join(words)
    tokens = count_tokens(text)

    # Adjust text to be exactly 512 tokens
    while tokens > 512:
        words.pop()
        text = " ".join(words)
        tokens = count_tokens(text)

    chunks = chunk_text(text, chunk_size=512, overlap=50)

    # Should be single chunk since it fits
    assert len(chunks) == 1
