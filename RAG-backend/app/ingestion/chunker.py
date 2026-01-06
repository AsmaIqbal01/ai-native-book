"""
Text chunking service for splitting documents into manageable segments.

Uses tiktoken for accurate token counting with sliding window approach.
"""

import tiktoken
from typing import List
from app.utils.tokens import count_tokens


def chunk_text(
    text: str,
    chunk_size: int = 1024,  # Increased from 512 to reduce number of chunks
    overlap: int = 100,      # Increased from 50 to provide better context
    encoding_name: str = "cl100k_base"
) -> List[str]:
    """
    Chunk text into segments with token-based sliding window.

    Args:
        text: Text to chunk.
        chunk_size: Maximum tokens per chunk (default: 1024 to reduce API calls).
        overlap: Token overlap between chunks (default: 100 for better context).
        encoding_name: Tokenizer encoding (default: cl100k_base for OpenAI models).

    Returns:
        List[str]: List of text chunks.
    """
    # Get tokenizer
    encoding = tiktoken.get_encoding(encoding_name)

    # Tokenize the text
    tokens = encoding.encode(text)

    # If text is shorter than chunk_size, return as single chunk
    if len(tokens) <= chunk_size:
        return [text]

    chunks = []
    start_idx = 0

    while start_idx < len(tokens):
        # Get chunk tokens
        end_idx = start_idx + chunk_size
        chunk_tokens = tokens[start_idx:end_idx]

        # Decode back to text
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)

        # Move start index forward (with overlap)
        start_idx += (chunk_size - overlap)

    return chunks
