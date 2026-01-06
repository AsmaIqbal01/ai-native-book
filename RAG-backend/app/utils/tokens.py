"""
Token utilities for text processing.

Provides consistent token counting functionality across the application.
"""
import tiktoken
from typing import Union


def count_tokens(text: str, encoding_name: str = "cl100k_base") -> int:
    """
    Count the number of tokens in text.

    Args:
        text: Text to count tokens for.
        encoding_name: Tokenizer encoding (default: cl100k_base for OpenAI models).

    Returns:
        int: Token count.
    """
    encoding = tiktoken.get_encoding(encoding_name)
    return len(encoding.encode(text))


def get_tokenizer(encoding_name: str = "cl100k_base"):
    """
    Get a tokenizer instance.

    Args:
        encoding_name: Tokenizer encoding name.

    Returns:
        Tokenizer instance.
    """
    return tiktoken.get_encoding(encoding_name)