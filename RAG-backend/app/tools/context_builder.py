"""
Context building service for RAG queries.

Formats retrieved chunks or selected text into context for the LLM.
Includes deduplication and token limit enforcement.
"""

import tiktoken

# Default token limits for different contexts
DEFAULT_MAX_CONTEXT_TOKENS = 8000  # Safe limit for most models (leaves room for prompt + response)
SEPARATOR_TOKENS = 10  # Estimated tokens for separator "\n\n---\n\n"


def count_tokens(text: str, model: str = "gpt-4") -> int:
    """
    Count tokens in text using tiktoken.

    Args:
        text: Text to count tokens for
        model: Model name for encoding (default: gpt-4)

    Returns:
        int: Number of tokens
    """
    try:
        encoding = tiktoken.encoding_for_model(model)
    except KeyError:
        # Fallback to cl100k_base (used by GPT-4, GPT-3.5-turbo)
        encoding = tiktoken.get_encoding("cl100k_base")

    return len(encoding.encode(text))


def deduplicate_chunks(chunks: list[dict]) -> list[dict]:
    """
    Remove duplicate chunks based on chunk_text content.

    Args:
        chunks: List of chunks with 'chunk_text' field

    Returns:
        list[dict]: Deduplicated chunks (preserves order, keeps first occurrence)
    """
    seen_texts = set()
    deduplicated = []

    for chunk in chunks:
        chunk_text = chunk.get("chunk_text", "")
        # Use normalized text for comparison (strip whitespace, lowercase)
        normalized = chunk_text.strip().lower()

        if normalized and normalized not in seen_texts:
            seen_texts.add(normalized)
            deduplicated.append(chunk)

    return deduplicated


def build_context_normal_rag(
    chunks: list[dict],
    max_tokens: int = DEFAULT_MAX_CONTEXT_TOKENS,
    model: str = "gpt-4"
) -> str:
    """
    Build context from retrieved Qdrant chunks with deduplication and token limits.

    Args:
        chunks: List of retrieved chunks with metadata.
                Expected format: [{'chunk_text': str, 'chapter': int,
                                   'section': str, 'page': int}, ...]
        max_tokens: Maximum tokens allowed in context (default: 8000)
        model: Model name for token counting (default: gpt-4)

    Returns:
        str: Formatted context for LLM, deduplicated and within token limit.
    """
    if not chunks:
        return "[No relevant passages found]"

    # Step 1: Deduplicate chunks
    deduplicated_chunks = deduplicate_chunks(chunks)

    # Step 2: Build context parts with token tracking
    context_parts = []
    total_tokens = 0
    chunks_included = 0

    for chunk in deduplicated_chunks:
        # Extract chunk content and metadata
        chunk_text = chunk.get("chunk_text", "")
        chapter = chunk.get("chapter", "N/A")
        section = chunk.get("section", "N/A")
        page = chunk.get("page", "N/A")

        # Format with metadata
        formatted_chunk = f"""{chunk_text}

Chapter: {chapter}, Section: {section}, Page: {page}"""

        # Count tokens for this chunk
        chunk_tokens = count_tokens(formatted_chunk, model)

        # Check if adding this chunk would exceed token limit
        if total_tokens + chunk_tokens + SEPARATOR_TOKENS > max_tokens:
            # Stop adding chunks - we've hit the limit
            break

        context_parts.append(formatted_chunk)
        total_tokens += chunk_tokens + SEPARATOR_TOKENS
        chunks_included += 1

    # If no chunks fit, return first chunk truncated
    if not context_parts and deduplicated_chunks:
        first_chunk = deduplicated_chunks[0]
        chunk_text = first_chunk.get("chunk_text", "")
        chapter = first_chunk.get("chapter", "N/A")
        section = first_chunk.get("section", "N/A")

        # Truncate to fit within max_tokens
        encoding = tiktoken.get_encoding("cl100k_base")
        tokens = encoding.encode(chunk_text)
        truncated_tokens = tokens[:max_tokens - 100]  # Reserve 100 tokens for metadata
        truncated_text = encoding.decode(truncated_tokens)

        return f"""{truncated_text}...

[Truncated due to token limit]
Chapter: {chapter}, Section: {section}"""

    # Join all chunks with separator
    return "\n\n---\n\n".join(context_parts)


def build_context_selected_text(selected_text: str) -> str:
    """
    Build context from user-selected text.

    Args:
        selected_text: Text selected by user.

    Returns:
        str: The selected text (no additional formatting needed).
    """
    return selected_text.strip()
