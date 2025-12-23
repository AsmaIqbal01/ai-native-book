"""
Context building service for RAG queries.

Formats retrieved chunks or selected text into context for the LLM.
"""


def build_context_normal_rag(chunks: list[dict]) -> str:
    """
    Build context from retrieved Qdrant chunks.

    Args:
        chunks: List of retrieved chunks with metadata.
                Expected format: [{'chunk_text': str, 'chapter': int,
                                   'section': str, 'page': int}, ...]

    Returns:
        str: Formatted context for LLM.
    """
    if not chunks:
        return "[No relevant passages found]"

    context_parts = []
    for chunk in chunks:
        # Extract chunk content and metadata
        chunk_text = chunk.get("chunk_text", "")
        chapter = chunk.get("chapter", "N/A")
        section = chunk.get("section", "N/A")
        page = chunk.get("page", "N/A")

        # Format with metadata
        context_parts.append(
            f"""{chunk_text}

Chapter: {chapter}, Section: {section}, Page: {page}"""
        )

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
