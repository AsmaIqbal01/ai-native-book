"""
Query mode detection service.

Determines whether a query should use:
- normal_rag: Semantic search + RAG (default)
- selected_text_only: Answer from selected text only (no Qdrant)
"""


def detect_mode(selected_text: str | None) -> str:
    """
    Detect query mode based on presence of selected text.

    Args:
        selected_text: Optional selected text from user.

    Returns:
        str: Either "selected_text_only" or "normal_rag".
    """
    if selected_text and selected_text.strip():
        return "selected_text_only"
    return "normal_rag"
