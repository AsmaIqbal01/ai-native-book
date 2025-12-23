"""
RAG Agents Package

Exports all agent classes for the multi-agent RAG system.
"""

# Legacy agent (deprecated, kept for backward compatibility)
from app.agents.rag_agent import RAGAgent, get_agent

# New multi-agent system (FR-004 to FR-008)
from app.agents.base_agent import BaseAgent, ValidationResult
from app.agents.query_router_agent import QueryRouterAgent
from app.agents.qdrant_retrieval_agent import QdrantRetrievalAgent
from app.agents.answer_synthesis_agent import AnswerSynthesisAgent
from app.agents.error_recovery_agent import ErrorRecoveryAgent
from app.agents.rag_chat_agent import RAGChatAgent

__all__ = [
    # Legacy (will be deprecated after migration)
    "RAGAgent",
    "get_agent",
    # New multi-agent system
    "BaseAgent",
    "ValidationResult",
    "QueryRouterAgent",
    "QdrantRetrievalAgent",
    "AnswerSynthesisAgent",
    "ErrorRecoveryAgent",
    "RAGChatAgent",
]
