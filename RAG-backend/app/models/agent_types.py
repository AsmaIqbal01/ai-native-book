"""
Agent Data Contracts

Defines Pydantic models for inter-agent communication.
Each agent has clearly defined input/output types to ensure type safety.
"""

from pydantic import BaseModel, Field, ConfigDict
from typing import Optional, List, Dict, Any, Literal


# ============================================================================
# QueryRouterAgent Types (FR-005)
# ============================================================================

class RouterInput(BaseModel):
    """Input for QueryRouterAgent."""
    question: str = Field(..., min_length=1, max_length=10000, description="User's question")
    selected_text: Optional[str] = Field(None, description="Selected text for selected-text mode")
    chapter: Optional[int] = Field(None, description="Chapter filter for retrieval")


class RouterResult(BaseModel):
    """Output from QueryRouterAgent."""
    is_valid: bool = Field(..., description="Whether the query is valid")
    mode: Literal["normal_rag", "selected_text_only", "greeting", "out_of_scope"] = Field(
        ..., description="Detected query mode"
    )
    normalized_query: str = Field(..., description="Normalized/cleaned query text")
    error: Optional[str] = Field(None, description="Error message if validation failed")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional routing metadata")


# ============================================================================
# QdrantRetrievalAgent Types (FR-006)
# ============================================================================

class RetrievalInput(BaseModel):
    """Input for QdrantRetrievalAgent."""
    query: str = Field(..., description="Normalized query from QueryRouter")
    mode: str = Field(..., description="Query mode")
    chapter: Optional[int] = Field(None, description="Chapter filter")
    section: Optional[str] = Field(None, description="Section filter")
    top_k: int = Field(5, ge=1, le=20, description="Number of chunks to retrieve")


class RetrievalResult(BaseModel):
    """Output from QdrantRetrievalAgent."""
    chunks: List[Dict[str, Any]] = Field(default_factory=list, description="Retrieved chunk payloads")
    chunk_ids: List[str] = Field(default_factory=list, description="Qdrant chunk IDs")
    context: str = Field("", description="Formatted context string for LLM")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Retrieval stats")
    error: Optional[str] = Field(None, description="Error if retrieval failed")


# ============================================================================
# AnswerSynthesisAgent Types (FR-007)
# ============================================================================

class SynthesisInput(BaseModel):
    """Input for AnswerSynthesisAgent."""
    context: str = Field(..., description="Retrieved or selected context")
    query: str = Field(..., description="User's question")
    mode: str = Field(..., description="Query mode (normal_rag vs selected_text_only)")
    chunks: List[Dict[str, Any]] = Field(default_factory=list, description="Chunks for citation generation")


class SynthesisResult(BaseModel):
    """Output from AnswerSynthesisAgent."""
    answer: str = Field(..., description="Generated answer")
    provider_used: str = Field(..., description="LLM provider that generated the answer")
    citations: List[Dict[str, Any]] = Field(default_factory=list, description="Source citations")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Generation metadata")
    error: Optional[str] = Field(None, description="Error if synthesis failed")


# ============================================================================
# ErrorRecoveryAgent Types (FR-008)
# ============================================================================

class ErrorClassification(BaseModel):
    """Error classification result."""
    error_type: Literal["connection", "timeout", "validation", "quota", "rate_limit", "unknown"] = Field(
        ..., description="Classified error type"
    )
    status_code: int = Field(..., description="HTTP status code")
    user_message: str = Field(..., description="User-safe error message")
    technical_details: Optional[str] = Field(None, description="Technical error details (server-side only)")
    should_retry: bool = Field(False, description="Whether the request can be retried")


class ErrorRecoveryInput(BaseModel):
    """Input for ErrorRecoveryAgent."""
    model_config = ConfigDict(arbitrary_types_allowed=True)

    exception: Exception = Field(..., description="Original exception")
    context: Dict[str, Any] = Field(default_factory=dict, description="Error context (agent, input, etc.)")


# ============================================================================
# RAGChatAgent Types (FR-004 - Orchestrator)
# ============================================================================

class OrchestratorInput(BaseModel):
    """Input for RAGChatAgent orchestrator (same as QueryRequest)."""
    question: str = Field(..., min_length=1, max_length=10000)
    selected_text: Optional[str] = None
    chapter: Optional[int] = None
    top_k: int = Field(5, ge=1, le=20)


class OrchestratorResult(BaseModel):
    """Output from RAGChatAgent orchestrator (same as QueryResponse)."""
    answer: str
    mode: str
    citations: List[Dict[str, Any]] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)


# ============================================================================
# Performance Tracking Types
# ============================================================================

class AgentPerformanceMetrics(BaseModel):
    """Performance metrics for an agent execution."""
    agent_name: str
    execution_time_ms: int
    success: bool
    error_type: Optional[str] = None


class PipelineMetrics(BaseModel):
    """Full pipeline performance metrics."""
    total_time_ms: int
    agent_metrics: List[AgentPerformanceMetrics] = Field(default_factory=list)
    provider_used: Optional[str] = None
    chunks_retrieved: int = 0
