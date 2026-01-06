"""
RAGChatAgent - FR-004 (Orchestrator)

Coordinates all sub-agents to execute the complete RAG pipeline.
Enforces constitution and maintains execution order.
"""

from app.agents.base_agent import BaseAgent, ValidationResult
from app.agents.query_router_agent import QueryRouterAgent
from app.agents.qdrant_retrieval_agent import QdrantRetrievalAgent
from app.agents.answer_synthesis_agent import AnswerSynthesisAgent
from app.agents.error_recovery_agent import ErrorRecoveryAgent
from app.models.agent_types import (
    OrchestratorInput,
    OrchestratorResult,
    RouterInput,
    RetrievalInput,
    SynthesisInput,
    ErrorRecoveryInput
)
from app.tools.context_builder import build_context_selected_text
from fastapi import HTTPException
import logging

logger = logging.getLogger(__name__)


class RAGChatAgent(BaseAgent[OrchestratorInput, OrchestratorResult]):
    """
    Orchestrator agent coordinating all RAG sub-agents.

    Execution Order (FR-004):
    1. QueryRouterAgent → Validate, detect mode, normalize
    2. QdrantRetrievalAgent → Embed, search, build context (if normal_rag)
    3. AnswerSynthesisAgent → Generate answer with multi-LLM failover
    4. ErrorRecoveryAgent → Handle errors if any step fails

    Enforces:
    - Constitution compliance (via system prompt in AnswerSynthesisAgent)
    - Clean separation of concerns
    - Error recovery at each boundary
    """

    def __init__(self):
        """Initialize orchestrator with all sub-agents."""
        super().__init__()
        self.query_router = QueryRouterAgent()
        self.retrieval_agent = QdrantRetrievalAgent()
        self.synthesis_agent = AnswerSynthesisAgent()
        self.error_recovery = ErrorRecoveryAgent()

        logger.info("RAGChatAgent orchestrator initialized with 4 sub-agents")

    async def validate_input(self, input_data: OrchestratorInput) -> ValidationResult:
        """
        Validate orchestrator input.

        Args:
            input_data: OrchestratorInput (same as QueryRequest)

        Returns:
            ValidationResult
        """
        if not input_data.question or input_data.question.strip() == "":
            return ValidationResult(
                is_valid=False,
                error_message="Question cannot be empty"
            )

        return ValidationResult(is_valid=True, normalized_input=input_data)

    async def execute(self, input_data: OrchestratorInput) -> OrchestratorResult:
        """
        Execute full RAG pipeline with orchestrated agents.

        Pipeline Flow:
        User Input → QueryRouter → QdrantRetrieval → AnswerSynthesis → Response
                         ↓              ↓                  ↓
                    ErrorRecovery  ErrorRecovery    ErrorRecovery

        Args:
            input_data: OrchestratorInput with question and filters

        Returns:
            OrchestratorResult with answer, citations, metadata

        Raises:
            HTTPException: If any step fails after error recovery
        """
        logger.info("RAGChatAgent orchestration started", extra={"question_preview": input_data.question[:100]})

        try:
            # ============================================================
            # STEP 1: Route Query (QueryRouterAgent)
            # ============================================================
            router_input = RouterInput(
                question=input_data.question,
                selected_text=input_data.selected_text,
                chapter=input_data.chapter
            )

            router_result = await self.query_router.run(router_input)

            # Handle greeting mode
            if router_result.mode == "greeting":
                logger.info("Greeting detected, returning friendly response")
                return OrchestratorResult(
                    answer="Hello! I'm here to help you with questions about the AI-Native Robotics textbook. What would you like to know?",
                    mode="greeting",
                    citations=[],
                    metadata={"greeting": True}
                )

            # Handle out-of-scope queries
            if not router_result.is_valid:
                logger.info("Invalid query detected", extra={"error": router_result.error})
                return OrchestratorResult(
                    answer=router_result.error or "Your question appears to be outside the scope of this documentation.",
                    mode="out_of_scope",
                    citations=[],
                    metadata={"out_of_scope": True}
                )

            # ============================================================
            # STEP 2: Retrieve Context (QdrantRetrievalAgent or SelectedText)
            # ============================================================
            if router_result.mode == "normal_rag":
                # Normal RAG: Retrieve from Qdrant
                retrieval_input = RetrievalInput(
                    query=router_result.normalized_query,
                    mode=router_result.mode,
                    chapter=input_data.chapter,
                    top_k=input_data.top_k
                )

                retrieval_result = await self.retrieval_agent.run(retrieval_input)

                # Check if any chunks were retrieved
                if not retrieval_result.chunks:
                    logger.warning("No chunks retrieved from Qdrant")
                    return OrchestratorResult(
                        answer="I couldn't find relevant information in the documentation to answer your question. Please try rephrasing or ask about a different topic.",
                        mode="normal_rag",
                        citations=[],
                        metadata={"no_chunks_retrieved": True}
                    )

                context = retrieval_result.context
                chunks = retrieval_result.chunks

            else:
                # Selected-text-only mode: Use selected text as context
                if not input_data.selected_text:
                    raise ValueError("Selected text mode requires selected_text parameter")

                context = build_context_selected_text(input_data.selected_text)
                chunks = []

            # ============================================================
            # STEP 3: Synthesize Answer (AnswerSynthesisAgent)
            # ============================================================
            synthesis_input = SynthesisInput(
                context=context,
                query=router_result.normalized_query,
                mode=router_result.mode,
                chunks=chunks
            )

            synthesis_result = await self.synthesis_agent.run(synthesis_input)

            # ============================================================
            # STEP 4: Build Final Response
            # ============================================================
            logger.info(
                "RAGChatAgent orchestration completed successfully",
                extra={
                    "mode": router_result.mode,
                    "chunks_used": len(chunks),
                    "provider_used": synthesis_result.provider_used
                }
            )

            return OrchestratorResult(
                answer=synthesis_result.answer,
                mode=router_result.mode,
                citations=synthesis_result.citations,
                metadata={
                    "chunks_retrieved": len(chunks),
                    "provider_used": synthesis_result.provider_used,
                    "mode": router_result.mode,
                    **synthesis_result.metadata
                }
            )

        except HTTPException:
            # Re-raise HTTP exceptions (already handled)
            raise

        except Exception as e:
            # ============================================================
            # STEP 5: Error Recovery (ErrorRecoveryAgent)
            # ============================================================
            logger.error("Error during orchestration", extra={"error": str(e), "type": type(e).__name__})

            error_input = ErrorRecoveryInput(
                exception=e,
                context={
                    "agent": "RAGChatAgent",
                    "question": input_data.question[:100],
                }
            )

            error_classification = await self.error_recovery.execute(error_input)

            # Raise HTTPException with classified error
            raise HTTPException(
                status_code=error_classification.status_code,
                detail=error_classification.user_message
            )
