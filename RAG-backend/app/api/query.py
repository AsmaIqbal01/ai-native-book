"""
Query endpoint for RAG chatbot.

Implements dual-mode query system:
- normal_rag: Semantic search + RAG
- selected_text_only: Answer from selected text only

Supports three architectures via feature flags:
- OpenAI Agents SDK: True Spec 3 compliance (use_openai_agents_sdk=True)
- Multi-Agent: Custom 5-agent system (use_multi_agent_system=True)
- Legacy: Monolithic RAGAgent (default)
"""

from fastapi import APIRouter, HTTPException
from datetime import datetime
from pydantic import ValidationError
from app.models.schemas import QueryRequest, QueryResponse
from app.models.agent_types import OrchestratorInput
from app.services.mode_detector import detect_mode
from app.services.context_builder import build_context_normal_rag, build_context_selected_text
from app.services.embedder import EmbeddingService
from app.services.retriever import RetrieverService
from app.agents.rag_agent import get_agent
from app.agents.rag_chat_agent import RAGChatAgent
from app.config import settings
from app.db.neon_client import get_pool
from app.utils.logging import get_logger
import uuid
import asyncio

router = APIRouter()
logger = get_logger(__name__)


@router.post("/query", response_model=QueryResponse)
async def query(request: QueryRequest):
    """
    Process a RAG query with dual-mode support.

    Supports two architectures via feature flag:
    - Legacy: Monolithic RAGAgent (default)
    - Multi-Agent: 5-agent system (FR-004 to FR-008)

    Modes:
    - normal_rag: Embed question → search Qdrant → RAG
    - selected_text_only: Answer from selected text only (no Qdrant)

    Args:
        request: QueryRequest with question, optional filters, and selected text.

    Returns:
        QueryResponse with answer, mode, and metadata.

    Raises:
        HTTPException: 400 for validation errors, 503 for service errors,
                      504 for timeout, 500 for unexpected errors.
    """
    start_time = datetime.utcnow()

    logger.info("Query received", {
        "question_preview": request.question[:100],
        "chapter": request.chapter,
        "top_k": request.top_k,
        "has_selected_text": bool(request.selected_text),
        "using_multi_agent": settings.use_multi_agent_system,
        "using_openai_sdk": settings.use_openai_agents_sdk,
    })

    # ========================================================================
    # FEATURE FLAG: OpenAI Agents SDK (Spec 3 Compliance)
    # ========================================================================
    if settings.use_openai_agents_sdk:
        logger.info("Using OpenAI Agents SDK architecture")

        from agents import Runner
        from app.agents.sdk_agents import default_orchestrator

        # Build structured input for the SDK agent
        # The agent will use its tools to handle the request
        if request.selected_text:
            agent_input = f"""Question: {request.question}

Selected Text (use this as context instead of retrieval):
{request.selected_text}

Instructions: Answer the question using ONLY the selected text above. Use mode='selected_text_only' when calling synthesize_answer."""
        else:
            agent_input = f"""Question: {request.question}

Chapter Filter: {request.chapter if request.chapter else 'None (search all chapters)'}
Top K Results: {request.top_k}

Instructions: Use embed_and_retrieve to get relevant context, then use synthesize_answer to generate the answer."""

        try:
            # Execute SDK agent via Runner.run()
            logger.info("Invoking SDK orchestrator agent via Runner.run()")
            result = await Runner.run(
                default_orchestrator,
                input=agent_input,
            )

            # Extract answer from SDK result
            answer = result.final_output

            if not answer:
                answer = "I couldn't generate an answer. Please try rephrasing your question."

            logger.info("SDK agent execution completed", {
                "answer_length": len(answer),
            })

            # Calculate latency
            end_time = datetime.utcnow()
            latency_ms = int((end_time - start_time).total_seconds() * 1000)

            # Log query to Neon
            try:
                pool = await get_pool()
                async with pool.acquire() as conn:
                    await conn.execute(
                        """
                        INSERT INTO query_logs (
                            query_id, question, mode, answer, chunks_used,
                            latency_ms, created_at
                        )
                        VALUES ($1, $2, $3, $4, $5, $6, $7)
                        """,
                        str(uuid.uuid4()),
                        request.question,
                        "openai_agents_sdk",
                        answer,
                        0,  # Chunks count can be extracted from tool results if needed
                        latency_ms,
                        start_time,
                    )
            except Exception as e:
                logger.error(f"Failed to log query: {e}")

            # Build response
            return QueryResponse(
                answer=answer,
                mode="openai_agents_sdk",
                citations=[],  # Can be populated from tool results if needed
                metadata={
                    "latency_ms": latency_ms,
                    "architecture": "openai_agents_sdk",
                    "sdk_version": "0.0.19",
                    "orchestrator": "default_orchestrator",
                }
            )

        except Exception as e:
            logger.error(f"SDK agent execution failed: {e}", exc_info=True)
            raise HTTPException(
                status_code=500,
                detail=f"Error during SDK agent execution: {str(e)}"
            )

    # ========================================================================
    # FEATURE FLAG: Multi-Agent System (FR-004 to FR-008)
    # ========================================================================
    if settings.use_multi_agent_system:
        logger.info("Using multi-agent architecture")

        # Use new multi-agent orchestrator
        orchestrator = RAGChatAgent()
        orchestrator_input = OrchestratorInput(
            question=request.question,
            selected_text=request.selected_text,
            chapter=request.chapter,
            top_k=request.top_k
        )

        try:
            result = await orchestrator.run(orchestrator_input)

            # Calculate latency
            end_time = datetime.utcnow()
            latency_ms = int((end_time - start_time).total_seconds() * 1000)

            # Log query to Neon
            try:
                pool = await get_pool()
                async with pool.acquire() as conn:
                    await conn.execute(
                        """
                        INSERT INTO query_logs (
                            query_id, question, mode, answer, chunks_used,
                            latency_ms, created_at
                        )
                        VALUES ($1, $2, $3, $4, $5, $6, $7)
                        """,
                        str(uuid.uuid4()),
                        request.question,
                        result.mode,
                        result.answer,
                        result.metadata.get("chunks_retrieved", 0),
                        latency_ms,
                        start_time,
                    )
            except Exception as e:
                logger.error(f"Failed to log query: {e}")

            # Add latency to metadata
            result.metadata["latency_ms"] = latency_ms

            logger.info("Multi-agent query completed", {
                "mode": result.mode,
                "latency_ms": latency_ms,
                "provider_used": result.metadata.get("provider_used")
            })

            return QueryResponse(
                answer=result.answer,
                mode=result.mode,
                citations=result.citations,
                metadata=result.metadata
            )

        except HTTPException:
            raise  # Re-raise HTTP exceptions from ErrorRecoveryAgent
        except Exception as e:
            logger.error(f"Multi-agent orchestration failed: {e}")
            raise HTTPException(
                status_code=500,
                detail=f"Error during query processing: {str(e)}"
            )

    # ========================================================================
    # LEGACY SYSTEM (Monolithic RAGAgent)
    # ========================================================================
    logger.info("Using legacy monolithic agent")

    # Step 1: Detect query mode
    mode = detect_mode(request.selected_text)
    logger.info(f"Query mode detected: {mode}")

    # Step 2: Build context based on mode
    if mode == "normal_rag":
        # Normal RAG: Embed → Search → Context
        try:
            # Embed the question
            embedder = EmbeddingService()
            query_embedding = await embedder.embed_text(request.question)

            # Search Qdrant for relevant chunks
            retriever = RetrieverService()
            search_results = await retriever.search(
                query_vector=query_embedding,
                chapter=request.chapter,
                section=None,  # Section filter not in QueryRequest, could add if needed
                top_k=request.top_k,
            )

            # Extract chunks and IDs from search results
            chunks = [result["payload"] for result in search_results]
            chunk_ids = [result["id"] for result in search_results]

            # Build context from chunks
            context = build_context_normal_rag(chunks)

        except ConnectionError as e:
            logger.error("Qdrant connection error", {"error": str(e)})
            raise HTTPException(
                status_code=503,
                detail="Vector database unavailable. Please try again later.",
            )
        except asyncio.TimeoutError as e:
            logger.error("Retrieval timeout", {"error": str(e)})
            raise HTTPException(
                status_code=504,
                detail="Retrieval request timed out. Please try again.",
            )
        except ValidationError as e:
            logger.error("Validation error during retrieval", {"error": str(e)})
            raise HTTPException(
                status_code=400,
                detail=f"Invalid query parameters: {str(e)}",
            )
        except Exception as e:
            logger.error("Unexpected error during retrieval", {"error": str(e), "type": type(e).__name__})
            raise HTTPException(
                status_code=500,
                detail=f"Error during retrieval: {str(e)}",
            )

    else:
        # Selected-text-only mode: Use selected text as context
        context = build_context_selected_text(request.selected_text)
        chunks = []
        chunk_ids = []

    # Step 3: Run RAG agent (with automatic failover)
    try:
        logger.info("Invoking RAG agent", {"mode": mode, "context_length": len(context)})
        agent = get_agent()
        agent_response = await agent.run_query(
            user_question=request.question,
            context=context,
            mode=mode,
        )
        answer = agent_response["answer"]
        provider_used = agent_response["provider_used"]
        logger.info("Agent response generated", {
            "answer_length": len(answer),
            "provider_used": provider_used
        })
    except ConnectionError as e:
        logger.error("LLM connection error", {"error": str(e)})
        raise HTTPException(
            status_code=503,
            detail="LLM service unavailable. Please try again later.",
        )
    except asyncio.TimeoutError as e:
        logger.error("LLM timeout", {"error": str(e)})
        raise HTTPException(
            status_code=504,
            detail="LLM request timed out. Please try again.",
        )
    except Exception as e:
        logger.error("Agent execution error", {"error": str(e), "type": type(e).__name__})
        # Check if it's a quota/rate limit error
        error_str = str(e).lower()
        if "quota" in error_str or "rate limit" in error_str or "429" in error_str:
            raise HTTPException(
                status_code=429,
                detail="LLM rate limit exceeded. Please try again later.",
            )
        raise HTTPException(
            status_code=500,
            detail=f"Error during agent execution: {str(e)}",
        )

    # Step 4: Calculate latency
    end_time = datetime.utcnow()
    latency_ms = int((end_time - start_time).total_seconds() * 1000)

    # Step 5: Log query to Neon (query_logs table)
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO query_logs (
                    query_id, question, mode, answer, chunks_used,
                    latency_ms, created_at
                )
                VALUES ($1, $2, $3, $4, $5, $6, $7)
                """,
                str(uuid.uuid4()),
                request.question,
                mode,
                answer,
                len(chunks),
                latency_ms,
                start_time,
            )
    except Exception as e:
        # Log error but don't fail the request
        print(f"[!] Failed to log query: {e}")

    # Step 6: Build response
    logger.info("Query completed successfully", {
        "mode": mode,
        "chunks_retrieved": len(chunks),
        "latency_ms": latency_ms,
        "provider_used": provider_used,
    })

    # Build citations from chunks
    citations = []
    for chunk in chunks:
        citations.append({
            "chapter": chunk.get("chapter"),
            "section": chunk.get("section"),
            "page": chunk.get("page"),
            "chunk_text": chunk.get("chunk_text", "")[:200],  # Truncate for response
        })

    return QueryResponse(
        answer=answer,
        mode=mode,
        citations=citations,
        metadata={
            "chunks_retrieved": len(chunks),
            "chunk_ids": chunk_ids,
            "latency_ms": latency_ms,
            "provider_used": provider_used,
        }
    )
