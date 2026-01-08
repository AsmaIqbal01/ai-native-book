"""
Analytics API endpoints for ChatKit session analysis.

Provides REST API for analyzing ChatKit sessions and generating reports.
"""

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Dict, Optional
from datetime import datetime

from app.utils.openrouter_chatkit import ChatKitAnalyzer
from app.utils.logging import get_logger

router = APIRouter()
logger = get_logger(__name__)


# Request/Response Models
class ChatKitSession(BaseModel):
    """ChatKit session data model."""
    id: str
    object: str = "chatkit.session"
    workflow: Dict
    scope: Dict
    max_requests_per_1_minute: int = 30
    current_requests: Optional[int] = 0
    ttl_seconds: int = 900
    status: str
    cancelled_at: Optional[int] = None
    reason: Optional[str] = None


class AnalysisRequest(BaseModel):
    """Request model for session analysis."""
    session: ChatKitSession


class BatchAnalysisRequest(BaseModel):
    """Request model for batch analysis."""
    sessions: List[ChatKitSession] = Field(..., max_items=50)


class AnalysisResponse(BaseModel):
    """Response model for session analysis."""
    session_id: str
    analysis: str
    analyzed_at: str
    metadata: Dict = {}


class BatchAnalysisResponse(BaseModel):
    """Response model for batch analysis."""
    total_sessions: int
    analyses: Dict[str, str]
    summary: Optional[str] = None
    analyzed_at: str


@router.post("/analytics/chatkit/analyze", response_model=AnalysisResponse)
async def analyze_session(request: AnalysisRequest):
    """
    Analyze a single ChatKit session.

    Provides detailed insights on:
    - Root cause of cancellation
    - Rate limit analysis
    - Recovery recommendations
    - Prevention strategies

    Example:
        POST /analytics/chatkit/analyze
        {
          "session": {
            "id": "cksess_123",
            "status": "cancelled",
            "max_requests_per_1_minute": 30,
            "current_requests": 35,
            ...
          }
        }
    """
    logger.info("ChatKit session analysis requested", {
        "session_id": request.session.id,
        "status": request.session.status
    })

    try:
        analyzer = ChatKitAnalyzer()
        analysis = analyzer.analyze_session(request.session.dict())

        return AnalysisResponse(
            session_id=request.session.id,
            analysis=analysis,
            analyzed_at=datetime.utcnow().isoformat(),
            metadata={
                "status": request.session.status,
                "max_requests": request.session.max_requests_per_1_minute,
                "current_requests": request.session.current_requests
            }
        )

    except Exception as e:
        logger.error("Failed to analyze session", {
            "session_id": request.session.id,
            "error": str(e)
        })
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Analysis failed: {str(e)}"
        )


@router.post("/analytics/chatkit/batch", response_model=BatchAnalysisResponse)
async def batch_analyze_sessions(request: BatchAnalysisRequest):
    """
    Analyze multiple ChatKit sessions and generate summary report.

    Provides:
    - Individual analysis for each session
    - Aggregate statistics
    - Common patterns
    - Overall recommendations

    Maximum 50 sessions per request.

    Example:
        POST /analytics/chatkit/batch
        {
          "sessions": [
            {"id": "cksess_123", ...},
            {"id": "cksess_124", ...}
          ]
        }
    """
    logger.info("Batch ChatKit analysis requested", {
        "session_count": len(request.sessions)
    })

    try:
        analyzer = ChatKitAnalyzer()

        # Convert Pydantic models to dicts
        session_dicts = [session.dict() for session in request.sessions]

        # Analyze each session
        analyses = analyzer.batch_analyze(session_dicts)

        # Generate summary report
        summary = analyzer.generate_summary_report(session_dicts)

        return BatchAnalysisResponse(
            total_sessions=len(request.sessions),
            analyses=analyses,
            summary=summary,
            analyzed_at=datetime.utcnow().isoformat()
        )

    except Exception as e:
        logger.error("Batch analysis failed", {
            "session_count": len(request.sessions),
            "error": str(e)
        })
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Batch analysis failed: {str(e)}"
        )


@router.get("/analytics/chatkit/health")
async def analytics_health():
    """
    Health check for analytics service.

    Returns:
        Status of ChatKit analyzer service
    """
    try:
        analyzer = ChatKitAnalyzer()
        return {
            "status": "healthy",
            "service": "chatkit_analyzer",
            "timestamp": datetime.utcnow().isoformat()
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e),
            "timestamp": datetime.utcnow().isoformat()
        }
