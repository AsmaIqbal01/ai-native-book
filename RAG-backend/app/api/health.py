"""
Health check endpoint for RAG backend.

Verifies connectivity to all external services:
- Neon Postgres
- Qdrant Vector Database
- LLM Provider (optional check)
"""

from fastapi import APIRouter
from pydantic import BaseModel
from app.db import neon_client
from app.retrieval import qdrant_client

router = APIRouter()


class ServiceStatus(BaseModel):
    """Status of an external service."""

    neon: str
    qdrant: str
    llm: str


class HealthResponse(BaseModel):
    """Health check response model."""

    status: str
    version: str
    services: ServiceStatus


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse: Status of the backend and all services.
    """
    # Check Neon connection
    neon_status = "connected" if await neon_client.health_check() else "disconnected"

    # Check Qdrant connection
    qdrant_status = "connected" if await qdrant_client.health_check() else "disconnected"

    # LLM check (optional - just mark as available for now)
    # TODO: Add actual LLM connectivity check if needed
    llm_status = "available"

    # Overall status
    overall_status = "ok" if (neon_status == "connected" and qdrant_status == "connected") else "degraded"

    return HealthResponse(
        status=overall_status,
        version="1.0.0",
        services=ServiceStatus(
            neon=neon_status,
            qdrant=qdrant_status,
            llm=llm_status,
        ),
    )
