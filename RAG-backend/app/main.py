"""
FastAPI application entry point for RAG Chatbot Backend.

This is the main application file that initializes FastAPI and includes all routers.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.health import router as health_router
from app.api.ingest import router as ingest_router
from app.api.embed import router as embed_router
from app.api.query import router as query_router
from app.api.chapters import router as chapters_router
from app.config import settings
from app.utils.provider_validator import validate_providers_at_startup

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend",
    description="Retrieval-Augmented Generation backend for AI-Native Development Book",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# CORS middleware - configured via CORS_ORIGINS environment variable
allowed_origins = settings.get_cors_origins_list()
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router, tags=["Health"])
app.include_router(ingest_router, tags=["Ingestion"])
app.include_router(embed_router, tags=["Ingestion"])
app.include_router(query_router, tags=["Query"])
app.include_router(chapters_router, tags=["Metadata"])


@app.on_event("startup")
async def startup_event():
    """Run on application startup."""
    print(f"[*] Starting RAG Backend in {settings.app_env} mode...")
    print(f"[*] Docs available at: http://localhost:8000/docs")

    # Validate LLM provider configurations
    print("[*] Validating LLM provider configurations...")
    try:
        is_valid, results = await validate_providers_at_startup()
        if not is_valid:
            print(f"[WARNING] Provider validation issues detected: {results['errors']}")
        else:
            print("[OK] All provider configurations are valid")
    except Exception as e:
        print(f"[ERROR] Failed to validate providers: {e}")

    # Initialize Qdrant collection (gracefully handle failures)
    print("[*] Initializing Qdrant vector database...")
    try:
        from app.retrieval.qdrant_client import initialize_collection
        success = initialize_collection()
        if not success:
            print("[WARNING] Qdrant unavailable - app will start without vector search")
            print("[INFO] Vector search features will be disabled until Qdrant is available")
    except Exception as e:
        print(f"[ERROR] Qdrant initialization failed: {e}")
        print("[WARNING] App will continue without vector search capabilities")


@app.on_event("shutdown")
async def shutdown_event():
    """Run on application shutdown."""
    from app.db import neon_client
    from app.retrieval import qdrant_client

    print("[*] Shutting down RAG Backend...")
    await neon_client.close_pool()
    qdrant_client.close_client()
    print("[OK] Shutdown complete")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot Backend API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
    }
