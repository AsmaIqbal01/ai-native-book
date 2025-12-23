"""
Hugging Face Space entry point.

This file is required by Hugging Face Spaces to run the FastAPI application.
"""

import uvicorn
from app.main import app

if __name__ == "__main__":
    # Hugging Face Spaces runs on port 7860 by default
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=7860,
        log_level="info",
    )
