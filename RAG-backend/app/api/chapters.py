"""
Chapters metadata endpoint.

Provides list of available chapters and sections in the book.
"""

from fastapi import APIRouter, HTTPException
from app.db.neon_client import get_pool

router = APIRouter()


@router.get("/chapters")
async def get_chapters():
    """
    Get list of available chapters with their sections.

    Returns:
        List of chapters with metadata:
        [
            {
                "chapter": 1,
                "title": "Introduction to Physical AI",
                "sections": ["Introduction", "What is Physical AI?", ...]
            },
            ...
        ]
    """
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Get distinct chapters with their titles and sections
            rows = await conn.fetch(
                """
                SELECT
                    chapter,
                    title,
                    ARRAY_AGG(DISTINCT section ORDER BY section) as sections
                FROM documents
                WHERE chapter IS NOT NULL
                GROUP BY chapter, title
                ORDER BY chapter
                """
            )

            # Format response
            chapters = []
            for row in rows:
                chapters.append({
                    "chapter": row["chapter"],
                    "title": row["title"],
                    "sections": [s for s in row["sections"] if s is not None],
                })

            return chapters

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error fetching chapters: {str(e)}",
        )
