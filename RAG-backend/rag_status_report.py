#!/usr/bin/env python3
"""
RAG System Status Report.

Provides detailed information about the current state of the RAG system,
including database contents, vector store status, and data integrity.
"""

import asyncio
import sys
from pathlib import Path
import json

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from app.db.neon_client import get_pool
from app.db.qdrant_client import get_client
from app.services.qdrant_indexer import QdrantIndexer


async def get_neon_status():
    """Get detailed status of Neon database."""
    print("=" * 80)
    print("NEON DATABASE STATUS")
    print("=" * 80)

    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Get document count and details
            doc_count = await conn.fetchval("SELECT COUNT(*) FROM documents")
            print(f"Documents: {doc_count}")

            if doc_count > 0:
                docs = await conn.fetch("SELECT * FROM documents LIMIT 10")
                print("\nSample Documents:")
                for doc in docs:
                    print(f"  - ID: {str(doc['id'])[:8]}..., Title: {doc['title'][:50]}..., Chapter: {doc['chapter']}")

            # Get chunk count and details
            chunk_count = await conn.fetchval("SELECT COUNT(*) FROM chunks")
            print(f"\nChunks: {chunk_count}")

            if chunk_count > 0:
                chunks = await conn.fetch("SELECT * FROM chunks LIMIT 5")
                print("\nSample Chunks:")
                for chunk in chunks:
                    print(f"  - ID: {str(chunk['id'])[:8]}..., Doc ID: {str(chunk['doc_id'])[:8]}..., Chapter: {chunk['chapter']}, Tokens: {chunk['token_count']}")

            # Check for relationship integrity
            orphaned_chunks = await conn.fetchval("""
                SELECT COUNT(*) FROM chunks c
                LEFT JOIN documents d ON c.doc_id = d.id
                WHERE d.id IS NULL
            """)
            print(f"\nOrphaned Chunks: {orphaned_chunks}")

            docs_without_chunks = await conn.fetchval("""
                SELECT COUNT(*) FROM documents d
                LEFT JOIN chunks c ON d.id = c.doc_id
                WHERE c.doc_id IS NULL
            """)
            print(f"Documents without chunks: {docs_without_chunks}")

    except Exception as e:
        print(f"Error getting Neon status: {e}")


async def get_qdrant_status():
    """Get detailed status of Qdrant vector store."""
    print("\n" + "=" * 80)
    print("QDRANT VECTOR STORE STATUS")
    print("=" * 80)

    try:
        client = get_client()
        collections = client.get_collections().collections

        print(f"Total Collections: {len(collections)}")

        for collection in collections:
            info = client.get_collection(collection.name)
            print(f"\nCollection: {collection.name}")
            print(f"  Points: {info.points_count}")
            print(f"  Vector Size: {info.config.params.vectors.size}")
            print(f"  Distance: {info.config.params.vectors.distance}")

            # Get sample points if collection has points
            if info.points_count > 0:
                try:
                    # Try to get first few points to inspect structure
                    scroll_result = client.scroll(
                        collection_name=collection.name,
                        limit=2,
                        with_payload=True,
                        with_vectors=False
                    )

                    # The scroll method returns (points, next_page_offset)
                    points, _ = scroll_result
                    print(f"  Sample Points:")
                    for i, point in enumerate(points):
                        payload = point.payload
                        print(f"    {i+1}. ID: {str(point.id)[:8]}..., Doc ID: {payload.get('doc_id', 'N/A')[:8]}..., Chapter: {payload.get('chapter', 'N/A')}")

                except Exception as e:
                    print(f"    Could not fetch sample points: {e}")

    except Exception as e:
        print(f"Error getting Qdrant status: {e}")


async def compare_neon_qdrant():
    """Compare contents between Neon and Qdrant."""
    print("\n" + "=" * 80)
    print("NEON vs QDRANT COMPARISON")
    print("=" * 80)

    try:
        # Get Neon data
        pool = await get_pool()
        async with pool.acquire() as conn:
            neon_chunk_count = await conn.fetchval("SELECT COUNT(*) FROM chunks")
            neon_doc_count = await conn.fetchval("SELECT COUNT(*) FROM documents")

            # Get distinct doc IDs from chunks
            neon_doc_ids = await conn.fetch("SELECT DISTINCT doc_id FROM chunks")
            neon_doc_ids_set = {str(row['doc_id']) for row in neon_doc_ids}

        # Get Qdrant data
        client = get_client()
        collections = client.get_collections().collections

        for collection in collections:
            info = client.get_collection(collection.name)
            qdrant_point_count = info.points_count

            print(f"\nCollection: {collection.name}")
            print(f"  Neon Chunks: {neon_chunk_count}")
            print(f"  Qdrant Points: {qdrant_point_count}")

            if qdrant_point_count > 0:
                # Get sample doc IDs from Qdrant
                try:
                    scroll_result = client.scroll(
                        collection_name=collection.name,
                        limit=10,
                        with_payload=True,
                        with_vectors=False
                    )

                    points, _ = scroll_result
                    qdrant_doc_ids = set()
                    for point in points:
                        doc_id = point.payload.get('doc_id')
                        if doc_id:
                            qdrant_doc_ids.add(str(doc_id))

                    print(f"  Neon Doc IDs: {len(neon_doc_ids_set)}")
                    print(f"  Qdrant Doc IDs: {len(qdrant_doc_ids)}")

                    # Check for missing doc IDs in Qdrant
                    missing_in_qdrant = neon_doc_ids_set - qdrant_doc_ids
                    extra_in_qdrant = qdrant_doc_ids - neon_doc_ids_set

                    if missing_in_qdrant:
                        print(f"  Missing in Qdrant: {len(missing_in_qdrant)} doc IDs")

                    if extra_in_qdrant:
                        print(f"  Extra in Qdrant: {len(extra_in_qdrant)} doc IDs")

                    if not missing_in_qdrant and not extra_in_qdrant:
                        print(f"  ✓ Doc ID alignment: Good")

                except Exception as e:
                    print(f"  Error fetching Qdrant doc IDs: {e}")

    except Exception as e:
        print(f"Error comparing Neon and Qdrant: {e}")


async def main():
    """Main status function."""
    print("RAG SYSTEM STATUS REPORT")
    print("=" * 80)

    await get_neon_status()
    await get_qdrant_status()
    await compare_neon_qdrant()

    print("\n" + "=" * 80)
    print("STATUS REPORT COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    asyncio.run(main())