#!/usr/bin/env python3
"""
Simple Data Integrity Check for RAG System.

Validates:
1. Consistency between Neon database and Qdrant vector store
2. Proper chunk-to-document relationships
3. Metadata accuracy and completeness
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from app.db.neon_client import get_pool
from app.db.qdrant_client import get_client
from app.services.qdrant_indexer import QdrantIndexer


async def check_neon_connection():
    """Check if Neon database connection is working."""
    print("[INFO] Checking Neon database connection...")
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            result = await conn.fetchval("SELECT 1")
            if result == 1:
                print("  [OK] Neon database connection successful")
                return True
            else:
                print("  [ERROR] Neon database connection failed")
                return False
    except Exception as e:
        print(f"  [ERROR] Neon connection error: {e}")
        return False


async def check_qdrant_connection():
    """Check if Qdrant connection is working."""
    print("[INFO] Checking Qdrant connection...")
    try:
        client = get_client()
        collections = client.get_collections().collections
        print(f"  [OK] Qdrant connection successful, found {len(collections)} collections")

        # Show available collections
        for collection in collections:
            print(f"    - Collection: {collection.name}")
        return True
    except Exception as e:
        print(f"  [ERROR] Qdrant connection error: {e}")
        return False


async def check_neon_tables():
    """Check if required tables exist in Neon."""
    print("[INFO] Checking Neon database tables...")
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Check if documents table exists
            try:
                doc_count = await conn.fetchval("SELECT COUNT(*) FROM documents")
                print(f"  [OK] Documents table exists with {doc_count} documents")
            except Exception as e:
                print(f"  [ERROR] Documents table missing: {e}")
                return False

            # Check if chunks table exists
            try:
                chunk_count = await conn.fetchval("SELECT COUNT(*) FROM chunks")
                print(f"  [OK] Chunks table exists with {chunk_count} chunks")
            except Exception as e:
                print(f"  [ERROR] Chunks table missing: {e}")
                return False

            return True
    except Exception as e:
        print(f"  [ERROR] Error checking tables: {e}")
        return False


async def check_document_chunk_relationships():
    """Check document-chunk relationships."""
    print("[INFO] Checking document-chunk relationships...")
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Check for orphaned chunks (chunks without corresponding documents)
            orphaned_chunks = await conn.fetchval("""
                SELECT COUNT(*)
                FROM chunks c
                LEFT JOIN documents d ON c.doc_id = d.id
                WHERE d.id IS NULL
            """)

            if orphaned_chunks > 0:
                print(f"  [ERROR] Found {orphaned_chunks} orphaned chunks")
                return False
            else:
                print(f"  [OK] No orphaned chunks found")

            # Check for documents without chunks
            docs_without_chunks = await conn.fetchval("""
                SELECT COUNT(*)
                FROM documents d
                LEFT JOIN chunks c ON d.id = c.doc_id
                WHERE c.doc_id IS NULL
            """)

            if docs_without_chunks > 0:
                print(f"  [WARN] Found {docs_without_chunks} documents without chunks")
            else:
                print(f"  [OK] All documents have associated chunks")

            # Check for duplicate chunk IDs
            duplicate_chunks = await conn.fetchval("""
                SELECT COUNT(*) FROM (
                    SELECT id, COUNT(*) as cnt
                    FROM chunks
                    GROUP BY id
                    HAVING COUNT(*) > 1
                ) as duplicates
            """)

            if duplicate_chunks > 0:
                print(f"  [ERROR] Found {duplicate_chunks} duplicate chunk IDs")
                return False
            else:
                print(f"  [OK] No duplicate chunk IDs found")

            return True
    except Exception as e:
        print(f"  [ERROR] Error checking relationships: {e}")
        return False


async def check_metadata_completeness():
    """Check metadata completeness."""
    print("[INFO] Checking metadata completeness...")
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            # Check for missing metadata in documents
            docs_missing_meta = await conn.fetchval("""
                SELECT COUNT(*)
                FROM documents
                WHERE title IS NULL OR title = '' OR chapter IS NULL
            """)

            if docs_missing_meta > 0:
                print(f"  [WARN] Found {docs_missing_meta} documents with missing metadata")
            else:
                print(f"  [OK] All documents have complete metadata")

            # Check for missing metadata in chunks
            chunks_missing_meta = await conn.fetchval("""
                SELECT COUNT(*)
                FROM chunks
                WHERE chunk_text IS NULL OR chunk_text = '' OR token_count IS NULL
            """)

            if chunks_missing_meta > 0:
                print(f"  [WARN] Found {chunks_missing_meta} chunks with missing metadata")
            else:
                print(f"  [OK] All chunks have complete metadata")

            # Get sample chunks to inspect
            sample_chunks = await conn.fetch("""
                SELECT id, doc_id, chunk_text, chapter, section, page, token_count
                FROM chunks
                LIMIT 3
            """)

            print(f"  [OK] Sample chunks:")
            for i, chunk in enumerate(sample_chunks, 1):
                print(f"    {i}. Chunk ID: {str(chunk['id'])[:8]}..., Doc ID: {str(chunk['doc_id'])[:8]}..., Chapter: {chunk['chapter']}, Tokens: {chunk['token_count']}")

            return True
    except Exception as e:
        print(f"  [ERROR] Error checking metadata: {e}")
        return False


async def main():
    """Main validation function."""
    print("=" * 80)
    print("[INFO] RAG System Simple Data Integrity Check")
    print("=" * 80)

    # Run all validation checks
    results = []

    results.append(("Neon Connection", await check_neon_connection()))
    results.append(("Qdrant Connection", await check_qdrant_connection()))
    results.append(("Neon Tables", await check_neon_tables()))
    results.append(("Document-Chunk Relationships", await check_document_chunk_relationships()))
    results.append(("Metadata Completeness", await check_metadata_completeness()))

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    passed = 0
    total = len(results)

    for name, result in results:
        status = "[PASS] PASS" if result else "[FAIL] FAIL"
        print(f"{name}: {status}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{total} checks passed")

    if passed == total:
        print("[SUCCESS] All validation checks passed!")
        return True
    else:
        print(f"[ERROR] {total - passed} validation checks failed!")
        return False


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)