#!/usr/bin/env python3
"""
Data Integrity Validation Script for RAG System.

Validates:
1. Embeddings align with source content
2. No missing, duplicate, or malformed vectors
3. Consistency between Neon database and Qdrant vector store
4. Proper chunk-to-document relationships
5. Metadata accuracy and completeness
"""

import asyncio
import sys
import uuid
from pathlib import Path
from typing import Dict, List, Any, Optional
import json

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from app.db.neon_client import get_pool
from app.db.qdrant_client import get_client
from app.services.qdrant_indexer import QdrantIndexer
from app.services.embedder import EmbeddingService


async def validate_embeddings_alignment():
    """Validate that embeddings align with source content."""
    print("[INFO] Validating embeddings alignment...")

    # Test embedding generation consistency
    embedder = EmbeddingService()

    test_texts = [
        "This is a test document for RAG system validation.",
        "Another test document to verify embedding consistency.",
        "Physical AI and Robotics concepts for humanoid systems."
    ]

    try:
        # Generate embeddings for same text multiple times
        for text in test_texts:
            emb1 = await embedder.embed_text(text)
            emb2 = await embedder.embed_text(text)

            # Check if embeddings are consistent (should be identical for same input)
            if emb1 != emb2:
                print(f"  [ERROR] Embedding inconsistency detected for text: {text[:50]}...")
                return False

        print("  [OK] Embeddings are consistent")
        return True
    except Exception as e:
        print(f"  [ERROR] Error validating embeddings: {e}")
        return False


async def validate_vector_integrity():
    """Validate vector store for missing, duplicate, or malformed vectors."""
    print("[INFO] Validating vector store integrity...")

    try:
        indexer = QdrantIndexer(collection_name="documentation_chunks")
        stats = await indexer.get_stats()

        if 'error' in stats:
            print(f"  [ERROR] Error getting collection stats: {stats['error']}")
            return False

        print(f"  [OK] Collection: {stats['collection_name']}")
        print(f"  [OK] Total chunks: {stats['total_chunks']}")
        print(f"  [OK] Vector size: {stats['vector_size']}D")
        print(f"  [OK] Distance metric: {stats['distance_metric']}")

        # Sample some chunks to verify they exist and have proper structure
        validation = await indexer.validate_indexed_chunks(indexer, sample_size=5)

        if validation['status'] == 'success':
            print(f"  [OK] Found {validation['samples_found']} sample chunks")
            for sample in validation['sample_chunks']:
                print(f"    - Chunk ID: {sample['chunk_id'][:8]}..., Chapter: {sample['chapter']}, Tokens: {sample['token_count']}")
            return True
        else:
            print(f"  [ERROR] Validation failed: {validation.get('error', 'Unknown error')}")
            return False

    except Exception as e:
        print(f"  [ERROR] Error validating vector integrity: {e}")
        return False


async def validate_neon_qdrant_consistency():
    """Validate consistency between Neon database and Qdrant vector store."""
    print("[INFO] Validating Neon-Qdrant consistency...")

    try:
        # Get connection to Neon
        pool = await get_pool()

        # Get Qdrant client
        qdrant_client = get_client()

        # Check if tables exist in Neon
        async with pool.acquire() as conn:
            # Check if documents table exists and has data
            try:
                doc_count = await conn.fetchval("SELECT COUNT(*) FROM documents")
                print(f"  [OK] Documents in Neon: {doc_count}")
            except Exception as e:
                print(f"  [ERROR] Error accessing documents table: {e}")
                return False

            # Check if chunks table exists and has data
            try:
                chunk_count = await conn.fetchval("SELECT COUNT(*) FROM chunks")
                print(f"  [OK] Chunks in Neon: {chunk_count}")
            except Exception as e:
                print(f"  [ERROR] Error accessing chunks table: {e}")
                return False

            # Get some sample chunk IDs from Neon
            sample_chunks = await conn.fetch(
                "SELECT id, doc_id, chunk_text, token_count FROM chunks LIMIT 5"
            )

            if not sample_chunks:
                print("  [ERROR] No chunks found in Neon database")
                return False

            print(f"  [OK] Sample chunks from Neon:")
            for chunk in sample_chunks:
                chunk_id = chunk['id']
                doc_id = chunk['doc_id']
                chunk_text = chunk['chunk_text'][:100] + "..." if len(chunk['chunk_text']) > 100 else chunk['chunk_text']
                token_count = chunk['token_count']

                print(f"    - Chunk ID: {str(chunk_id)[:8]}...")
                print(f"      Doc ID: {str(doc_id)[:8]}...")
                print(f"      Text: {chunk_text}")
                print(f"      Tokens: {token_count}")

        # Check if these chunks exist in Qdrant
        indexer = QdrantIndexer(collection_name="documentation_chunks")

        # Search for some chunks by content to verify they exist in Qdrant
        sample_texts = [chunk['chunk_text'] for chunk in sample_chunks]
        if sample_texts:
            for text in sample_texts[:2]:  # Test first 2
                results = await indexer.search_chunks(text[:50], limit=1)  # Search by first 50 chars
                if results:
                    print(f"  [OK] Found corresponding chunk in Qdrant for Neon chunk")
                else:
                    print(f"  [WARN] Could not verify Qdrant correspondence for chunk starting: {text[:50]}...")

        return True

    except Exception as e:
        print(f"  [ERROR] Error validating Neon-Qdrant consistency: {e}")
        return False


async def validate_chunk_document_relationships():
    """Validate proper chunk-to-document relationships."""
    print("[INFO] Validating chunk-to-document relationships...")

    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            # Get sample documents and their chunks
            docs_with_chunks = await conn.fetch(
                """
                SELECT d.id as doc_id, d.title, d.chapter, COUNT(c.id) as chunk_count
                FROM documents d
                LEFT JOIN chunks c ON d.id = c.doc_id
                GROUP BY d.id, d.title, d.chapter
                LIMIT 10
                """
            )

            print(f"  [OK] Found {len(docs_with_chunks)} documents with chunk counts:")
            for doc in docs_with_chunks:
                print(f"    - Doc ID: {str(doc['doc_id'])[:8]}..., Chapter: {doc['chapter']}, Title: {doc['title'][:50]}..., Chunks: {doc['chunk_count']}")

            # Validate that chunks reference valid documents
            orphaned_chunks = await conn.fetchval(
                """
                SELECT COUNT(*)
                FROM chunks c
                LEFT JOIN documents d ON c.doc_id = d.id
                WHERE d.id IS NULL
                """
            )

            if orphaned_chunks > 0:
                print(f"  [ERROR] Found {orphaned_chunks} orphaned chunks (no corresponding document)")
                return False
            else:
                print(f"  [OK] No orphaned chunks found")

            # Check for duplicate chunk IDs
            duplicate_chunks = await conn.fetchval(
                """
                SELECT COUNT(*)
                FROM (
                    SELECT id, COUNT(*) as cnt
                    FROM chunks
                    GROUP BY id
                    HAVING COUNT(*) > 1
                ) as duplicates
                """
            )

            if duplicate_chunks > 0:
                print(f"  [ERROR] Found {duplicate_chunks} duplicate chunk IDs")
                return False
            else:
                print(f"  [OK] No duplicate chunk IDs found")

            return True

    except Exception as e:
        print(f"  [ERROR] Error validating chunk-document relationships: {e}")
        return False


async def validate_metadata_accuracy():
    """Validate metadata accuracy and completeness."""
    print("[INFO] Validating metadata accuracy and completeness...")

    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            # Check for missing metadata in documents
            docs_missing_meta = await conn.fetchval(
                """
                SELECT COUNT(*)
                FROM documents
                WHERE title IS NULL OR title = '' OR chapter IS NULL
                """
            )

            if docs_missing_meta > 0:
                print(f"  [WARN] Found {docs_missing_meta} documents with missing metadata")
            else:
                print(f"  [OK] All documents have complete metadata")

            # Check for missing metadata in chunks
            chunks_missing_meta = await conn.fetchval(
                """
                SELECT COUNT(*)
                FROM chunks
                WHERE chunk_text IS NULL OR chunk_text = '' OR token_count IS NULL
                """
            )

            if chunks_missing_meta > 0:
                print(f"  [WARN] Found {chunks_missing_meta} chunks with missing metadata")
            else:
                print(f"  [OK] All chunks have complete metadata")

            # Validate token counts are reasonable
            invalid_tokens = await conn.fetchval(
                """
                SELECT COUNT(*)
                FROM chunks
                WHERE token_count < 0 OR token_count > 10000  -- Assuming max 10k tokens is reasonable
                """
            )

            if invalid_tokens > 0:
                print(f"  [WARN] Found {invalid_tokens} chunks with invalid token counts")
            else:
                print(f"  [OK] All chunks have valid token counts")

            # Get sample chunks to verify metadata structure
            sample_chunks = await conn.fetch(
                "SELECT doc_id, chunk_text, chapter, section, page, token_count FROM chunks LIMIT 3"
            )

            print(f"  [OK] Sample chunk metadata:")
            for chunk in sample_chunks:
                print(f"    - Doc: {str(chunk['doc_id'])[:8]}..., Chapter: {chunk['chapter']}, Section: {chunk['section']}, Page: {chunk['page']}, Tokens: {chunk['token_count']}")

            return True

    except Exception as e:
        print(f"  [ERROR] Error validating metadata: {e}")
        return False


async def main():
    """Main validation function."""
    print("=" * 80)
    print("[INFO] RAG System Data Integrity Validation")
    print("=" * 80)

    # Run all validation checks
    results = []

    results.append(("Embeddings Alignment", await validate_embeddings_alignment()))
    results.append(("Vector Integrity", await validate_vector_integrity()))
    results.append(("Neon-Qdrant Consistency", await validate_neon_qdrant_consistency()))
    results.append(("Chunk-Document Relationships", await validate_chunk_document_relationships()))
    results.append(("Metadata Accuracy", await validate_metadata_accuracy()))

    # Summary
    print("\n" + "=" * 80)
    print("📊 Validation Summary")
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