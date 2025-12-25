#!/usr/bin/env python3
"""
Documentation Indexing CLI Script.

Processes the entire documentation corpus and indexes it into Qdrant
following the canonical chunk schema.

Usage:
    python scripts/index_documentation.py --docs-path ../frontend/Physical\ AI\ and\ Robotics/docs
    python scripts/index_documentation.py --docs-path ../frontend/Physical\ AI\ and\ Robotics/docs --recreate
    python scripts/index_documentation.py --validate-only
"""

import asyncio
import argparse
import json
import sys
from pathlib import Path
from datetime import datetime

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.semantic_chunker import SemanticChunker, validate_chunk_schema
from app.services.qdrant_indexer import QdrantIndexer, validate_indexed_chunks
import logging

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def safe_print(*args, **kwargs):
    """Print with fallback for Unicode encoding errors on Windows."""
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        # Replace common emojis with ASCII equivalents
        new_args = []
        for arg in args:
            if isinstance(arg, str):
                arg = (arg.replace('📚', '[DOCS]')
                         .replace('🔧', '[TOOL]')
                         .replace('✓', '[OK]')
                         .replace('✗', '[X]')
                         .replace('🗄️', '[DB]')
                         .replace('📖', '[BOOK]')
                         .replace('🔍', '[SEARCH]')
                         .replace('💾', '[SAVE]')
                         .replace('⚡', '[FAST]')
                         .replace('📊', '[STATS]')
                         .replace('✅', '[DONE]')
                         .replace('🔎', '[FIND]')
                         .replace('→', '->'))
            new_args.append(arg)
        print(*new_args, **kwargs)


def print_banner():
    """Print CLI banner."""
    safe_print("=" * 80)
    try:
        safe_print("📚 Documentation Indexing Pipeline")
        safe_print("   Semantic Chunking → Embeddings → Qdrant Vector Database")
    except UnicodeEncodeError:
        safe_print("Documentation Indexing Pipeline")
        safe_print("   Semantic Chunking -> Embeddings -> Qdrant Vector Database")
    safe_print("=" * 80)
    safe_print()


async def run_indexing(
    docs_path: Path,
    recreate: bool = False,
    save_chunks: bool = False,
    validate_schema: bool = True
):
    """
    Run the complete indexing pipeline.

    Args:
        docs_path: Path to documentation directory
        recreate: Whether to recreate the Qdrant collection
        save_chunks: Whether to save chunks to JSON file
        validate_schema: Whether to validate chunk schema
    """
    print_banner()

    # Step 1: Initialize services
    safe_print("🔧 Initializing services...")
    chunker = SemanticChunker(
        min_tokens=150,
        max_tokens=300,
        overlap_tokens=50
    )
    indexer = QdrantIndexer(
        collection_name="documentation_chunks",
        batch_size=50
    )
    safe_print(f"   ✓ Chunker: {chunker.min_tokens}-{chunker.max_tokens} tokens")
    safe_print(f"   ✓ Indexer: {indexer.collection_name} (batch_size={indexer.batch_size})")
    safe_print(f"   ✓ Embedder: {indexer.embedder.provider} ({indexer.vector_size}D)")
    safe_print()

    # Step 2: Create/verify Qdrant collection
    safe_print("🗄️  Setting up Qdrant collection...")
    try:
        await indexer.create_collection(recreate=recreate)
        if recreate:
            safe_print("   ✓ Collection recreated")
        else:
            safe_print("   ✓ Collection ready")
    except Exception as e:
        safe_print(f"   ✗ Error: {e}")
        return
    safe_print()

    # Step 3: Process documentation
    safe_print(f"📖 Processing documentation from: {docs_path}")
    safe_print(f"   Looking for *.md and *.mdx files...")
    safe_print()

    try:
        chunks = chunker.process_directory(docs_path)
    except Exception as e:
        logger.error(f"Error processing documentation: {e}")
        return

    if not chunks:
        safe_print("   ✗ No chunks generated. Check documentation path.")
        return

    safe_print()
    safe_print(f"✓ Generated {len(chunks)} total chunks")
    safe_print()

    # Step 4: Validate chunk schema (if enabled)
    if validate_schema:
        safe_print("🔍 Validating chunk schema...")
        invalid_count = 0
        for i, chunk in enumerate(chunks):
            if not validate_chunk_schema(chunk):
                invalid_count += 1
                if invalid_count <= 5:  # Show first 5 errors
                    safe_print(f"   ✗ Invalid chunk at index {i}")

        if invalid_count == 0:
            safe_print("   ✓ All chunks valid")
        else:
            safe_print(f"   ✗ Found {invalid_count} invalid chunks")
            safe_print("   Aborting indexing. Fix schema validation errors.")
            return
        safe_print()

    # Step 5: Save chunks to JSON (if enabled)
    if save_chunks:
        safe_print("💾 Saving chunks to JSON...")
        output_file = Path("chunks_output.json")
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(chunks, f, indent=2, ensure_ascii=False)
        safe_print(f"   ✓ Saved to {output_file}")
        safe_print()

    # Step 6: Index into Qdrant
    safe_print("⚡ Indexing chunks into Qdrant...")
    safe_print("   This may take several minutes depending on corpus size...")
    safe_print()

    try:
        stats = await indexer.index_chunks(chunks, show_progress=True)
    except Exception as e:
        logger.error(f"Error indexing chunks: {e}")
        return

    safe_print()
    safe_print("=" * 80)
    safe_print("📊 Indexing Statistics")
    safe_print("=" * 80)
    safe_print(f"Total Chunks:          {stats['total_chunks']}")
    safe_print(f"Successfully Indexed:  {stats['indexed_count']}")
    safe_print(f"Failed:                {stats['failed_count']}")
    safe_print(f"Success Rate:          {stats['success_rate']:.1f}%")
    safe_print(f"Total in Collection:   {stats['total_in_collection']}")
    safe_print()

    # Step 7: Validate indexing
    safe_print("✅ Validating indexed chunks...")
    validation = await validate_indexed_chunks(indexer, sample_size=5)

    if validation['status'] == 'success':
        safe_print(f"   ✓ Found {validation['samples_found']} sample chunks")
        safe_print()
        safe_print("Sample chunks:")
        for i, sample in enumerate(validation['sample_chunks'], 1):
            safe_print(f"\n   {i}. Chapter {sample['chapter']}, Section: {sample['section']}")
            safe_print(f"      Tokens: {sample['token_count']}")
            safe_print(f"      Preview: {sample['preview']}")
    else:
        safe_print(f"   ✗ Validation failed: {validation.get('error', 'Unknown error')}")

    safe_print()
    safe_print("=" * 80)
    safe_print("✅ Documentation Indexing Complete!")
    safe_print("=" * 80)

    # Close connections
    await indexer.close()


async def run_validation_only():
    """Run validation on existing indexed chunks."""
    print_banner()
    safe_print("🔍 Running validation on existing Qdrant collection...")
    safe_print()

    indexer = QdrantIndexer(collection_name="documentation_chunks")

    # Get stats
    stats = await indexer.get_stats()
    if 'error' in stats:
        safe_print(f"✗ Error: {stats['error']}")
        return

    safe_print("Collection Stats:")
    safe_print(f"  Name:           {stats['collection_name']}")
    safe_print(f"  Total Chunks:   {stats['total_chunks']}")
    safe_print(f"  Vector Size:    {stats['vector_size']}D")
    safe_print(f"  Distance:       {stats['distance_metric']}")
    safe_print()

    # Validate samples
    validation = await validate_indexed_chunks(indexer, sample_size=10)

    if validation['status'] == 'success':
        safe_print(f"✓ Validation successful - found {validation['samples_found']} chunks")
        safe_print()
        safe_print("Sample chunks:")
        for i, sample in enumerate(validation['sample_chunks'], 1):
            safe_print(f"\n{i}. Chunk ID: {sample['chunk_id']}")
            safe_print(f"   Chapter: {sample['chapter']}, Section: {sample['section']}")
            safe_print(f"   Tokens: {sample['token_count']}")
            safe_print(f"   Preview: {sample['preview']}")
    else:
        safe_print(f"✗ Validation failed: {validation.get('error', 'Unknown error')}")

    await indexer.close()


async def run_search_test(query: str):
    """Test search functionality."""
    print_banner()
    safe_print(f"🔎 Testing search with query: '{query}'")
    safe_print()

    indexer = QdrantIndexer(collection_name="documentation_chunks")

    results = await indexer.search_chunks(query, limit=5, score_threshold=0.3)

    safe_print(f"Found {len(results)} results:")
    safe_print()

    for i, result in enumerate(results, 1):
        safe_print(f"{i}. Score: {result['score']:.3f}")
        safe_print(f"   Chapter {result['chapter']}: {result['section']}")
        safe_print(f"   Tokens: {result['token_count']}")
        safe_print(f"   Preview: {result['chunk_text'][:150]}...")
        safe_print()

    await indexer.close()


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Index documentation into Qdrant vector database"
    )

    parser.add_argument(
        '--docs-path',
        type=str,
        help='Path to documentation directory (e.g., ../frontend/Physical\\ AI\\ and\\ Robotics/docs)'
    )

    parser.add_argument(
        '--recreate',
        action='store_true',
        help='Recreate Qdrant collection (deletes existing data)'
    )

    parser.add_argument(
        '--save-chunks',
        action='store_true',
        help='Save generated chunks to JSON file'
    )

    parser.add_argument(
        '--no-validation',
        action='store_true',
        help='Skip schema validation (faster but risky)'
    )

    parser.add_argument(
        '--validate-only',
        action='store_true',
        help='Only validate existing indexed chunks'
    )

    parser.add_argument(
        '--search',
        type=str,
        help='Test search with a query'
    )

    args = parser.parse_args()

    # Run appropriate command
    if args.validate_only:
        asyncio.run(run_validation_only())
    elif args.search:
        asyncio.run(run_search_test(args.search))
    elif args.docs_path:
        docs_path = Path(args.docs_path)
        if not docs_path.exists():
            safe_print(f"✗ Error: Documentation path does not exist: {docs_path}")
            sys.exit(1)

        asyncio.run(run_indexing(
            docs_path=docs_path,
            recreate=args.recreate,
            save_chunks=args.save_chunks,
            validate_schema=not args.no_validation
        ))
    else:
        parser.print_help()
        safe_print()
        safe_print("Examples:")
        safe_print("  python scripts/index_documentation.py --docs-path docs/")
        safe_print("  python scripts/index_documentation.py --docs-path docs/ --recreate")
        safe_print("  python scripts/index_documentation.py --validate-only")
        safe_print("  python scripts/index_documentation.py --search 'physical AI'")


if __name__ == "__main__":
    main()
