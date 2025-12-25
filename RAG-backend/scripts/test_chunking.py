#!/usr/bin/env python3
"""
Quick test script for chunking pipeline.

Tests semantic chunking on a sample document to verify setup.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.semantic_chunker import SemanticChunker, validate_chunk_schema
import json


def test_chunking():
    """Test chunking on a sample document."""
    print("=" * 80)
    print("🧪 Testing Semantic Chunking Pipeline")
    print("=" * 80)
    print()

    # Find a sample document
    docs_path = Path("../frontend/Physical AI and Robotics/docs")

    if not docs_path.exists():
        docs_path = Path("../../frontend/Physical AI and Robotics/docs")

    if not docs_path.exists():
        print("✗ Error: Could not find docs directory")
        print(f"  Tried: {docs_path.absolute()}")
        return

    # Find first MDX file
    sample_files = list(docs_path.glob("**/*.mdx"))
    if not sample_files:
        print("✗ Error: No MDX files found")
        return

    sample_file = sample_files[0]
    print(f"📄 Testing with: {sample_file.name}")
    print(f"   Path: {sample_file}")
    print()

    # Initialize chunker
    chunker = SemanticChunker(min_tokens=150, max_tokens=300)

    # Process document
    print("⚙️  Processing document...")
    try:
        chunks = chunker.process_document(sample_file)
    except Exception as e:
        print(f"✗ Error processing document: {e}")
        return

    print(f"✓ Generated {len(chunks)} chunks")
    print()

    # Validate schema
    print("🔍 Validating chunk schema...")
    all_valid = True
    for i, chunk in enumerate(chunks):
        if not validate_chunk_schema(chunk):
            print(f"✗ Chunk {i} invalid")
            all_valid = False

    if all_valid:
        print("✓ All chunks valid")
    else:
        print("✗ Some chunks invalid")
        return

    print()

    # Show sample chunks
    print("📊 Sample Chunks:")
    print("-" * 80)

    for i, chunk in enumerate(chunks[:3], 1):
        print(f"\nChunk {i}:")
        print(f"  ID:      {chunk['chunk_id']}")
        print(f"  Doc ID:  {chunk['doc_id']}")
        print(f"  Chapter: {chunk['chapter']}")
        print(f"  Section: {chunk['section']}")
        print(f"  Page:    {chunk['page']}")
        print(f"  Tokens:  {chunk['token_count']}")
        print(f"  Preview: {chunk['chunk_text'][:150]}...")

    print()
    print("=" * 80)

    # Show statistics
    total_tokens = sum(c['token_count'] for c in chunks)
    avg_tokens = total_tokens / len(chunks) if chunks else 0
    min_tokens = min(c['token_count'] for c in chunks) if chunks else 0
    max_tokens = max(c['token_count'] for c in chunks) if chunks else 0

    print("📈 Statistics:")
    print(f"  Total Chunks:   {len(chunks)}")
    print(f"  Total Tokens:   {total_tokens}")
    print(f"  Average Tokens: {avg_tokens:.1f}")
    print(f"  Min Tokens:     {min_tokens}")
    print(f"  Max Tokens:     {max_tokens}")

    # Check if within target range
    out_of_range = sum(1 for c in chunks if c['token_count'] < 150 or c['token_count'] > 300)
    in_range_pct = ((len(chunks) - out_of_range) / len(chunks) * 100) if chunks else 0

    print(f"  In Range (150-300): {in_range_pct:.1f}%")

    print()
    print("=" * 80)
    print("✅ Test Complete!")
    print()
    print("Next steps:")
    print("  1. Install dependencies: pip install python-frontmatter tiktoken")
    print("  2. Run full indexing: python scripts/index_documentation.py --docs-path docs/")
    print("=" * 80)


if __name__ == "__main__":
    test_chunking()
