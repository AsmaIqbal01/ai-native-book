# 📚 Documentation Indexing Pipeline

Complete guide for chunking and indexing your entire documentation corpus into Qdrant for RAG-ready retrieval.

## 🎯 Overview

This pipeline processes documentation files (MDX/MD) and converts them into semantic chunks following a strict canonical schema:

```json
{
  "chunk_id": "a0b7ab87-1aad-4b90-9674-ec07f021a619",
  "doc_id": "1e9b1973-2d75-48cd-adfd-cae9f8552a26",
  "chunk_text": "...",
  "chapter": 1,
  "section": "Introduction",
  "page": 1,
  "token_count": 190
}
```

Each chunk is embedded using Cohere/OpenAI embeddings and stored in Qdrant vector database for semantic search.

## 🏗️ Architecture

```
Documentation Files (MDX/MD)
        ↓
Semantic Chunker (app/services/semantic_chunker.py)
        ↓
Token Counting & Validation
        ↓
Embedding Generation (app/services/embedder.py)
        ↓
Qdrant Vector Database (app/services/qdrant_indexer.py)
```

## 📦 Components

### 1. **SemanticChunker** (`app/services/semantic_chunker.py`)

Intelligent chunking that:
- Preserves semantic meaning
- Splits by logical sections (headers)
- Maintains target token range (150-300 tokens)
- Extracts metadata from frontmatter
- Generates UUIDs for chunks and documents

**Key Methods:**
- `process_document(file_path)` → Chunk a single document
- `process_directory(docs_path)` → Chunk entire documentation
- `extract_metadata(file_path)` → Parse frontmatter
- `validate_chunk_schema(chunk)` → Ensure schema compliance

### 2. **QdrantIndexer** (`app/services/qdrant_indexer.py`)

Handles Qdrant operations:
- Collection creation/management
- Batch embedding generation
- Efficient chunk storage with metadata
- Semantic search and filtering
- Validation and statistics

**Key Methods:**
- `create_collection(recreate=False)` → Setup Qdrant collection
- `index_chunks(chunks)` → Batch index with embeddings
- `search_chunks(query, limit, chapter_filter)` → Semantic search
- `get_stats()` → Collection statistics

### 3. **CLI Script** (`scripts/index_documentation.py`)

Command-line interface for:
- Processing entire documentation corpus
- Creating/recreating Qdrant collections
- Validating chunk schema
- Testing search functionality

## 🚀 Quick Start

### 1. Install Dependencies

```bash
cd RAG-backend
pip install -r requirements.txt
```

Required packages:
- `python-frontmatter` - Parse MDX frontmatter
- `tiktoken` - Token counting
- `qdrant-client` - Vector database
- `cohere` or `openai` - Embeddings

### 2. Configure Environment

Ensure your `.env` has:

```env
# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# Embeddings (choose one)
COHERE_API_KEY=your_cohere_key
# OR
OPENAI_API_KEY=your_openai_key
```

### 3. Run Indexing

```bash
# Basic indexing
python scripts/index_documentation.py \
  --docs-path ../frontend/Physical\ AI\ and\ Robotics/docs

# Recreate collection (deletes existing data)
python scripts/index_documentation.py \
  --docs-path ../frontend/Physical\ AI\ and\ Robotics/docs \
  --recreate

# Save chunks to JSON for inspection
python scripts/index_documentation.py \
  --docs-path ../frontend/Physical\ AI\ and\ Robotics/docs \
  --save-chunks
```

### 4. Validate Indexing

```bash
# Validate existing indexed chunks
python scripts/index_documentation.py --validate-only

# Test search
python scripts/index_documentation.py --search "physical AI"
```

## 📋 Chunking Rules

### Semantic Chunking Strategy

1. **Section-Based Splitting**
   - Chunks are created at logical boundaries (## and ### headers)
   - Each chunk represents a complete idea or concept
   - Headers are included in chunk text for context

2. **Token Range**
   - Target: 150-300 tokens per chunk
   - Minimum: 150 tokens (chunks below this may be merged)
   - Maximum: 300 tokens (larger sections are split)

3. **Metadata Extraction**
   - `chapter`: From frontmatter `chapter_number` or path
   - `section`: Current section title from headers
   - `page`: Sequential order index
   - `token_count`: Accurate tiktoken count

4. **Content Coverage**
   All documentation is processed:
   - Book chapters
   - Sections and subsections
   - Sidebar content
   - Technical explanations
   - Code examples (preserved as-is)

### Schema Compliance

Every chunk MUST have:
- ✅ `chunk_id` (UUID string)
- ✅ `doc_id` (UUID string, same for all chunks from one document)
- ✅ `chunk_text` (string, the actual content)
- ✅ `chapter` (integer)
- ✅ `section` (string)
- ✅ `page` (integer, sequential)
- ✅ `token_count` (integer)

Schema validation runs automatically before indexing.

## 🔍 Qdrant Integration

### Collection Structure

- **Name**: `documentation_chunks`
- **Vector Size**: 1024 (Cohere) or 768 (OpenAI)
- **Distance Metric**: Cosine similarity
- **Indexing Threshold**: 10,000 points

### Payload Fields

Each point in Qdrant contains:

```python
{
  'doc_id': 'uuid',
  'chunk_text': 'full text content',
  'chapter': 1,
  'section': 'Section Title',
  'page': 1,
  'token_count': 190
}
```

### Search Capabilities

1. **Semantic Search**
   ```python
   results = await indexer.search_chunks(
       query="What is physical AI?",
       limit=10,
       score_threshold=0.5
   )
   ```

2. **Filtered Search**
   ```python
   results = await indexer.search_chunks(
       query="ROS2 nodes",
       limit=5,
       chapter_filter=2,  # Only Chapter 2
       score_threshold=0.6
   )
   ```

3. **Metadata Retrieval**
   - All chunks include full metadata
   - Can filter by chapter, section, token count
   - Supports score thresholding

## 📊 Output & Statistics

### Indexing Output

```
📚 Documentation Indexing Pipeline
   Semantic Chunking → Embeddings → Qdrant Vector Database
================================================================================

🔧 Initializing services...
   ✓ Chunker: 150-300 tokens
   ✓ Indexer: documentation_chunks (batch_size=50)
   ✓ Embedder: cohere (1024D)

🗄️  Setting up Qdrant collection...
   ✓ Collection ready

📖 Processing documentation from: docs/
   Looking for *.md and *.mdx files...

Processing: physical-ai.mdx
  → Generated 8 chunks
Processing: embodied-intelligence.mdx
  → Generated 6 chunks
...

✓ Generated 342 total chunks

🔍 Validating chunk schema...
   ✓ All chunks valid

⚡ Indexing chunks into Qdrant...

Batch 1: Indexed 50/342 chunks (14.6%)
Batch 2: Indexed 100/342 chunks (29.2%)
...

================================================================================
📊 Indexing Statistics
================================================================================
Total Chunks:          342
Successfully Indexed:  342
Failed:                0
Success Rate:          100.0%
Total in Collection:   342

✅ Validating indexed chunks...
   ✓ Found 5 sample chunks

Sample chunks:
   1. Chapter 1, Section: Introduction to Physical AI
      Tokens: 256
      Preview: # Introduction to Physical AI...

✅ Documentation Indexing Complete!
```

### JSON Output (if `--save-chunks`)

```json
[
  {
    "chunk_id": "a0b7ab87-1aad-4b90-9674-ec07f021a619",
    "doc_id": "1e9b1973-2d75-48cd-adfd-cae9f8552a26",
    "chunk_text": "## Introduction to Physical AI\n\nPhysical AI...",
    "chapter": 1,
    "section": "Introduction to Physical AI",
    "page": 1,
    "token_count": 256
  },
  ...
]
```

## 🧪 Testing & Validation

### 1. Schema Validation

Automatic validation ensures:
- All required fields present
- Correct data types
- Valid UUIDs
- Positive integers for counters

### 2. Indexing Validation

After indexing:
- Sample chunks retrieved and inspected
- Metadata integrity verified
- Token counts validated
- Search functionality tested

### 3. Manual Testing

```bash
# Validate collection
python scripts/index_documentation.py --validate-only

# Test search
python scripts/index_documentation.py --search "humanoid robots"

# Check specific chapter
python scripts/index_documentation.py --search "ROS2 architecture" --chapter 2
```

## 🛠️ Programmatic Usage

### Chunking Only

```python
from pathlib import Path
from app.services.semantic_chunker import SemanticChunker

chunker = SemanticChunker(min_tokens=150, max_tokens=300)

# Chunk single document
chunks = chunker.process_document(Path("docs/chapter1/intro.mdx"))

# Chunk entire directory
all_chunks = chunker.process_directory(Path("docs/"))

# Save to JSON
import json
with open("chunks.json", "w") as f:
    json.dump(all_chunks, f, indent=2)
```

### Indexing Only

```python
from app.services.qdrant_indexer import QdrantIndexer
import asyncio

async def index_chunks(chunks):
    indexer = QdrantIndexer(collection_name="documentation_chunks")

    # Create collection
    await indexer.create_collection(recreate=True)

    # Index chunks
    stats = await indexer.index_chunks(chunks)

    print(f"Indexed {stats['indexed_count']} chunks")

    await indexer.close()

asyncio.run(index_chunks(all_chunks))
```

### Search

```python
from app.services.qdrant_indexer import QdrantIndexer
import asyncio

async def search():
    indexer = QdrantIndexer()

    results = await indexer.search_chunks(
        query="What are the key components of Physical AI?",
        limit=5,
        score_threshold=0.6
    )

    for result in results:
        print(f"Chapter {result['chapter']}: {result['section']}")
        print(f"Score: {result['score']:.3f}")
        print(f"Text: {result['chunk_text'][:200]}...")
        print()

    await indexer.close()

asyncio.run(search())
```

## 🔧 Advanced Configuration

### Custom Token Ranges

```python
chunker = SemanticChunker(
    min_tokens=100,     # Smaller minimum
    max_tokens=500,     # Larger maximum
    overlap_tokens=75   # More overlap for context
)
```

### Custom Collection Settings

```python
indexer = QdrantIndexer(
    collection_name="my_custom_collection",
    vector_size=1536,  # OpenAI text-embedding-3-large
    batch_size=100     # Larger batches
)
```

### Filtering by Document

```python
# Delete all chunks for a specific document
await indexer.delete_by_doc_id("1e9b1973-2d75-48cd-adfd-cae9f8552a26")

# Re-index just that document
new_chunks = chunker.process_document(
    Path("docs/chapter1/updated.mdx"),
    doc_id="1e9b1973-2d75-48cd-adfd-cae9f8552a26"
)
await indexer.index_chunks(new_chunks)
```

## 📈 Performance

### Expected Processing Times

- **Chunking**: ~0.5-1 second per document
- **Embedding**: ~2-5 seconds per batch of 50 chunks
- **Indexing**: ~3-7 seconds per batch of 50 chunks

### Total Time Estimate

For 64 documentation files (~300-400 chunks):
- Chunking: ~1 minute
- Embedding + Indexing: ~5-10 minutes
- **Total: ~6-11 minutes**

### Optimization Tips

1. **Increase batch size** for faster embedding (up to 96 for Cohere)
2. **Use Cohere** (1024D, faster than OpenAI for large batches)
3. **Skip validation** with `--no-validation` (only if schema is verified)
4. **Parallel processing** (future enhancement)

## 🐛 Troubleshooting

### Issue: "Collection not found"

```bash
# Create collection first
python scripts/index_documentation.py --docs-path docs/ --recreate
```

### Issue: "Invalid API key"

- Check `.env` has correct `QDRANT_API_KEY`, `COHERE_API_KEY`, or `OPENAI_API_KEY`
- Verify keys are active

### Issue: "Schema validation failed"

- Check frontmatter has `chapter_number` or proper path structure
- Ensure MDX files are properly formatted
- Use `--no-validation` to skip (not recommended)

### Issue: "Batch embedding failed"

- Reduce `batch_size` in `QdrantIndexer`
- Check API quota/limits
- Verify network connectivity

### Issue: "Token count mismatch"

- Update `tiktoken` to latest version
- Ensure encoding model matches (`cl100k_base` for GPT-4)

## 📚 References

- **Canonical Schema**: See reference chunk in prompt
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Tiktoken**: https://github.com/openai/tiktoken
- **Cohere Embeddings**: https://docs.cohere.com/reference/embed

## 🤝 Contributing

To enhance the pipeline:

1. **Improve Chunking Logic**: Modify `semantic_chunker.py`
2. **Add Filters**: Extend search in `qdrant_indexer.py`
3. **Custom Metadata**: Add fields to schema and update both services
4. **Performance**: Implement parallel processing for large corpora

## 📄 License

This indexing pipeline is part of the AI Native Book RAG backend.
