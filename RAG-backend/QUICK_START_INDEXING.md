# 🚀 Quick Start: Documentation Indexing

## Prerequisites

1. **Install Dependencies**
   ```bash
   cd RAG-backend
   pip install python-frontmatter tiktoken qdrant-client cohere
   # OR if using OpenAI embeddings:
   pip install python-frontmatter tiktoken qdrant-client openai
   ```

2. **Configure Environment**

   Ensure your `RAG-backend/.env` has:
   ```env
   # Qdrant Vector Database
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your_qdrant_key

   # Embeddings (Cohere recommended for speed)
   COHERE_API_KEY=your_cohere_key
   # OR
   OPENAI_API_KEY=your_openai_key
   ```

## Running the Indexing Pipeline

### Option 1: Full Indexing (Recommended First Time)

```bash
cd RAG-backend

python scripts/index_documentation.py \
  --docs-path "../frontend/Physical AI and Robotics/docs" \
  --recreate \
  --save-chunks
```

**What this does:**
- ✅ Deletes existing Qdrant collection (fresh start)
- ✅ Chunks all documentation files
- ✅ Validates chunk schema
- ✅ Generates embeddings
- ✅ Indexes into Qdrant
- ✅ Saves chunks to `chunks_output.json`

**Expected Output:**
```
📚 Documentation Indexing Pipeline
================================================================================

🔧 Initializing services...
   ✓ Chunker: 150-300 tokens
   ✓ Indexer: documentation_chunks (batch_size=50)
   ✓ Embedder: cohere (1024D)

📖 Processing documentation...
Found 64 documentation files

Processing: physical-ai.mdx
  → Generated 8 chunks
Processing: embodied-intelligence.mdx
  → Generated 6 chunks
...

✓ Generated 342 total chunks

⚡ Indexing chunks into Qdrant...
Batch 1: Indexed 50/342 chunks (14.6%)
...

✅ Documentation Indexing Complete!
```

**Time:** ~6-11 minutes for 64 files

### Option 2: Update Existing Collection

```bash
python scripts/index_documentation.py \
  --docs-path "../frontend/Physical AI and Robotics/docs"
```

**What this does:**
- ✅ Adds new chunks to existing collection
- ✅ No deletion of existing data
- ✅ Useful for incremental updates

### Option 3: Test First (No Indexing)

```bash
python scripts/test_chunking.py
```

**What this does:**
- ✅ Tests chunking on one sample file
- ✅ Validates schema
- ✅ Shows sample chunks
- ✅ No Qdrant operations (safe dry-run)

## Validation

### 1. Validate Indexed Data

```bash
python scripts/index_documentation.py --validate-only
```

**Output:**
```
Collection Stats:
  Name:           documentation_chunks
  Total Chunks:   342
  Vector Size:    1024D
  Distance:       COSINE

✓ Validation successful - found 10 chunks

Sample chunks:
1. Chunk ID: a0b7ab87-...
   Chapter: 1, Section: Introduction
   Tokens: 256
```

### 2. Test Search

```bash
python scripts/index_documentation.py --search "physical AI"
```

**Output:**
```
Found 5 results:

1. Score: 0.892
   Chapter 1: Introduction to Physical AI
   Preview: Physical AI refers to artificial intelligence...

2. Score: 0.856
   Chapter 1: What is Physical AI
   Preview: Unlike traditional AI that operates...
```

## Common Issues

### ❌ "ModuleNotFoundError: No module named 'frontmatter'"

**Fix:**
```bash
pip install python-frontmatter
```

### ❌ "Collection not found"

**Fix:** Create collection first
```bash
python scripts/index_documentation.py --docs-path docs/ --recreate
```

### ❌ "Invalid API key"

**Fix:** Check your `.env` file has correct keys:
```env
QDRANT_API_KEY=your_actual_key_here
COHERE_API_KEY=your_actual_key_here
```

### ❌ "Rate limit exceeded"

**Fix:** Reduce batch size or wait
```python
# In qdrant_indexer.py, reduce batch_size
indexer = QdrantIndexer(batch_size=25)  # Default is 50
```

## File Locations

```
RAG-backend/
├── app/services/
│   ├── semantic_chunker.py     ← Chunking logic
│   ├── qdrant_indexer.py       ← Qdrant operations
│   └── embedder.py             ← Embedding generation
├── scripts/
│   ├── index_documentation.py  ← Main CLI script
│   └── test_chunking.py        ← Test script
├── DOCUMENTATION_INDEXING.md   ← Full documentation
└── QUICK_START_INDEXING.md     ← This file
```

## Next Steps

After successful indexing:

1. **Test RAG Agent**
   ```bash
   # Start backend
   uvicorn app.main:app --reload

   # Test query
   curl -X POST http://localhost:8000/api/chat \
     -H "Content-Type: application/json" \
     -d '{"question": "What is Physical AI?", "mode": "normal_rag"}'
   ```

2. **Integrate with Frontend**
   - Update frontend to use the `/api/chat` endpoint
   - Chunks will be automatically retrieved and used for context

3. **Monitor Performance**
   - Check Qdrant dashboard for collection stats
   - Monitor retrieval quality
   - Adjust `score_threshold` in search if needed

## Chunk Schema Reference

Every chunk follows this structure:

```json
{
  "chunk_id": "a0b7ab87-1aad-4b90-9674-ec07f021a619",  // UUID
  "doc_id": "1e9b1973-2d75-48cd-adfd-cae9f8552a26",    // UUID (same per doc)
  "chunk_text": "## Introduction...",                   // Full content
  "chapter": 1,                                         // Integer
  "section": "Introduction to Physical AI",             // String
  "page": 1,                                            // Integer (sequential)
  "token_count": 256                                    // Integer
}
```

## CLI Command Reference

```bash
# Full indexing (fresh start)
python scripts/index_documentation.py --docs-path docs/ --recreate

# Incremental update
python scripts/index_documentation.py --docs-path docs/

# Save chunks to JSON
python scripts/index_documentation.py --docs-path docs/ --save-chunks

# Skip validation (faster but risky)
python scripts/index_documentation.py --docs-path docs/ --no-validation

# Validate only
python scripts/index_documentation.py --validate-only

# Test search
python scripts/index_documentation.py --search "your query"

# Test chunking (no indexing)
python scripts/test_chunking.py
```

## Support

For detailed documentation, see:
- **Full Guide**: `DOCUMENTATION_INDEXING.md`
- **OpenRouter Setup**: `OPENROUTER_USAGE.md`
- **RAG Backend README**: `README.md`
