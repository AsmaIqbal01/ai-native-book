# Quickstart Guide
## Spec 1 - Core RAG Backend Foundation

**Target Audience**: Developers setting up the RAG backend locally for the first time.

**Time to Complete**: ~15 minutes

---

## Prerequisites

Before you begin, ensure you have:

- ✅ **Python 3.11+** installed (`python --version`)
- ✅ **pip** package manager (`pip --version`)
- ✅ **Git** for cloning the repository
- ✅ **Qdrant instance** (cloud or local) - Get free cloud instance at [https://qdrant.to/cloud](https://qdrant.to/cloud)
- ✅ **OpenAI API key** - Get from [https://platform.openai.com/api-keys](https://platform.openai.com/api-keys)

**Optional**:
- Docker (for containerized deployment)
- Cohere API key (alternative to OpenAI)

---

## Step 1: Clone Repository

```bash
git clone https://github.com/your-org/ai-native-book.git
cd ai-native-book/RAG-backend
```

---

## Step 2: Install Dependencies

```bash
# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

**Expected output**:
```
Installing collected packages: fastapi, qdrant-client, openai, pydantic, uvicorn, ...
Successfully installed fastapi-0.115.0 qdrant-client-1.7.0 ...
```

---

## Step 3: Configure Environment Variables

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your actual API keys
nano .env  # or use your preferred editor
```

**Required configuration** (`.env`):
```bash
# Qdrant Vector Database
QDRANT_URL=https://your-cluster-id.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# OpenAI Embeddings
OPENAI_API_KEY=sk-proj-...your_openai_api_key_here
EMBEDDING_PROVIDER=openai

# Application
APP_ENV=development
LOG_LEVEL=INFO
```

**Optional configuration**:
```bash
# Cohere (alternative to OpenAI)
COHERE_API_KEY=your_cohere_api_key_here
EMBEDDING_PROVIDER=cohere  # Change from 'openai' to 'cohere'
```

---

## Step 4: Initialize Qdrant Collection

```bash
# Run initialization script (creates collection if it doesn't exist)
python -m app.db.qdrant_client
```

**Expected output**:
```
[INFO] Connecting to Qdrant at https://your-cluster-id.qdrant.io:6333
[INFO] Created Qdrant collection: documentation_chunks
[INFO] Collection ready with 1536-dimensional vectors (OpenAI text-embedding-3-small)
```

---

## Step 5: Start the Backend Server

```bash
# Start FastAPI server with auto-reload
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using WatchFiles
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Server is now running** at:
- **API**: http://localhost:8000
- **Docs**: http://localhost:8000/docs (Swagger UI)
- **ReDoc**: http://localhost:8000/redoc (Alternative docs)

---

## Step 6: Verify Health Check

```bash
# In a new terminal, test the health endpoint
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "ok",
  "qdrant_connected": true,
  "embedding_api_available": true,
  "timestamp": "2025-12-25T12:34:56.789Z"
}
```

✅ **If you see `"status": "ok"`, your backend is ready!**

---

## Step 7: Ingest Sample Document

```bash
# Ingest a sample textbook chapter
curl -X POST http://localhost:8000/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Chapter 1: Introduction to AI-Native Robotics\n\nAI-Native Robotics combines traditional robotics with modern AI techniques like Large Language Models (LLMs), Vision-Language-Action (VLA) models, and reinforcement learning. This approach enables robots to understand natural language commands, perceive their environment, and adapt to new tasks without explicit programming.\n\n## Key Concepts\n\n1. **Natural Language Understanding**: Robots can interpret human instructions in plain language.\n2. **Visual Perception**: VLA models enable robots to understand scenes and objects.\n3. **Adaptive Behavior**: Reinforcement learning allows robots to learn from experience.\n\nThis paradigm shift makes robotics more accessible and flexible for real-world applications.",
    "metadata": {
      "title": "AI-Native Robotics Introduction",
      "chapter": 1,
      "section": "Introduction",
      "page": 1
    }
  }'
```

**Expected response**:
```json
{
  "status": "success",
  "doc_id": "550e8400-e29b-41d4-a716-446655440000",
  "chunks_created": 3,
  "embeddings_stored": 3
}
```

---

## Step 8: Query the System

```bash
# Search for relevant chunks
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is AI-Native Robotics?",
    "top_k": 2
  }'
```

**Expected response**:
```json
{
  "chunks": [
    {
      "chunk_text": "AI-Native Robotics combines traditional robotics with modern AI techniques like Large Language Models (LLMs), Vision-Language-Action (VLA) models, and reinforcement learning...",
      "chapter": 1,
      "section": "Introduction",
      "page": 1,
      "score": 0.91
    },
    {
      "chunk_text": "This paradigm shift makes robotics more accessible and flexible for real-world applications.",
      "chapter": 1,
      "section": "Introduction",
      "page": 1,
      "score": 0.76
    }
  ],
  "metadata": {
    "query_embedding_dim": 1536,
    "search_latency_ms": 187,
    "qdrant_hits": 3
  }
}
```

✅ **If you see chunks returned, your RAG pipeline is working!**

---

## Step 9: Explore API Documentation

Open your browser and navigate to:

**Swagger UI**: http://localhost:8000/docs

You'll see an interactive API documentation page where you can:
- 📖 View all endpoints (`/ingest`, `/query`, `/health`)
- 🧪 Test endpoints directly from the browser
- 📋 See request/response schemas
- 💡 View example payloads

---

## Troubleshooting

### Issue: "Qdrant connection error"

**Symptom**:
```json
{
  "error": "Vector database unavailable",
  "detail": "Failed to connect to Qdrant at https://...",
  "status_code": 503
}
```

**Solution**:
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Check Qdrant instance is running (visit Qdrant dashboard)
3. Test connection: `curl https://your-cluster-id.qdrant.io:6333/collections`

---

### Issue: "Embedding API unavailable"

**Symptom**:
```json
{
  "error": "Embedding service unavailable",
  "detail": "OpenAI API returned 401 Unauthorized",
  "status_code": 503
}
```

**Solution**:
1. Verify `OPENAI_API_KEY` in `.env` is valid
2. Check API key has credits: https://platform.openai.com/usage
3. Test API key: `curl https://api.openai.com/v1/models -H "Authorization: Bearer $OPENAI_API_KEY"`

---

### Issue: "Port 8000 already in use"

**Symptom**:
```
ERROR:    [Errno 48] Address already in use
```

**Solution**:
1. Kill existing process: `lsof -ti:8000 | xargs kill -9`
2. Or use a different port: `uvicorn app.main:app --port 8001`

---

## Next Steps

Now that your backend is running, you can:

1. **Run Tests**:
   ```bash
   pytest tests/ -v
   ```

2. **Ingest More Content**:
   - Write a script to ingest all textbook chapters
   - See `scripts/ingest_chapters.py` (to be created)

3. **Deploy to Production**:
   - Build Docker image: `docker build -t rag-backend .`
   - Deploy to cloud (Hugging Face, Render, Railway)

4. **Integrate Frontend** (Spec 4):
   - Connect React/Docusaurus UI to `/query` endpoint
   - Display chunks and citations

---

## Summary

✅ **You have successfully**:
- Installed dependencies
- Configured environment variables
- Initialized Qdrant collection
- Started FastAPI server
- Verified health check
- Ingested sample document
- Retrieved relevant chunks

**Your RAG backend foundation (Spec 1) is now complete!**

---

## Quick Reference

| Command | Description |
|---------|-------------|
| `uvicorn app.main:app --reload` | Start server with auto-reload |
| `curl http://localhost:8000/health` | Check system health |
| `curl -X POST http://localhost:8000/ingest` | Ingest document |
| `curl -X POST http://localhost:8000/query` | Query for chunks |
| `pytest tests/ -v` | Run all tests |
| `docker build -t rag-backend .` | Build Docker image |

---

**Need help?** Check the full documentation in `README.md` or open an issue on GitHub.
