# 🤖 RAG Chatbot Backend

**Intelligent Retrieval-Augmented Generation backend with multi-LLM failover**

[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.115+-green.svg)](https://fastapi.tiangolo.com)

---

## ✨ Key Features

- **🎯 Dual-Mode RAG**: Normal semantic search + selected-text-only modes
- **🔄 Multi-LLM Failover**: Auto-switch between X.AI, OpenAI, Gemini on quota errors
- **📚 Constitution-Bound**: 6 principles ensure accurate, grounded responses  
- **⚡ Production-Ready**: Error handling, retry logic, structured logging
- **🔍 Semantic Search**: Qdrant vector DB with Cohere 1024-dim embeddings
- **💾 Dual Storage**: Neon Postgres + Qdrant for hybrid data management

---

## 🚀 Quick Start

\`\`\`bash
# Install dependencies
pip install -r requirements.txt

# Set up environment  
cp .env.example .env
# Add your API keys to .env

# Initialize databases
python scripts/setup_neon.py
python scripts/setup_qdrant.py

# Run server
uvicorn app.main:app --reload
\`\`\`

**Server**: http://localhost:8000  
**Docs**: http://localhost:8000/docs  
**Health**: http://localhost:8000/health

---

## 📚 API Endpoints

### Query (RAG)
\`\`\`bash
POST /query
{
  "question": "What is Physical AI?",
  "top_k": 3
}
\`\`\`

### Ingest Document
\`\`\`bash
POST /ingest
{
  "content": "# Chapter 1...",
  "metadata": {"title": "Introduction", "chapter": 1}
}
\`\`\`

### Health Check
\`\`\`bash
GET /health
\`\`\`

---

## 🏗️ Architecture

\`\`\`
Query → Multi-LLM Failover (X.AI → OpenAI → Gemini)
   ↓
Embed (Cohere) → Search (Qdrant) → Context Building
   ↓
RAG Agent (Constitution-Bound) → Response  
   ↓
Log to Postgres + Return Answer
\`\`\`

---

## 🧪 Testing

\`\`\`bash
# Run unit tests (18 tests)
pytest tests/unit/ -v

# Run with coverage
pytest --cov=app --cov-report=html
\`\`\`

---

## 🚢 Deployment

See [AUDIT_REPORT.md](AUDIT_REPORT.md) for comprehensive deployment guide.

**Quick Deploy to Hugging Face:**
1. Add API keys to HF Secrets
2. Configure as Docker Space  
3. Deploy!

---

## 📄 License

MIT License

---

**Built for the AI-Native Development community** ❤️
