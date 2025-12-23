# Deployment Guide - RAG Chatbot System

Complete deployment guide for deploying the integrated RAG chatbot with FastAPI backend and React frontend.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Deployment Options](#deployment-options)
3. [Option 1: GitHub Pages + Hugging Face (Recommended)](#option-1-github-pages--hugging-face-recommended)
4. [Option 2: Full Hugging Face Space](#option-2-full-hugging-face-space)
5. [Environment Configuration](#environment-configuration)
6. [Switching Between Environments](#switching-between-environments)
7. [Post-Deployment](#post-deployment)

---

## Architecture Overview

The system consists of two main components:

```
┌─────────────────────┐         ┌─────────────────────┐
│                     │         │                     │
│  React Frontend     │◄───────►│  FastAPI Backend    │
│  (Docusaurus)       │  CORS   │  (RAG System)       │
│                     │         │                     │
│  - Chat UI          │         │  - Query API        │
│  - Theme Support    │         │  - Embeddings       │
│  - API Client       │         │  - Vector Search    │
│                     │         │                     │
└─────────────────────┘         └─────────────────────┘
        │                               │
        │                               │
        ▼                               ▼
┌─────────────────────┐         ┌─────────────────────┐
│  GitHub Pages       │         │  External Services  │
│  asmaiqbal01.github │         │  - Qdrant (Vectors) │
│  .io/ai-native-book │         │  - Neon (Postgres)  │
│                     │         │  - LLM Providers    │
└─────────────────────┘         └─────────────────────┘
```

---

## Deployment Options

### Option 1: GitHub Pages + Hugging Face (Recommended)
- **Frontend**: GitHub Pages (free, fast CDN)
- **Backend**: Hugging Face Spaces (free tier available, good for ML)
- **Pros**: Clear separation, easy to update independently, leverages free tiers
- **Cons**: Requires CORS configuration, two deployment targets

### Option 2: Full Hugging Face Space
- **Frontend + Backend**: Single Hugging Face Space
- **Pros**: Single deployment, no CORS issues, easier configuration
- **Cons**: Higher resource usage, single point of failure

---

## Option 1: GitHub Pages + Hugging Face (Recommended)

### Step 1: Deploy Backend to Hugging Face Spaces

#### 1.1 Prepare Deployment Files

```bash
# Run the automated deployment script
python deploy-to-huggingface.py
```

This script will:
- Copy necessary backend files to `C:\Users\umzaid\Desktop\huggingface`
- Exclude unnecessary files (.venv, tests, etc.)
- Create `app.py` entry point
- Generate Hugging Face-specific README

**Manual verification:**
```bash
cd "C:\Users\umzaid\Desktop\huggingface"
ls

# Should contain:
# - app/             (FastAPI application)
# - app.py           (HF entry point)
# - requirements.txt
# - runtime.txt
# - Dockerfile
# - .env.example
# - README.md
```

#### 1.2 Configure Environment Variables

**Option A: Using .env file (Development/Testing)**

```bash
cd "C:\Users\umzaid\Desktop\huggingface"
cp .env.example .env
```

Edit `.env` and set:
```bash
# Primary LLM Provider
LLM_PROVIDER=xai
LLM_API_KEY=your_actual_api_key_here
LLM_BASE_URL=https://api.x.ai/v1
LLM_MODEL=grok-beta

# Fallback Providers
LLM_PROVIDER_FALLBACK_1=openai
LLM_API_KEY_FALLBACK_1=your_openai_key_here
LLM_BASE_URL_FALLBACK_1=https://api.openai.com/v1
LLM_MODEL_FALLBACK_1=gpt-4o-mini

# OpenAI for embeddings
OPENAI_API_KEY=your_openai_key_here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Application Configuration
APP_ENV=production
LOG_LEVEL=INFO

# CORS - CRITICAL for GitHub Pages integration
CORS_ORIGINS=https://asmaiqbal01.github.io,https://asmaiqbal000-hackathon1-ai-book.hf.space
```

**Option B: Using Hugging Face Secrets (Production - Recommended)**

1. Go to your Space settings: https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE/settings
2. Click on "Variables and secrets"
3. Add each variable as a secret:

| Variable Name | Example Value |
|--------------|---------------|
| `LLM_PROVIDER` | `xai` |
| `LLM_API_KEY` | `xai-...` (your actual key) |
| `LLM_BASE_URL` | `https://api.x.ai/v1` |
| `LLM_MODEL` | `grok-beta` |
| `OPENAI_API_KEY` | `sk-...` |
| `QDRANT_URL` | `https://xyz.qdrant.io` |
| `QDRANT_API_KEY` | `your_key` |
| `NEON_DATABASE_URL` | `postgresql://...` |
| `CORS_ORIGINS` | `https://asmaiqbal01.github.io,https://asmaiqbal000-hackathon1-ai-book.hf.space` |
| `APP_ENV` | `production` |
| `LOG_LEVEL` | `INFO` |

#### 1.3 Deploy to Hugging Face

**First-time setup:**

```bash
cd "C:\Users\umzaid\Desktop\huggingface"

# Initialize git repository
git init

# Add Hugging Face remote
git remote add origin https://huggingface.co/spaces/asmaiqbal000/hackathon1-ai-book

# Configure git
git config user.email "your-email@example.com"
git config user.name "Your Name"

# Add all files (but not .env if using secrets)
git add .

# Commit
git commit -m "Initial deployment of RAG backend"

# Push to Hugging Face
git push -u origin main
```

**For updates:**

```bash
cd "C:\Users\umzaid\Desktop\huggingface"
git add .
git commit -m "Update backend"
git push
```

#### 1.4 Verify Backend Deployment

1. Wait for Hugging Face to build (check build logs in Space settings)
2. Test health endpoint:
   ```bash
   curl https://asmaiqbal000-hackathon1-ai-book.hf.space/health
   ```
3. View API docs: https://asmaiqbal000-hackathon1-ai-book.hf.space/docs

**Expected Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T...",
  "services": {
    "qdrant": "connected",
    "neon": "connected",
    "llm": "configured"
  }
}
```

### Step 2: Deploy Frontend to GitHub Pages

#### 2.1 Configure Frontend Environment

```bash
cd "C:\Users\asmaiqbal\ai-native-book\frontend\Physical AI and Robotics"

# Copy production environment template
cp .env.example .env

# Edit .env
```

Set in `.env`:
```bash
# Production backend URL
REACT_APP_API_BASE_URL=https://asmaiqbal000-hackathon1-ai-book.hf.space

# Optional settings
REACT_APP_API_TIMEOUT=30000
REACT_APP_DEBUG=false
```

#### 2.2 Update package.json

Ensure your `package.json` has the correct homepage:

```json
{
  "homepage": "https://asmaiqbal01.github.io/ai-native-book",
  "scripts": {
    "predeploy": "npm run build",
    "deploy": "gh-pages -d build"
  }
}
```

#### 2.3 Build and Deploy

```bash
cd "C:\Users\asmaiqbal\ai-native-book\frontend\Physical AI and Robotics"

# Install dependencies if needed
npm install

# Install gh-pages if not already installed
npm install --save-dev gh-pages

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy
```

**Expected Output:**
```
> ai-native-robotics-textbook@0.1.0 predeploy
> npm run build

Creating an optimized production build...
Compiled successfully.

> ai-native-robotics-textbook@0.1.0 deploy
> gh-pages -d build

Published
```

#### 2.4 Verify Frontend Deployment

1. Navigate to: https://asmaiqbal01.github.io/ai-native-book/
2. Check that the page loads
3. Navigate to the chat interface
4. Verify "● Connected" status indicator
5. Test with a question

**Troubleshooting:**
- If you see 404, wait a few minutes for GitHub Pages to propagate
- If connection shows offline, check browser console for CORS errors
- Verify backend URL in `.env` matches your Hugging Face Space

---

## Option 2: Full Hugging Face Space

This option serves both frontend and backend from a single Hugging Face Space.

### Step 1: Build Frontend

```bash
cd "C:\Users\asmaiqbal\ai-native-book\frontend\Physical AI and Robotics"

# Set API URL to same origin (relative paths)
echo "REACT_APP_API_BASE_URL=/api" > .env

# Build
npm run build
```

### Step 2: Copy to Hugging Face Space

```bash
# Create static directory in HF Space
mkdir -p "C:\Users\umzaid\Desktop\huggingface\static"

# Copy build files
cp -r build/* "C:\Users\umzaid\Desktop\huggingface\static/"
```

### Step 3: Update Backend to Serve Static Files

Edit `C:\Users\umzaid\Desktop\huggingface\app.py`:

```python
"""
Hugging Face Space entry point with static file serving.
"""

import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

# Import main app
from app.main import app

# Mount static files
static_path = Path(__file__).parent / "static"
if static_path.exists():
    app.mount("/static", StaticFiles(directory=str(static_path)), name="static")

    @app.get("/")
    async def serve_frontend():
        return FileResponse(str(static_path / "index.html"))

if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=7860,
        log_level="info",
    )
```

### Step 4: Deploy

```bash
cd "C:\Users\umzaid\Desktop\huggingface"
git add .
git commit -m "Deploy full stack to Hugging Face"
git push
```

### Step 5: Test

Navigate to: https://asmaiqbal000-hackathon1-ai-book.hf.space/

---

## Environment Configuration

### Backend Environment Variables

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `LLM_PROVIDER` | Yes | Primary LLM provider | `xai`, `openai`, `gemini` |
| `LLM_API_KEY` | Yes | API key for primary LLM | `xai-...` |
| `LLM_BASE_URL` | Yes | Base URL for LLM API | `https://api.x.ai/v1` |
| `LLM_MODEL` | Yes | Model name | `grok-beta` |
| `OPENAI_API_KEY` | Yes | OpenAI key (for embeddings) | `sk-...` |
| `QDRANT_URL` | Yes | Qdrant vector DB URL | `https://xyz.qdrant.io` |
| `QDRANT_API_KEY` | Yes | Qdrant API key | `...` |
| `NEON_DATABASE_URL` | Yes | PostgreSQL connection string | `postgresql://...` |
| `APP_ENV` | No | Environment mode | `development`, `production` |
| `LOG_LEVEL` | No | Logging level | `INFO`, `DEBUG` |
| `CORS_ORIGINS` | Yes* | Allowed CORS origins | `https://example.com,https://other.com` or `*` |
| `LLM_PROVIDER_FALLBACK_1` | No | First fallback LLM | `openai` |
| `LLM_API_KEY_FALLBACK_1` | No | Fallback 1 API key | `sk-...` |
| `COHERE_API_KEY` | No | Cohere API key | `...` |

*Required for production with separate frontend

### Frontend Environment Variables

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `REACT_APP_API_BASE_URL` | Yes | Backend API URL | `https://your-space.hf.space` or `http://localhost:8000` |
| `REACT_APP_API_TIMEOUT` | No | Request timeout (ms) | `30000` |
| `REACT_APP_DEBUG` | No | Enable debug mode | `true`, `false` |

---

## Switching Between Environments

### Local Development

**Backend:**
```bash
cd RAG-backend
cp .env.example .env
# Edit .env: set CORS_ORIGINS=*
uvicorn app.main:app --reload --port 8000
```

**Frontend:**
```bash
cd "frontend/Physical AI and Robotics"
cp .env.local.example .env
# Verify: REACT_APP_API_BASE_URL=http://localhost:8000
npm start
```

### GitHub Pages + Hugging Face (Production)

**Backend:**
- Use Hugging Face Secrets with production values
- Set `CORS_ORIGINS=https://asmaiqbal01.github.io,...`

**Frontend:**
```bash
cd "frontend/Physical AI and Robotics"
cp .env.example .env
# Set: REACT_APP_API_BASE_URL=https://your-space.hf.space
npm run deploy
```

### Quick Switch Script

Create `switch-env.sh`:

```bash
#!/bin/bash

if [ "$1" == "local" ]; then
    echo "Switching to LOCAL development..."
    cd "frontend/Physical AI and Robotics"
    cp .env.local.example .env
    echo "✓ Frontend configured for localhost:8000"
elif [ "$1" == "production" ]; then
    echo "Switching to PRODUCTION..."
    cd "frontend/Physical AI and Robotics"
    cp .env.example .env
    echo "✓ Frontend configured for Hugging Face"
else
    echo "Usage: ./switch-env.sh [local|production]"
fi
```

Usage:
```bash
chmod +x switch-env.sh
./switch-env.sh local     # For local development
./switch-env.sh production # For production deployment
```

---

## Post-Deployment

### 1. Monitoring

**Hugging Face Space:**
- Monitor build logs
- Check runtime logs for errors
- Monitor space status

**GitHub Pages:**
- Check GitHub Actions for deployment status
- Monitor via browser console

### 2. Testing

Follow the [Testing Guide](TESTING_GUIDE.md) to verify all functionality.

### 3. Performance Optimization

**Backend:**
- Monitor response times
- Optimize Qdrant queries
- Consider caching strategies
- Monitor LLM API usage and costs

**Frontend:**
- Enable production builds
- Optimize bundle size
- Implement code splitting
- Add service worker for offline support

### 4. Security

**Backend:**
- Restrict CORS to specific origins
- Rotate API keys regularly
- Monitor for unauthorized access
- Rate limit API endpoints

**Frontend:**
- Don't commit .env files
- Use environment variables for all configs
- Implement CSP headers

### 5. Maintenance

**Regular Updates:**
```bash
# Update backend dependencies
cd RAG-backend
pip install --upgrade -r requirements.txt

# Update frontend dependencies
cd "frontend/Physical AI and Robotics"
npm update

# Rebuild and redeploy
npm run deploy
```

**Monitor:**
- Qdrant storage usage
- Neon database connections
- LLM API quotas
- Hugging Face Space resource usage

---

## Troubleshooting

### Common Issues

**1. CORS Errors**
```
Access to fetch at 'https://...' from origin 'https://...' has been blocked by CORS policy
```

**Solution:**
- Verify `CORS_ORIGINS` in backend includes your frontend URL
- Ensure no trailing slashes
- Check that backend is actually using the environment variable

**2. Backend Won't Start on Hugging Face**

**Check:**
- Build logs in Space settings
- All required secrets are configured
- `requirements.txt` has no missing dependencies
- `app.py` exists and is correct

**3. Frontend Shows "Offline"**

**Check:**
- Backend URL in `.env` is correct and accessible
- Backend health endpoint returns 200
- No CORS errors in browser console
- Hugging Face Space is not sleeping (free tier)

**4. Slow Responses**

**Check:**
- LLM provider status and quotas
- Qdrant query performance
- Network latency
- Consider implementing caching

### Getting Help

- Check logs in Hugging Face Space settings
- Use browser DevTools for frontend issues
- Test backend endpoints directly with curl
- Refer to [Testing Guide](TESTING_GUIDE.md)

---

## Rollback Strategy

### Frontend Rollback

```bash
# Revert to previous commit
git log # Find commit hash
git revert <commit-hash>
npm run deploy
```

### Backend Rollback

```bash
cd "C:\Users\umzaid\Desktop\huggingface"
git log
git revert <commit-hash>
git push
```

---

## CI/CD Setup (Optional)

### GitHub Actions for Frontend

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [ main ]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '20'
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

---

## Summary

You now have:

✅ Backend deployed to Hugging Face Spaces
✅ Frontend deployed to GitHub Pages
✅ Environment configurations for local and production
✅ CORS properly configured
✅ API integration working
✅ Testing and monitoring strategies

Next steps:
1. Follow [Testing Guide](TESTING_GUIDE.md) to verify deployment
2. Set up monitoring and alerts
3. Implement analytics
4. Gather user feedback
5. Iterate and improve

---

For additional help:
- Backend README: `RAG-backend/README.md`
- Testing Guide: `TESTING_GUIDE.md`
- Frontend docs: `frontend/Physical AI and Robotics/README.md`
