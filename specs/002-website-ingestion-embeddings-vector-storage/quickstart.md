# Quickstart Guide: Spec 1 - Website Ingestion, Embeddings & Vector Storage

## Overview
This guide will help you set up and run the website ingestion pipeline that crawls websites, extracts content, generates embeddings using Cohere, and stores them in Qdrant.

## Prerequisites
- Python 3.9+
- Cohere API key
- Qdrant instance (local or cloud)
- Required Python packages (see requirements.txt)

## Environment Setup

1. **Set up environment variables**:
```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="http://localhost:6333"  # or your Qdrant cloud URL
export QDRANT_API_KEY="your-qdrant-api-key"  # if using cloud
```

2. **Install dependencies**:
```bash
pip install -r requirements.txt
# Or install specific packages:
pip install cohere qdrant-client requests beautifulsoup4 newspaper3k playwright python-dotenv
```

## Running the Ingestion Pipeline

### 1. Configure the Ingestion Job

Create a configuration file or use command-line arguments:

```python
from app.ingestion.ingestion_job import IngestionJob

config = {
    "urls": ["https://example.com/docs", "https://example.com/blog"],
    "sitemap_url": "https://example.com/sitemap.xml",  # Optional
    "chunk_size": 512,
    "chunk_overlap": 50,
    "embedding_model": "embed-multilingual-v3.0"
}

job = IngestionJob(config)
```

### 2. Execute the Pipeline

```python
# Start the ingestion job
job.start()

# Monitor progress
while job.status != "completed":
    print(f"Progress: {job.progress}")
    time.sleep(5)

print(f"Ingestion completed! {job.report.total_chunks} chunks ingested.")
```

### 3. Verify Results

Check that embeddings are stored in Qdrant:

```python
from qdrant_client import QdrantClient

client = QdrantClient(url="http://localhost:6333")
collection_name = "ingested_content"

# Count vectors in collection
count = client.count(collection_name=collection_name)
print(f"Vectors in collection: {count}")
```

## Example Usage

### Simple Website Ingestion
```bash
python -m app.ingestion.cli --urls "https://docs.anthropic.com" --chunk-size 512
```

### With Sitemap
```bash
python -m app.ingestion.cli --sitemap "https://example.com/sitemap.xml" --urls "https://example.com"
```

### Advanced Configuration
```bash
python -m app.ingestion.cli \
  --urls "https://docs.anthropic.com" "https://docs.anthropic.com/api" \
  --chunk-size 1024 \
  --chunk-overlap 100 \
  --model "embed-multilingual-v3.0" \
  --collection-name "my_docs"
```

## API Endpoints

### Start Ingestion Job
```
POST /api/v1/ingestion/jobs
Content-Type: application/json

{
  "urls": ["https://example.com"],
  "sitemap_url": "https://example.com/sitemap.xml",
  "chunk_size": 512,
  "chunk_overlap": 50,
  "embedding_model": "embed-multilingual-v3.0"
}
```

### Get Job Status
```
GET /api/v1/ingestion/jobs/{job_id}
```

### Get Ingestion Reports
```
GET /api/v1/ingestion/reports
```

## Troubleshooting

### Common Issues

1. **Rate Limiting**: If you encounter rate limiting, adjust the crawl delays in the configuration
2. **JavaScript-heavy Sites**: For sites that require JavaScript rendering, ensure Playwright is properly installed
3. **API Limits**: Monitor your Cohere API usage to avoid hitting rate limits

### Checking Logs
```bash
# Check application logs
tail -f logs/ingestion.log

# Or check structured logs in your logging system
```

## Next Steps

After successful ingestion:
1. Test semantic search with the stored embeddings
2. Integrate with your RAG application
3. Set up monitoring for production use