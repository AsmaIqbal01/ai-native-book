# Data Model: Spec 1 - Website Ingestion, Embeddings & Vector Storage

## Entities

### CrawledContent
- **id**: string (deterministic hash of URL + content)
- **url**: string (source URL)
- **title**: string (page title)
- **content**: string (extracted clean text)
- **metadata**: object (author, date, tags, etc.)
- **created_at**: datetime (timestamp of crawling)
- **updated_at**: datetime (timestamp of last update)
- **content_type**: string (html, pdf, etc.)
- **status**: enum (success, failed, partial)

### Chunk
- **id**: string (deterministic hash of content + parent_id)
- **parent_id**: string (reference to CrawledContent.id)
- **content**: string (chunked text content)
- **chunk_index**: integer (position in original document)
- **token_count**: integer (number of tokens in chunk)
- **created_at**: datetime (timestamp of chunking)

### Embedding
- **id**: string (same as Chunk.id for consistency)
- **chunk_id**: string (reference to Chunk.id)
- **vector**: array<float> (embedding values)
- **model**: string (embedding model used)
- **created_at**: datetime (timestamp of embedding generation)

### IngestionJob
- **id**: string (unique identifier)
- **name**: string (job name/description)
- **urls**: array<string> (list of URLs to crawl)
- **sitemap_url**: string (optional sitemap URL)
- **chunk_size**: integer (target chunk size)
- **chunk_overlap**: integer (overlap between chunks)
- **embedding_model**: string (model to use for embeddings)
- **status**: enum (pending, running, completed, failed)
- **progress**: object (stats about crawling progress)
- **created_at**: datetime (timestamp of job creation)
- **completed_at**: datetime (timestamp of job completion)
- **error**: string (error message if failed)

### IngestionReport
- **id**: string (unique identifier)
- **job_id**: string (reference to IngestionJob.id)
- **total_urls**: integer (total URLs processed)
- **successful_crawls**: integer (number of successful crawls)
- **failed_crawls**: integer (number of failed crawls)
- **total_chunks**: integer (total chunks created)
- **total_embeddings**: integer (total embeddings generated)
- **start_time**: datetime (when job started)
- **end_time**: datetime (when job ended)
- **duration**: float (duration in seconds)
- **errors**: array<object> (list of errors encountered)

## Relationships

```
IngestionJob (1) -> (0..n) CrawledContent
CrawledContent (1) -> (1..n) Chunk
Chunk (1) -> (1) Embedding
IngestionJob (1) -> (0..1) IngestionReport
```

## Validation Rules

### CrawledContent
- URL must be a valid URL format
- Content must not be empty
- Created_at must be set on creation

### Chunk
- Content must not exceed maximum token count
- Chunk_index must be >= 0
- Parent_id must reference an existing CrawledContent

### Embedding
- Vector must have consistent dimensions based on model
- Chunk_id must reference an existing Chunk
- Model must be a supported embedding model

### IngestionJob
- URLs list must not be empty
- Chunk_size must be > 0
- Chunk_overlap must be >= 0 and < chunk_size