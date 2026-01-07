# Research Findings: Spec 1 - Website Ingestion, Embeddings & Vector Storage

## Decision: Web Crawling Strategy
**Rationale**: For robust website crawling, we'll use a combination of requests/BeautifulSoup for simple sites and Playwright for JavaScript-heavy sites. This provides both efficiency for simple sites and capability for complex sites.
**Alternatives considered**:
- Scrapy: More complex setup but very powerful
- Selenium: Heavier than Playwright but more mature
- Simple requests: Can't handle JavaScript sites

## Decision: Content Extraction Method
**Rationale**: Use newspaper3k for content extraction as it's specifically designed for extracting main content from web pages while filtering out navigation, ads, etc. Alternatively, use Trafilatura for more customizable extraction.
**Alternatives considered**:
- BeautifulSoup with custom parsing: Requires more manual work
- Readability: Good but less maintained
- Trafilatura: More configurable but potentially more complex

## Decision: Cohere Embedding Model
**Rationale**: Use Cohere's embed-multilingual-v3.0 model as it provides good performance for technical documentation and supports multiple languages, which is important for the book content.
**Alternatives considered**:
- OpenAI embeddings: More expensive and vendor lock-in
- Sentence Transformers: Open source but potentially slower
- Cohere embed-english-v3.0: Less suitable for multilingual content

## Decision: Qdrant Configuration
**Rationale**: Use Qdrant in local mode for development and cluster mode for production. Configure with cosine distance metric for semantic similarity.
**Alternatives considered**:
- Pinecone: Managed but vendor-specific
- Weaviate: Good alternative but more complex schema
- Elasticsearch: Good for text but not optimized for vector search

## Decision: Chunking Strategy
**Rationale**: Use recursive character splitting with 512-1024 token chunks and 10% overlap to maintain context while ensuring semantic coherence.
**Alternatives considered**:
- Sentence-based splitting: May break up related content
- Fixed character length: May cut in inappropriate places
- Semantic splitting: More complex but potentially better context

## Decision: Rate Limiting & Crawl Delays
**Rationale**: Implement polite crawling with configurable delays (1-3 seconds) and respect robots.txt. Include exponential backoff for rate limits.
**Alternatives considered**:
- No rate limiting: Could be blocked by sites
- Fixed delays: Less adaptive to site responses
- IP rotation: More complex but allows higher throughput