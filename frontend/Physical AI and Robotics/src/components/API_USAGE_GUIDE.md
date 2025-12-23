# API Usage Guide

This guide demonstrates how to use the RAG Backend API in your React application.

## Table of Contents
- [Configuration](#configuration)
- [API Functions](#api-functions)
- [Example Components](#example-components)
- [Error Handling](#error-handling)
- [TypeScript Types](#typescript-types)

## Configuration

### Environment Variables

Create a `.env` file in your project root (see `.env.example`):

```env
# API Base URL
REACT_APP_API_BASE_URL=https://asmaiqbal000-hackathon1-ai-book.hf.space

# Optional: API timeout in milliseconds
REACT_APP_API_TIMEOUT=30000
```

### Development vs Production

The API automatically defaults to your HuggingFace Space:

```typescript
// Default production URL
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL ||
  'https://asmaiqbal000-hackathon1-ai-book.hf.space';
```

For local development, create a `.env.local`:

```env
REACT_APP_API_BASE_URL=http://localhost:8000
```

## API Functions

### 1. Health Check

Check if the backend services are running:

```typescript
import { checkHealth } from '../services/api';

const checkBackendHealth = async () => {
  try {
    const health = await checkHealth();
    console.log('Status:', health.status);
    console.log('Services:', health.services);
    // Output:
    // Status: ok
    // Services: { qdrant: "Connected", neon: "Connected", llm: "Available" }
  } catch (error) {
    console.error('Health check failed:', error);
  }
};
```

### 2. Query RAG System

Query the RAG system for answers:

```typescript
import { queryRAG } from '../services/api';

const askQuestion = async () => {
  try {
    const response = await queryRAG({
      question: "What is physical AI?",
      chapter: 1,  // Optional: filter by chapter
      top_k: 5,    // Optional: number of chunks to retrieve
    });

    console.log('Answer:', response.answer);
    console.log('Mode:', response.mode);
    console.log('Citations:', response.citations);
    console.log('Metadata:', response.metadata);
  } catch (error) {
    console.error('Query failed:', error);
  }
};
```

#### With Selected Text (Context-Aware Mode)

```typescript
const response = await queryRAG({
  question: "Explain this concept",
  selected_text: "Physical AI combines robotics with artificial intelligence...",
  top_k: 3,
});
// response.mode will be "selected_text_only"
```

### 3. Get Available Chapters

Retrieve all available chapters:

```typescript
import { getChapters } from '../services/api';

const fetchChapters = async () => {
  try {
    const chapters = await getChapters();
    chapters.forEach(chapter => {
      console.log(`Chapter ${chapter.chapter}: ${chapter.title}`);
      console.log('Sections:', chapter.sections);
    });
  } catch (error) {
    console.error('Failed to fetch chapters:', error);
  }
};
```

### 4. Ingest New Content

Add new content to the RAG system:

```typescript
import { ingestContent } from '../services/api';

const addNewContent = async () => {
  try {
    const response = await ingestContent({
      content: `
# Chapter 1: Introduction to Physical AI

Physical AI represents the convergence of robotics and artificial intelligence...

## Key Concepts
- Sensor integration
- Real-time processing
- Embodied intelligence
      `,
      metadata: {
        title: "Introduction to Physical AI",
        chapter: 1,
        section: "Overview",
        page_start: 1,
        page_end: 10,
      },
    });

    console.log('Document ID:', response.doc_id);
    console.log('Chunks created:', response.chunks_created);
    console.log('Embeddings stored:', response.embeddings_stored);
  } catch (error) {
    console.error('Ingestion failed:', error);
  }
};
```

### 5. Re-Embed Document

Refresh embeddings for an existing document:

```typescript
import { reEmbedDocument } from '../services/api';

const refreshEmbeddings = async (documentId: string) => {
  try {
    const response = await reEmbedDocument({
      doc_id: documentId,
    });

    console.log('Re-embedded document:', response.doc_id);
    console.log('Embeddings updated:', response.embeddings_updated);
  } catch (error) {
    console.error('Re-embedding failed:', error);
  }
};
```

## Example Components

Two ready-to-use React components are provided:

### ContentIngest Component

Located at `src/components/ContentIngest.tsx`

A complete form-based UI for ingesting new content:

```typescript
import ContentIngest from './components/ContentIngest';

function App() {
  return (
    <div>
      <ContentIngest />
    </div>
  );
}
```

Features:
- Form validation
- Loading states
- Error handling
- Success feedback with ingestion stats
- Automatic form reset on success

### DocumentReEmbed Component

Located at `src/components/DocumentReEmbed.tsx`

A UI for re-embedding existing documents:

```typescript
import DocumentReEmbed from './components/DocumentReEmbed';

function App() {
  return (
    <div>
      <DocumentReEmbed />
    </div>
  );
}
```

Features:
- Simple document ID input
- Loading states
- Error handling
- Success feedback
- Usage guidance

## Error Handling

All API functions use the custom `ApiError` class:

```typescript
import { ApiError } from '../services/api';

try {
  const response = await queryRAG({ question: "..." });
} catch (error) {
  if (error instanceof ApiError) {
    console.error('API Error:', error.message);
    console.error('Status Code:', error.status);
    console.error('Detail:', error.detail);

    // Handle specific errors
    if (error.status === 404) {
      console.log('Resource not found');
    } else if (error.status === 500) {
      console.log('Server error');
    } else if (error.status === 504) {
      console.log('Request timeout');
    }
  } else {
    console.error('Unexpected error:', error);
  }
}
```

## TypeScript Types

All API functions are fully typed. Import the types you need:

```typescript
import {
  QueryRequest,
  QueryResponse,
  Citation,
  Chapter,
  HealthResponse,
  IngestRequest,
  IngestResponse,
  DocumentMetadata,
  EmbedRequest,
  EmbedResponse,
  ApiError,
} from '../services/api';

// Example usage
const handleQuery = async (request: QueryRequest): Promise<QueryResponse> => {
  return await queryRAG(request);
};
```

### Key Types

```typescript
// Query Request
interface QueryRequest {
  question: string;
  chapter?: number;
  top_k?: number;
  selected_text?: string;
}

// Document Metadata
interface DocumentMetadata {
  title: string;
  chapter: number; // 1-100
  section?: string;
  page_start?: number;
  page_end?: number;
}

// Ingest Request
interface IngestRequest {
  content: string;
  metadata: DocumentMetadata;
}

// Re-Embed Request
interface EmbedRequest {
  doc_id: string;
}
```

## Best Practices

1. **Always use try-catch blocks** when calling API functions
2. **Check error types** using `instanceof ApiError` for better error messages
3. **Implement loading states** to provide user feedback during API calls
4. **Use TypeScript types** for better autocomplete and type safety
5. **Set appropriate timeouts** based on expected operation duration
6. **Handle network failures** gracefully with user-friendly messages
7. **Validate input** before sending requests to reduce unnecessary API calls

## Debugging

Get the current API configuration:

```typescript
import { getApiBaseUrl, isLocalDevelopment } from '../services/api';

console.log('API Base URL:', getApiBaseUrl());
console.log('Local Development:', isLocalDevelopment());
```

## Next Steps

- Add the example components to your routing
- Customize the UI to match your application's design
- Implement additional features like batch ingestion
- Add caching for frequently accessed data
- Implement retry logic for failed requests
