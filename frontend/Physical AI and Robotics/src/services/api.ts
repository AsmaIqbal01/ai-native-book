/**
 * API Service Layer for RAG Backend Integration
 *
 * This service handles all API calls to the FastAPI RAG backend.
 * It supports both local development and production environments.
 */

// Safe way to access environment variables in browser context
const getEnvVar = (key: string, defaultValue: string): string => {
  // Check if we're in a Node.js environment with process.env available
  if (typeof process !== 'undefined' && process.env) {
    return process.env[key] || defaultValue;
  }
  // Fallback for browser environment
  return defaultValue;
};

// API Configuration
const API_BASE_URL = getEnvVar('REACT_APP_API_BASE_URL', 'https://asmaiqbal000-hackathon1-ai-book.hf.space');
const API_TIMEOUT = parseInt(getEnvVar('REACT_APP_API_TIMEOUT', '30000'), 10);

// Types
export interface QueryRequest {
  question: string;
  chapter?: number;
  top_k?: number;
  selected_text?: string;
}

export interface Citation {
  chapter?: number;
  section?: string;
  page?: number;
  chunk_text: string;
}

export interface QueryResponse {
  answer: string;
  mode: 'normal_rag' | 'selected_text_only';
  citations: Citation[];
  metadata: {
    chunks_retrieved: number;
    chunk_ids: string[];
    latency_ms: number;
    provider_used: string;
  };
}

export interface Chapter {
  chapter: number;
  title: string;
  sections: string[];
}

export interface HealthResponse {
  status: string;
  timestamp: string;
  services: {
    qdrant: string;
    neon: string;
    llm: string;
  };
}

export interface DocumentMetadata {
  title: string;
  chapter: number; // 1-100
  section?: string;
  page_start?: number;
  page_end?: number;
}

export interface IngestRequest {
  content: string;
  metadata: DocumentMetadata;
}

export interface IngestResponse {
  status: string;
  doc_id: string;
  chunks_created: number;
  embeddings_stored: number;
}

export interface EmbedRequest {
  doc_id: string;
}

export interface EmbedResponse {
  status: string;
  doc_id: string;
  embeddings_updated: number;
}

// Error class for API errors
export class ApiError extends Error {
  constructor(
    message: string,
    public status?: number,
    public detail?: string
  ) {
    super(message);
    this.name = 'ApiError';
  }
}

// Helper function to handle fetch with timeout
async function fetchWithTimeout(
  url: string,
  options: RequestInit = {},
  timeout: number = API_TIMEOUT
): Promise<Response> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(timeoutId);
    return response;
  } catch (error) {
    clearTimeout(timeoutId);
    if (error instanceof Error && error.name === 'AbortError') {
      throw new ApiError('Request timeout', 504, 'The request took too long to complete');
    }
    throw error;
  }
}

// Helper function to handle API responses
async function handleResponse<T>(response: Response): Promise<T> {
  if (!response.ok) {
    let detail = `HTTP ${response.status}: ${response.statusText}`;
    try {
      const errorData = await response.json();
      detail = errorData.detail || detail;
    } catch {
      // If response is not JSON, use status text
    }
    throw new ApiError(`API Error: ${detail}`, response.status, detail);
  }

  try {
    return await response.json();
  } catch (error) {
    throw new ApiError('Failed to parse response', response.status);
  }
}

/**
 * Health Check
 * GET /health
 */
export async function checkHealth(): Promise<HealthResponse> {
  const response = await fetchWithTimeout(`${API_BASE_URL}/health`);
  return handleResponse<HealthResponse>(response);
}

/**
 * Query the RAG system
 * POST /query
 */
export async function queryRAG(request: QueryRequest): Promise<QueryResponse> {
  const response = await fetchWithTimeout(`${API_BASE_URL}/query`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });
  return handleResponse<QueryResponse>(response);
}

/**
 * Get available chapters
 * GET /chapters
 */
export async function getChapters(): Promise<Chapter[]> {
  const response = await fetchWithTimeout(`${API_BASE_URL}/chapters`);
  return handleResponse<Chapter[]>(response);
}

/**
 * Ingest new content into the RAG system
 * POST /ingest
 */
export async function ingestContent(request: IngestRequest): Promise<IngestResponse> {
  const response = await fetchWithTimeout(`${API_BASE_URL}/ingest`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });
  return handleResponse<IngestResponse>(response);
}

/**
 * Re-embed an existing document
 * POST /embed
 */
export async function reEmbedDocument(request: EmbedRequest): Promise<EmbedResponse> {
  const response = await fetchWithTimeout(`${API_BASE_URL}/embed`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });
  return handleResponse<EmbedResponse>(response);
}

/**
 * Get API base URL (useful for debugging)
 */
export function getApiBaseUrl(): string {
  return API_BASE_URL;
}

/**
 * Check if API is configured for local development
 */
export function isLocalDevelopment(): boolean {
  return API_BASE_URL.includes('localhost') || API_BASE_URL.includes('127.0.0.1');
}
