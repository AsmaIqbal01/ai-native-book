/**
 * TypeScript Type Definitions for RAG Chatbot API
 *
 * Generated from OpenAPI specification
 * Feature: 001-rag-chatbot-agent-system
 * Date: 2025-12-20
 *
 * These types ensure type safety between frontend and backend.
 */

// ============================================================================
// Core Entities (Frontend State)
// ============================================================================

export interface Message {
  id: string
  role: 'user' | 'assistant'
  content: string
  timestamp: Date
  sources?: string[]
}

export type ErrorType = 'network' | 'database' | 'llm' | 'validation' | 'agent' | 'unknown'

export interface ErrorContext {
  error_type: ErrorType
  user_safe_message: string
  retry_able: boolean
}

// ============================================================================
// API Request/Response Types
// ============================================================================

// ──────────────────────────────────────────────────────────────────────────
// Health Endpoint (FR-001)
// ──────────────────────────────────────────────────────────────────────────

export type HealthStatus = 'healthy' | 'degraded' | 'unhealthy'
export type QdrantStatus = 'connected' | 'disconnected' | 'error'
export type LLMStatus = 'available' | 'unavailable'

export interface HealthResponse {
  status: HealthStatus
  timestamp: string  // ISO 8601 format
  qdrant_status?: QdrantStatus
  qdrant_collection_count?: number
  llm_status?: LLMStatus
}

// ──────────────────────────────────────────────────────────────────────────
// Query Endpoint (FR-002)
// ──────────────────────────────────────────────────────────────────────────

export interface QueryRequest {
  question: string  // Min 1, Max 10000 characters
  session_id?: string  // Optional UUID (not used for persistence)
}

export interface QueryResponse {
  answer: string
  sources: string[]  // Array of "chapter/section" references
  confidence?: number  // 0.0 - 1.0
  processing_time_ms?: number
}

export interface ErrorResponse {
  error: ErrorType
  message: string  // User-safe message (no stack traces)
  retryable: boolean
  request_id?: string
}

// ──────────────────────────────────────────────────────────────────────────
// Chapters Endpoint (FR-003)
// ──────────────────────────────────────────────────────────────────────────

export interface Chapter {
  id: string
  title: string
  sections: string[]
  url?: string  // URL to chapter in documentation
}

export interface ChaptersResponse {
  chapters: Chapter[]
  total_count?: number
}

// ============================================================================
// Frontend-Specific Types (Zustand Store)
// ============================================================================

export interface ChatState {
  // State
  isChatOpen: boolean
  messages: Message[]
  isLoading: boolean
  error: ErrorContext | null

  // Actions
  toggleChat: () => void
  closeChat: () => void  // FR-016: Clears messages
  addMessage: (message: Omit<Message, 'id' | 'timestamp'>) => void
  clearMessages: () => void
  setLoading: (loading: boolean) => void
  setError: (error: ErrorContext | null) => void
}

// ============================================================================
// API Client Types
// ============================================================================

export interface ApiConfig {
  baseURL: string
  timeout?: number  // Request timeout in milliseconds (default: 30000)
}

export interface ApiError extends Error {
  type: ErrorType
  retryable: boolean
  request_id?: string
}

// ============================================================================
// Component Prop Types
// ============================================================================

export interface FloatingChatWidgetProps {
  initialPosition?: 'bottom-left' | 'bottom-right'
  zIndex?: number  // Default: 9999
  theme?: 'light' | 'dark'
}

export interface ChatWindowProps {
  isOpen: boolean
  onClose: () => void
  messages: Message[]
  onSendMessage: (content: string) => Promise<void>
  isLoading: boolean
  error: ErrorContext | null
}

export interface MessageBubbleProps {
  message: Message
  isLatest: boolean
}

export interface ChatInputProps {
  onSend: (content: string) => Promise<void>
  isLoading: boolean
  placeholder?: string
  maxLength?: number  // Default: 10000
}

export interface SystemNotificationProps {
  error: ErrorContext
  onDismiss: () => void
  onRetry?: () => void
}

// ============================================================================
// Utility Types
// ============================================================================

/**
 * API result wrapper for consistent error handling
 */
export type ApiResult<T> =
  | { success: true; data: T }
  | { success: false; error: ApiError }

/**
 * Message creation helper (generates id and timestamp)
 */
export function createMessage(
  role: Message['role'],
  content: string,
  sources?: string[]
): Message {
  return {
    id: crypto.randomUUID(),
    role,
    content,
    timestamp: new Date(),
    sources
  }
}

/**
 * Error type guard
 */
export function isApiError(error: unknown): error is ApiError {
  return (
    error instanceof Error &&
    'type' in error &&
    'retryable' in error
  )
}

/**
 * Validation helpers
 */
export const Validation = {
  isValidQuestion(question: string): boolean {
    return question.trim().length > 0 && question.length <= 10000
  },

  isValidSessionId(sessionId: string): boolean {
    const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i
    return uuidRegex.test(sessionId)
  },

  sanitizeQuestion(question: string): string {
    return question.trim().slice(0, 10000)
  }
}

// ============================================================================
// Constants
// ============================================================================

export const API_CONSTANTS = {
  MAX_QUESTION_LENGTH: 10000,
  DEFAULT_TIMEOUT_MS: 30000,
  RETRY_DELAY_MS: 1000,
  MAX_RETRIES: 3
} as const

export const ERROR_MESSAGES: Record<ErrorType, string> = {
  network: 'Network error. Please check your connection and try again.',
  database: 'The assistant is temporarily unavailable. Please try again later.',
  llm: 'The AI service is currently unavailable. Please try again in a moment.',
  validation: 'Invalid question. Please ensure your question is not empty and under 10,000 characters.',
  agent: 'An error occurred while processing your question. Please try again.',
  unknown: 'An unexpected error occurred. Please try again later.'
} as const

// ============================================================================
// Type Exports
// ============================================================================

export type {
  // Re-export for convenience
  Message as ChatMessage,
  ErrorContext as ChatError,
  QueryRequest as ChatQueryRequest,
  QueryResponse as ChatQueryResponse
}
