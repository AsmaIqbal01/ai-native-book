import React, { useState } from 'react';
import { ingestContent, IngestRequest, IngestResponse, ApiError } from '../services/api';

/**
 * ContentIngest Component
 *
 * Demonstrates how to use the ingestContent API to add new content to the RAG system.
 * This component allows users to ingest markdown/HTML content with metadata.
 */
export const ContentIngest: React.FC = () => {
  const [content, setContent] = useState('');
  const [title, setTitle] = useState('');
  const [chapter, setChapter] = useState<number>(1);
  const [section, setSection] = useState('');
  const [pageStart, setPageStart] = useState<number | undefined>();
  const [pageEnd, setPageEnd] = useState<number | undefined>();
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState<IngestResponse | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const request: IngestRequest = {
        content,
        metadata: {
          title,
          chapter,
          ...(section && { section }),
          ...(pageStart && { page_start: pageStart }),
          ...(pageEnd && { page_end: pageEnd }),
        },
      };

      const response = await ingestContent(request);
      setResult(response);

      // Clear form on success
      setContent('');
      setTitle('');
      setChapter(1);
      setSection('');
      setPageStart(undefined);
      setPageEnd(undefined);
    } catch (err) {
      if (err instanceof ApiError) {
        setError(`${err.message} (Status: ${err.status})`);
      } else if (err instanceof Error) {
        setError(err.message);
      } else {
        setError('An unexpected error occurred');
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="content-ingest">
      <h2>Ingest New Content</h2>
      <p>Add new content to the RAG system for querying.</p>

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="title">Title *</label>
          <input
            id="title"
            type="text"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            required
            placeholder="e.g., Introduction to Physical AI"
          />
        </div>

        <div className="form-group">
          <label htmlFor="chapter">Chapter * (1-100)</label>
          <input
            id="chapter"
            type="number"
            min="1"
            max="100"
            value={chapter}
            onChange={(e) => setChapter(parseInt(e.target.value))}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="section">Section (optional)</label>
          <input
            id="section"
            type="text"
            value={section}
            onChange={(e) => setSection(e.target.value)}
            placeholder="e.g., Overview"
          />
        </div>

        <div className="form-row">
          <div className="form-group">
            <label htmlFor="pageStart">Page Start (optional)</label>
            <input
              id="pageStart"
              type="number"
              min="1"
              value={pageStart || ''}
              onChange={(e) => setPageStart(e.target.value ? parseInt(e.target.value) : undefined)}
              placeholder="1"
            />
          </div>

          <div className="form-group">
            <label htmlFor="pageEnd">Page End (optional)</label>
            <input
              id="pageEnd"
              type="number"
              min="1"
              value={pageEnd || ''}
              onChange={(e) => setPageEnd(e.target.value ? parseInt(e.target.value) : undefined)}
              placeholder="10"
            />
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="content">Content * (Markdown/HTML)</label>
          <textarea
            id="content"
            value={content}
            onChange={(e) => setContent(e.target.value)}
            required
            rows={10}
            placeholder="# Chapter Title&#10;&#10;Your content here in markdown or HTML format..."
          />
        </div>

        <button type="submit" disabled={loading}>
          {loading ? 'Ingesting...' : 'Ingest Content'}
        </button>
      </form>

      {error && (
        <div className="alert alert-error">
          <strong>Error:</strong> {error}
        </div>
      )}

      {result && (
        <div className="alert alert-success">
          <h3>Success!</h3>
          <ul>
            <li><strong>Status:</strong> {result.status}</li>
            <li><strong>Document ID:</strong> {result.doc_id}</li>
            <li><strong>Chunks Created:</strong> {result.chunks_created}</li>
            <li><strong>Embeddings Stored:</strong> {result.embeddings_stored}</li>
          </ul>
        </div>
      )}
    </div>
  );
};

export default ContentIngest;
