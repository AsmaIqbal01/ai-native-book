import React, { useState } from 'react';
import { reEmbedDocument, EmbedRequest, EmbedResponse, ApiError } from '../services/api';

/**
 * DocumentReEmbed Component
 *
 * Demonstrates how to use the reEmbedDocument API to re-embed existing documents.
 * This is useful when the embedding model is updated or when you need to refresh embeddings.
 */
export const DocumentReEmbed: React.FC = () => {
  const [docId, setDocId] = useState('');
  const [loading, setLoading] = useState(false);
  const [result, setResult] = useState<EmbedResponse | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    setResult(null);

    try {
      const request: EmbedRequest = {
        doc_id: docId,
      };

      const response = await reEmbedDocument(request);
      setResult(response);
      setDocId(''); // Clear form on success
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
    <div className="document-re-embed">
      <h2>Re-Embed Document</h2>
      <p>
        Re-generate embeddings for an existing document. Useful after model updates
        or to refresh the vector representations.
      </p>

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="docId">Document ID *</label>
          <input
            id="docId"
            type="text"
            value={docId}
            onChange={(e) => setDocId(e.target.value)}
            required
            placeholder="e.g., doc_abc123xyz"
          />
          <small className="help-text">
            Enter the document ID you want to re-embed. You can find this ID in the
            response when ingesting content.
          </small>
        </div>

        <button type="submit" disabled={loading || !docId.trim()}>
          {loading ? 'Re-Embedding...' : 'Re-Embed Document'}
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
            <li><strong>Embeddings Updated:</strong> {result.embeddings_updated}</li>
          </ul>
        </div>
      )}

      <div className="info-box">
        <h4>When to Re-Embed:</h4>
        <ul>
          <li>After upgrading the embedding model</li>
          <li>When you notice degraded search quality</li>
          <li>To apply new preprocessing techniques</li>
          <li>After changing chunking strategies</li>
        </ul>
      </div>
    </div>
  );
};

export default DocumentReEmbed;
