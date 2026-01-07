// src/utils/apiClient.js
import axios from "axios";

// Read from environment variables
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || "http://localhost:8000";
const API_TIMEOUT = parseInt(process.env.REACT_APP_API_TIMEOUT) || 30000;

// Create Axios instance
const apiClient = axios.create({
  baseURL: API_BASE_URL,
  timeout: API_TIMEOUT,
  headers: {
    "Content-Type": "application/json",
  },
});

// Response interceptor for error handling
apiClient.interceptors.response.use(
  (response) => response,
  (error) => {
    console.error("[API ERROR]", error.response || error.message);
    return Promise.reject(error);
  }
);

export default apiClient;

// ==========================
// Helper API functions
// ==========================

// Health check
export const getHealth = async () => {
  try {
    const res = await apiClient.get("/health");
    return res.data;
  } catch (err) {
    return { status: "error", message: err.message };
  }
};

// Ingest document
export const ingestDocument = async (content, metadata = {}) => {
  try {
    const res = await apiClient.post("/ingest", {
      content,
      metadata,
    });
    return res.data;
  } catch (err) {
    console.error("Ingest failed:", err.message);
    throw err;
  }
};

// Query RAG system
export const queryRAG = async (query, top_k = 5) => {
  try {
    const res = await apiClient.post("/query", {
      query,
      top_k,
    });
    return res.data;
  } catch (err) {
    console.error("Query failed:", err.message);
    throw err;
  }
};
