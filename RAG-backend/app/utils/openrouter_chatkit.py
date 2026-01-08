# backend/app/utils/openrouter_chatkit.py
"""
ChatKit Session Analysis using OpenRouter.

This module provides tools for analyzing ChatKit sessions, diagnosing
cancellation reasons, and providing administrative insights.
"""

import os
import time
import json
from datetime import datetime
from typing import Dict, Optional
from openai import OpenAI

from app.config import settings
from app.utils.logging import get_logger

logger = get_logger(__name__)


class ChatKitAnalyzer:
    """Analyzer for ChatKit sessions using OpenRouter LLM."""

    def __init__(self):
        """Initialize OpenRouter client with configuration from settings."""
        self.client = OpenAI(
            api_key=settings.llm_api_key,  # Use existing config
            base_url=settings.llm_base_url  # ✅ Correct parameter name
        )
        self.model = settings.llm_model
        logger.info("ChatKitAnalyzer initialized", {
            "provider": settings.llm_provider,
            "model": self.model
        })

    def analyze_session(self, session: Dict) -> str:
        """
        Analyze a ChatKit session and provide insights on cancellation.

        Args:
            session: ChatKit session JSON dictionary

        Returns:
            String with analysis and suggestions

        Example:
            >>> analyzer = ChatKitAnalyzer()
            >>> session = {"id": "cksess_123", "status": "cancelled", ...}
            >>> analysis = analyzer.analyze_session(session)
            >>> print(analysis)
        """
        try:
            # Convert cancelled_at timestamp to human-readable time
            cancelled_at = session.get("cancelled_at", 0)
            if cancelled_at:
                cancelled_time = datetime.fromtimestamp(cancelled_at).strftime('%Y-%m-%d %H:%M:%S UTC')
            else:
                cancelled_time = "Not available"

            # Extract key metrics
            max_requests = session.get("max_requests_per_1_minute", "Not set")
            current_requests = session.get("current_requests", 0)
            ttl_seconds = session.get("ttl_seconds", 0)
            status = session.get("status", "unknown")

            # Construct detailed prompt for OpenRouter
            prompt = f"""
You are an expert ChatKit session analyst. Analyze this session and provide detailed insights.

**Session Data:**
{json.dumps(session, indent=2)}

**Parsed Information:**
- Status: {status}
- Cancelled at: {cancelled_time}
- Max requests per minute: {max_requests}
- Current requests: {current_requests}
- TTL (seconds): {ttl_seconds}

**Analysis Required:**

1. **Root Cause Analysis**
   - Why was this session cancelled?
   - Was it due to rate limiting (max_requests_per_1_minute)?
   - Was it a TTL expiration?
   - Was it manually cancelled?

2. **Impact Assessment**
   - How many requests were processed before cancellation?
   - Was the workflow partially completed?
   - What data might be lost?

3. **Recovery Steps**
   - How can the admin resume this session?
   - Should they create a new session for the same workflow?
   - What parameters should be adjusted?

4. **Prevention Recommendations**
   - Should rate limits be increased?
   - Should TTL be extended?
   - Are there workflow optimization opportunities?

Provide clear, actionable advice for administrators.
"""

            # Send prompt to OpenRouter
            logger.info("Analyzing ChatKit session", {
                "session_id": session.get("id"),
                "status": status
            })

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert ChatKit session analyst helping administrators troubleshoot and optimize their workflows."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.2,
                max_tokens=2000
            )

            analysis = response.choices[0].message.content

            logger.info("ChatKit session analysis completed", {
                "session_id": session.get("id"),
                "analysis_length": len(analysis)
            })

            return analysis

        except Exception as e:
            logger.error("Failed to analyze ChatKit session", {
                "error": str(e),
                "session_id": session.get("id", "unknown")
            })
            raise

    def batch_analyze(self, sessions: list[Dict]) -> Dict[str, str]:
        """
        Analyze multiple ChatKit sessions and return insights for each.

        Args:
            sessions: List of ChatKit session dictionaries

        Returns:
            Dictionary mapping session IDs to analysis results
        """
        results = {}

        for session in sessions:
            session_id = session.get("id", "unknown")
            try:
                results[session_id] = self.analyze_session(session)
            except Exception as e:
                results[session_id] = f"Analysis failed: {str(e)}"

        return results

    def generate_summary_report(self, sessions: list[Dict]) -> str:
        """
        Generate a summary report for multiple sessions.

        Args:
            sessions: List of ChatKit session dictionaries

        Returns:
            String with summary analysis across all sessions
        """
        # Aggregate statistics
        total_sessions = len(sessions)
        cancelled_sessions = sum(1 for s in sessions if s.get("status") == "cancelled")
        rate_limited = sum(1 for s in sessions
                          if s.get("current_requests", 0) > s.get("max_requests_per_1_minute", float('inf')))

        summary_prompt = f"""
Analyze these {total_sessions} ChatKit sessions and provide a high-level summary:

**Overall Statistics:**
- Total sessions: {total_sessions}
- Cancelled sessions: {cancelled_sessions}
- Rate-limited sessions: {rate_limited}

**Session Details:**
{json.dumps(sessions, indent=2)}

**Provide:**
1. Common patterns across cancelled sessions
2. Overall health assessment
3. Top recommendations for optimization
4. Risk areas requiring attention
"""

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {
                    "role": "system",
                    "content": "You are a ChatKit operations analyst providing executive summaries."
                },
                {
                    "role": "user",
                    "content": summary_prompt
                }
            ],
            temperature=0.3,
            max_tokens=1500
        )

        return response.choices[0].message.content


# Standalone function for backward compatibility
def analyze_chatkit_session(session: dict) -> str:
    """
    Analyze a ChatKit session and provide insights on cancellation.

    Args:
        session: ChatKit session JSON

    Returns:
        A string with analysis and suggestions
    """
    analyzer = ChatKitAnalyzer()
    return analyzer.analyze_session(session)


# Example usage and testing
if __name__ == "__main__":
    # Example session data
    example_session = {
        "id": "cksess_123",
        "object": "chatkit.session",
        "workflow": {
            "id": "workflow_alpha",
            "version": "1"
        },
        "scope": {
            "customer_id": "cust_456"
        },
        "max_requests_per_1_minute": 30,
        "current_requests": 35,  # Exceeded limit
        "ttl_seconds": 900,
        "status": "cancelled",
        "cancelled_at": 1712345678,
        "reason": "Rate limit exceeded"
    }

    print("\n" + "="*60)
    print("ChatKit Session Analysis")
    print("="*60 + "\n")

    try:
        analyzer = ChatKitAnalyzer()
        analysis = analyzer.analyze_session(example_session)
        print(analysis)

        print("\n" + "="*60)
        print("Analysis Complete")
        print("="*60 + "\n")

    except Exception as e:
        print(f"Error during analysis: {e}")
