#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple ChatKit Analytics Test
Run from RAG-backend directory: python test_simple.py
"""

from app.utils.openrouter_chatkit import analyze_chatkit_session

example_session = {
    "id": "cksess_123",
    "object": "chatkit.session",
    "workflow": {"id": "workflow_alpha", "version": "1"},
    "scope": {"customer_id": "cust_456"},
    "max_requests_per_1_minute": 30,
    "ttl_seconds": 900,
    "status": "cancelled",
    "cancelled_at": 1712345678,
    "current_requests": 35
}

def test_single_analysis():
    result = analyze_chatkit_session(example_session)
    assert "rate limit" in result.lower() or "exceeded" in result.lower()
    print("[PASS] Single Analysis")
    print("\nAnalysis Preview:")
    print(result[:200] + "...")

if __name__ == "__main__":
    print("\n[TEST] Testing ChatKit Analysis...\n")
    test_single_analysis()
    print("\n[SUCCESS] Test passed!\n")
