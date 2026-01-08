#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for ChatKit Analytics functionality.

Run this to verify:
1. OpenRouter integration works
2. ChatKit analyzer functions correctly
3. API endpoints are accessible
4. Analysis quality is good

Usage:
    python test_chatkit_analytics.py
"""

import sys
import json
import requests
from datetime import datetime

# Test data
TEST_SESSION = {
    "id": "cksess_test_001",
    "object": "chatkit.session",
    "workflow": {
        "id": "workflow_robotics_tutorial",
        "version": "2.1"
    },
    "scope": {
        "customer_id": "cust_test_asma_001"
    },
    "max_requests_per_1_minute": 30,
    "current_requests": 37,  # Exceeded!
    "ttl_seconds": 900,
    "status": "cancelled",
    "cancelled_at": int(datetime.now().timestamp()),
    "reason": "Rate limit exceeded - too many questions asked"
}

TEST_SESSIONS_BATCH = [
    {
        "id": "cksess_test_001",
        "object": "chatkit.session",
        "workflow": {"id": "workflow_alpha", "version": "1"},
        "scope": {"customer_id": "cust_001"},
        "max_requests_per_1_minute": 30,
        "current_requests": 35,
        "ttl_seconds": 900,
        "status": "cancelled",
        "cancelled_at": int(datetime.now().timestamp()),
        "reason": "Rate limit exceeded"
    },
    {
        "id": "cksess_test_002",
        "object": "chatkit.session",
        "workflow": {"id": "workflow_beta", "version": "1"},
        "scope": {"customer_id": "cust_002"},
        "max_requests_per_1_minute": 30,
        "current_requests": 25,
        "ttl_seconds": 900,
        "status": "cancelled",
        "cancelled_at": int(datetime.now().timestamp()),
        "reason": "TTL expired"
    }
]


def print_header(text):
    """Print formatted header."""
    print("\n" + "=" * 70)
    print(f"  {text}")
    print("=" * 70 + "\n")


def print_success(text):
    """Print success message."""
    print(f"[PASS] {text}")


def print_error(text):
    """Print error message."""
    print(f"[FAIL] {text}")


def print_info(text):
    """Print info message."""
    print(f"[INFO] {text}")


def test_python_direct():
    """Test Python module directly."""
    print_header("Test 1: Python Module (Direct Import)")

    try:
        from app.utils.openrouter_chatkit import ChatKitAnalyzer

        print_info("Creating ChatKitAnalyzer instance...")
        analyzer = ChatKitAnalyzer()
        print_success("ChatKitAnalyzer initialized")

        print_info("Analyzing test session...")
        analysis = analyzer.analyze_session(TEST_SESSION)

        print_success("Analysis completed!")
        print("\n--- Analysis Output ---")
        print(analysis[:500] + "..." if len(analysis) > 500 else analysis)
        print("-" * 70)

        return True

    except ImportError as e:
        print_error(f"Import failed: {e}")
        print_info("Make sure you're running from RAG-backend directory")
        return False
    except Exception as e:
        print_error(f"Analysis failed: {e}")
        return False


def test_api_health(base_url):
    """Test health endpoint."""
    print_header("Test 2: Health Check Endpoint")

    try:
        print_info(f"Checking {base_url}/analytics/chatkit/health...")
        response = requests.get(f"{base_url}/analytics/chatkit/health", timeout=10)

        if response.status_code == 200:
            data = response.json()
            print_success(f"Health check passed: {data['status']}")
            print(f"   Service: {data.get('service')}")
            print(f"   Timestamp: {data.get('timestamp')}")
            return True
        else:
            print_error(f"Health check failed: {response.status_code}")
            print(response.text)
            return False

    except requests.exceptions.ConnectionError:
        print_error("Could not connect to backend")
        print_info("Make sure backend is running: uvicorn app.main:app --reload")
        return False
    except Exception as e:
        print_error(f"Health check error: {e}")
        return False


def test_api_single_analysis(base_url):
    """Test single session analysis endpoint."""
    print_header("Test 3: Single Session Analysis API")

    try:
        print_info(f"Analyzing session via POST {base_url}/analytics/chatkit/analyze...")

        payload = {"session": TEST_SESSION}
        response = requests.post(
            f"{base_url}/analytics/chatkit/analyze",
            json=payload,
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()
            print_success("Analysis completed via API!")
            print(f"\n   Session ID: {data['session_id']}")
            print(f"   Analyzed at: {data['analyzed_at']}")
            print(f"   Metadata: {json.dumps(data.get('metadata', {}), indent=2)}")
            print("\n--- API Analysis Output ---")
            analysis = data['analysis']
            print(analysis[:500] + "..." if len(analysis) > 500 else analysis)
            print("-" * 70)
            return True
        else:
            print_error(f"API request failed: {response.status_code}")
            print(response.text)
            return False

    except requests.exceptions.Timeout:
        print_error("Request timed out (>30s)")
        print_info("Analysis takes time. Try increasing timeout or check backend logs.")
        return False
    except Exception as e:
        print_error(f"API analysis error: {e}")
        return False


def test_api_batch_analysis(base_url):
    """Test batch analysis endpoint."""
    print_header("Test 4: Batch Analysis API")

    try:
        print_info(f"Analyzing {len(TEST_SESSIONS_BATCH)} sessions via POST {base_url}/analytics/chatkit/batch...")

        payload = {"sessions": TEST_SESSIONS_BATCH}
        response = requests.post(
            f"{base_url}/analytics/chatkit/batch",
            json=payload,
            timeout=60
        )

        if response.status_code == 200:
            data = response.json()
            print_success("Batch analysis completed!")
            print(f"\n   Total sessions: {data['total_sessions']}")
            print(f"   Analyzed at: {data['analyzed_at']}")

            print("\n--- Individual Analyses ---")
            for session_id, analysis in data['analyses'].items():
                print(f"\n{session_id}:")
                print(analysis[:200] + "..." if len(analysis) > 200 else analysis)

            if data.get('summary'):
                print("\n--- Summary Report ---")
                summary = data['summary']
                print(summary[:300] + "..." if len(summary) > 300 else summary)

            print("-" * 70)
            return True
        else:
            print_error(f"Batch API request failed: {response.status_code}")
            print(response.text)
            return False

    except requests.exceptions.Timeout:
        print_error("Batch request timed out (>60s)")
        return False
    except Exception as e:
        print_error(f"Batch analysis error: {e}")
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 70)
    print("   ChatKit Analytics Test Suite")
    print("=" * 70)

    # Configuration
    BASE_URL = "http://localhost:8000"
    print_info(f"Testing against: {BASE_URL}")
    print_info("Ensure backend is running: uvicorn app.main:app --reload\n")

    # Run tests
    results = []

    # Test 1: Python direct
    results.append(("Python Module", test_python_direct()))

    # Test 2: API health
    results.append(("API Health", test_api_health(BASE_URL)))

    # Test 3: Single analysis
    results.append(("Single Analysis", test_api_single_analysis(BASE_URL)))

    # Test 4: Batch analysis
    results.append(("Batch Analysis", test_api_batch_analysis(BASE_URL)))

    # Summary
    print_header("Test Summary")

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"{status} - {name}")

    print(f"\n[SUMMARY] Overall: {passed}/{total} tests passed")

    if passed == total:
        print("\n[SUCCESS] All tests passed! ChatKit Analytics is ready to use.")
        return 0
    else:
        print("\n[WARNING] Some tests failed. Check errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
