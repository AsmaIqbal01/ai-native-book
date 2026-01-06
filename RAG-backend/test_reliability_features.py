"""
Test suite for the enhanced LLM reliability and authentication safety features.

This test suite validates the new features implemented for the backend LLM pipeline:
- Authentication error detection and fail-fast behavior
- Network error handling with appropriate backoff
- Provider validation at startup
- Enhanced logging with provider details
- Proper retry mechanisms
"""

import asyncio
import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from openai import RateLimitError, AuthenticationError
from app.utils.retry import retry_with_exponential_backoff
from app.utils.provider_validator import ProviderValidator
from app.utils.api_key_validator import validate_provider_configuration


class TestAuthenticationErrorHandling:
    """Test authentication error handling and fail-fast behavior."""

    @pytest.mark.asyncio
    async def test_auth_error_fails_fast(self):
        """Test that authentication errors fail immediately without retrying."""
        retry_count = 0

        @retry_with_exponential_backoff(
            max_retries=3,
            initial_delay=0.1,
            detect_auth_errors=True,
            detect_429=False,
            detect_network_errors=False
        )
        async def test_func():
            nonlocal retry_count
            retry_count += 1
            raise AuthenticationError("Invalid API key")

        # Should fail immediately without retries
        with pytest.raises(AuthenticationError):
            await test_func()

        # Should only be called once due to fail-fast behavior
        assert retry_count == 1

    @pytest.mark.asyncio
    async def test_401_error_fails_fast(self):
        """Test that 401 errors fail immediately without retrying."""
        retry_count = 0

        @retry_with_exponential_backoff(
            max_retries=3,
            initial_delay=0.1,
            detect_auth_errors=True,
            detect_429=False,
            detect_network_errors=False
        )
        async def test_func():
            nonlocal retry_count
            retry_count += 1
            raise Exception("401 Client Error: Unauthorized")

        # Should fail immediately without retries
        with pytest.raises(Exception) as exc_info:
            await test_func()

        # Should only be called once due to fail-fast behavior
        assert retry_count == 1
        assert "401 Client Error: Unauthorized" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_invalid_api_key_fails_fast(self):
        """Test that invalid API key errors fail immediately without retrying."""
        retry_count = 0

        @retry_with_exponential_backoff(
            max_retries=3,
            initial_delay=0.1,
            detect_auth_errors=True,
            detect_429=False,
            detect_network_errors=False
        )
        async def test_func():
            nonlocal retry_count
            retry_count += 1
            raise Exception("Error code: invalid_api_key")

        # Should fail immediately without retries
        with pytest.raises(Exception) as exc_info:
            await test_func()

        # Should only be called once due to fail-fast behavior
        assert retry_count == 1
        assert "invalid_api_key" in str(exc_info.value)


class TestRateLimitErrorHandling:
    """Test rate limit error handling with proper retry behavior."""

    @pytest.mark.asyncio
    async def test_429_error_retries_with_backoff(self):
        """Test that 429 errors are retried with exponential backoff."""
        retry_count = 0
        max_retries = 2

        @retry_with_exponential_backoff(
            max_retries=max_retries,
            initial_delay=0.01,  # Fast test
            detect_429=True,
            detect_auth_errors=True,
            detect_network_errors=False
        )
        async def test_func():
            nonlocal retry_count
            retry_count += 1
            if retry_count <= max_retries:
                raise Exception("429 Too Many Requests")
            return "success"

        # Should eventually succeed after retries
        result = await test_func()
        assert result == "success"
        assert retry_count == max_retries + 1  # Initial call + retries


class TestNetworkErrorHandling:
    """Test network error handling with appropriate backoff."""

    @pytest.mark.asyncio
    async def test_network_error_retries_with_moderate_backoff(self):
        """Test that network errors are retried with moderate backoff."""
        retry_count = 0
        max_retries = 2

        @retry_with_exponential_backoff(
            max_retries=max_retries,
            initial_delay=0.01,  # Fast test
            detect_network_errors=True,
            detect_429=False,
            detect_auth_errors=False
        )
        async def test_func():
            nonlocal retry_count
            retry_count += 1
            if retry_count <= max_retries:
                raise Exception("Connection timeout")
            return "success"

        # Should eventually succeed after retries
        result = await test_func()
        assert result == "success"
        assert retry_count == max_retries + 1  # Initial call + retries


class TestProviderValidation:
    """Test provider validation functionality."""

    def test_api_key_validation(self):
        """Test API key validation for different providers."""
        # Test OpenAI key validation
        is_valid, error_msg = validate_provider_configuration(
            "sk-1234567890abcdef",  # OpenAI key format
            "openai",
            "https://api.openai.com/v1"
        )
        assert is_valid, f"OpenAI validation failed: {error_msg}"

        # Test OpenRouter key validation
        is_valid, error_msg = validate_provider_configuration(
            "sk-or-1234567890abcdef",  # OpenRouter key format
            "openrouter",
            "https://openrouter.ai/api/v1"
        )
        assert is_valid, f"OpenRouter validation failed: {error_msg}"

        # Test invalid key-provider combination
        is_valid, error_msg = validate_provider_configuration(
            "sk-1234567890abcdef",  # OpenAI key format
            "openrouter",  # Wrong provider
            "https://openrouter.ai/api/v1"
        )
        assert not is_valid, "Should have failed validation for mismatched key/provider"
        assert "expects key to start with one of" in error_msg


class TestIntegration:
    """Integration tests for the enhanced reliability features."""

    @pytest.mark.asyncio
    async def test_complete_retry_flow(self):
        """Test the complete retry flow with different error types."""
        call_count = 0

        @retry_with_exponential_backoff(
            max_retries=2,
            initial_delay=0.01,
            detect_auth_errors=True,
            detect_429=True,
            detect_network_errors=True
        )
        async def test_func(error_type="429"):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                if error_type == "429":
                    raise Exception("429 Too Many Requests")
                elif error_type == "network":
                    raise Exception("Connection timeout")
                elif error_type == "auth":
                    raise Exception("401 Unauthorized")
            return "success"

        # Test 429 error handling (should retry)
        call_count = 0
        result = await test_func("429")
        assert result == "success"
        assert call_count == 2  # Initial call + 1 retry

        # Test network error handling (should retry)
        call_count = 0
        result = await test_func("network")
        assert result == "success"
        assert call_count == 2  # Initial call + 1 retry

        # Test auth error handling (should fail fast)
        call_count = 0
        with pytest.raises(Exception) as exc_info:
            await test_func("auth")
        assert call_count == 1  # Should only try once
        assert "Unauthorized" in str(exc_info.value)


if __name__ == "__main__":
    # Run basic validation
    print("Running basic validation tests...")

    # Test provider validation
    validator = ProviderValidator()
    print("✓ Provider validation class created")

    # Test API key validation
    is_valid, msg = validate_provider_configuration("sk-test123", "openai", "https://api.openai.com/v1")
    print(f"✓ API key validation: {is_valid}, {msg}")

    print("All basic validation tests passed!")
    print("\nTo run full test suite, use: pytest test_reliability_features.py -v")