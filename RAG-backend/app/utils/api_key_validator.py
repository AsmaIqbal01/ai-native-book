"""
API Key Validation Utilities for LLM Provider Matching.

This module provides validation functions to ensure that API keys are matched
to the correct LLM provider endpoints, preventing 401 errors when using
mismatched keys (e.g., sending xAI keys to OpenAI endpoints).
"""

import re
from typing import Optional


def validate_api_key_provider_match(api_key: str, provider: str) -> tuple[bool, Optional[str]]:
    """
    Validate that the API key format matches the expected provider.

    Args:
        api_key: The API key to validate
        provider: The provider name (e.g., 'openai', 'openrouter', 'xai', 'gemini')

    Returns:
        tuple[bool, Optional[str]]: (is_valid, error_message)
    """
    if not api_key or not provider:
        return False, "API key and provider must be provided"

    # Normalize provider name to lowercase
    provider = provider.lower().strip()

    # Extract key prefix (first few characters before the actual key)
    key_prefix = api_key.lower()[:6]  # First 6 characters should be enough to identify the prefix

    # Define expected prefixes for each provider
    expected_prefixes = {
        'openai': ['sk-', 'sess-'],  # OpenAI keys typically start with 'sk-'
        'openrouter': ['sk-or-'],    # OpenRouter keys start with 'sk-or-'
        'xai': ['xai-'],            # xAI keys start with 'xai-'
        'gemini': [''],             # Gemini uses different format, often starts with 'AI' or similar
        'anthropic': ['sk-ant-'],   # Anthropic keys start with 'sk-ant-'
        'cohere': ['']              # Cohere keys vary, often start with 'Cohere-'
    }

    # Special case for Gemini which has different key format
    if provider == 'gemini':
        # Gemini API keys often start with 'AI' followed by letters and numbers
        if re.match(r'^AI[a-zA-Z0-9]+', api_key):
            return True, None
        elif re.match(r'^\w{30,}', api_key):  # Long alphanumeric keys are also valid
            return True, None
        else:
            return False, f"Gemini API key expected to start with 'AI' prefix, got key starting with '{key_prefix[:2] if len(key_prefix) >= 2 else key_prefix}'"

    # Check if provider is in our expected prefixes
    if provider in expected_prefixes:
        expected = expected_prefixes[provider]
        for prefix in expected:
            if api_key.lower().startswith(prefix.lower()):
                return True, None
        # If we get here, no expected prefix matched
        return False, f"Provider '{provider}' expects key to start with one of {expected}, but got key starting with '{key_prefix}'"

    # If provider is not in our known list, we can't validate it
    return True, None  # Allow unknown providers but warn about it


def get_provider_from_api_key(api_key: str) -> Optional[str]:
    """
    Attempt to determine the provider from the API key prefix.

    Args:
        api_key: The API key to analyze

    Returns:
        str or None: The likely provider name, or None if unknown
    """
    if not api_key:
        return None

    key_lower = api_key.lower()

    if key_lower.startswith('sk-or-'):
        return 'openrouter'
    elif key_lower.startswith('sk-') and not key_lower.startswith('sk-or-') and not key_lower.startswith('sk-ant-'):
        return 'openai'
    elif key_lower.startswith('xai-'):
        return 'xai'
    elif key_lower.startswith('sk-ant-'):
        return 'anthropic'
    elif re.match(r'^AI[a-zA-Z0-9]+', api_key):
        return 'gemini'
    else:
        return None  # Unknown provider based on key prefix


def validate_provider_configuration(api_key: str, provider: str, base_url: str) -> tuple[bool, Optional[str]]:
    """
    Validate the complete provider configuration including API key, provider, and base URL.

    Args:
        api_key: The API key
        provider: The provider name
        base_url: The base URL for the API

    Returns:
        tuple[bool, Optional[str]]: (is_valid, error_message)
    """
    # First validate key-provider match
    is_valid, error_msg = validate_api_key_provider_match(api_key, provider)
    if not is_valid:
        return False, error_msg

    # Additional validation for base URL vs provider
    provider_url_patterns = {
        'openai': r'api\.openai\.com',
        'openrouter': r'openrouter\.ai',
        'xai': r'api\.x\.ai',
        'gemini': r'generativelanguage\.googleapis\.com|google\.com',
        'anthropic': r'api\.anthropic\.com',
        'claude': r'api\.anthropic\.com'  # Alias for anthropic
    }

    if provider.lower() in provider_url_patterns:
        pattern = provider_url_patterns[provider.lower()]
        if not re.search(pattern, base_url.lower()):
            return False, f"Provider '{provider}' expects base URL to contain '{pattern}', but got '{base_url}'"

    return True, None