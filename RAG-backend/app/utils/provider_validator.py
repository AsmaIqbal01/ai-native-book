"""
Provider validation utilities for LLM provider configuration validation.

This module provides functions to validate provider configurations at startup
and ensure that API keys, base URLs, and models are properly configured
to prevent runtime failures.
"""

import asyncio
import logging
from typing import List, Tuple
from openai import AsyncOpenAI
from app.config import settings
from app.utils.api_key_validator import validate_provider_configuration

logger = logging.getLogger(__name__)


class ProviderValidator:
    """
    Validates LLM provider configurations at startup to ensure they are properly set up.
    """

    @staticmethod
    def validate_primary_provider() -> Tuple[bool, str]:
        """
        Validate the primary LLM provider configuration.

        Returns:
            Tuple[bool, str]: (is_valid, error_message)
        """
        # Check if required settings are present
        if not settings.llm_api_key:
            return False, "LLM_API_KEY is not configured in environment"

        if not settings.llm_provider:
            return False, "LLM_PROVIDER is not configured in environment"

        if not settings.llm_base_url:
            return False, "LLM_BASE_URL is not configured in environment"

        if not settings.llm_model:
            return False, "LLM_MODEL is not configured in environment"

        # Validate provider configuration
        is_valid, error_msg = validate_provider_configuration(
            settings.llm_api_key,
            settings.llm_provider,
            settings.llm_base_url
        )

        if not is_valid:
            return False, f"Primary provider validation failed: {error_msg}"

        return True, "Primary provider configuration is valid"

    @staticmethod
    def validate_fallback_providers() -> Tuple[bool, List[str]]:
        """
        Validate fallback provider configurations.

        Returns:
            Tuple[bool, List[str]]: (all_valid, list_of_errors)
        """
        errors = []

        # Validate fallback 1
        if settings.llm_api_key_fallback_1:
            is_valid, error_msg = validate_provider_configuration(
                settings.llm_api_key_fallback_1,
                settings.llm_provider_fallback_1,
                settings.llm_base_url_fallback_1
            )
            if not is_valid:
                errors.append(f"Fallback 1: {error_msg}")

        # Validate fallback 2
        if settings.llm_api_key_fallback_2:
            is_valid, error_msg = validate_provider_configuration(
                settings.llm_api_key_fallback_2,
                settings.llm_provider_fallback_2,
                settings.llm_base_url_fallback_2
            )
            if not is_valid:
                errors.append(f"Fallback 2: {error_msg}")

        return len(errors) == 0, errors

    @staticmethod
    async def test_provider_connection(provider_config: dict) -> Tuple[bool, str]:
        """
        Test connection to a provider by making a minimal API call.

        Args:
            provider_config: Dictionary with 'api_key', 'base_url', 'model' keys

        Returns:
            Tuple[bool, str]: (is_connected, message)
        """
        try:
            client = AsyncOpenAI(
                api_key=provider_config['api_key'],
                base_url=provider_config['base_url'],
            )

            # Make a minimal test call
            response = await client.chat.completions.create(
                model=provider_config['model'],
                messages=[{"role": "user", "content": "test"}],
                max_tokens=1,
                temperature=0
            )

            return True, "Provider connection successful"

        except Exception as e:
            error_str = str(e).lower()
            provider_name = provider_config.get('provider', 'provider')
            api_key = provider_config['api_key']
            base_url = provider_config['base_url']
            model = provider_config['model']

            # Check for authentication errors specifically
            if "401" in error_str or "invalid_api_key" in error_str or \
               "authentication" in error_str or "unauthorized" in error_str:
                # Determine the actual provider from the API key
                from app.utils.api_key_validator import get_provider_from_api_key
                detected_provider = get_provider_from_api_key(api_key)

                error_msg = f"❌ Authentication failed (401 Unauthorized)\n"
                error_msg += f"   Provider: {provider_name}\n"
                error_msg += f"   Base URL: {base_url}\n"
                error_msg += f"   Model: {model}\n"
                error_msg += f"   API Key starts with: {api_key[:10]}...\n"

                if detected_provider and detected_provider != provider_name:
                    error_msg += f"\n   ⚠️  MISMATCH DETECTED: Your API key appears to be for '{detected_provider}', "
                    error_msg += f"but you're using base URL for '{provider_name}'.\n"
                    error_msg += f"   💡 Fix: Update LLM_BASE_URL to match your provider:\n"

                    # Provide specific URL recommendations
                    url_recommendations = {
                        'anthropic': 'https://api.anthropic.com/v1',
                        'openai': 'https://api.openai.com/v1',
                        'openrouter': 'https://openrouter.ai/api/v1',
                        'xai': 'https://api.x.ai/v1'
                    }
                    if detected_provider in url_recommendations:
                        error_msg += f"      LLM_BASE_URL={url_recommendations[detected_provider]}\n"
                else:
                    error_msg += f"\n   💡 Possible causes:\n"
                    error_msg += f"      1. Invalid or expired API key\n"
                    error_msg += f"      2. API key doesn't have permission for this model\n"
                    error_msg += f"      3. API key is for a different environment (test vs production)\n"

                return False, error_msg

            elif "404" in error_str or "not found" in error_str or "not_found" in error_str:
                error_msg = f"❌ Resource not found (404)\n"
                error_msg += f"   Provider: {provider_name}\n"
                error_msg += f"   Base URL: {base_url}\n"
                error_msg += f"   Model: {model}\n"
                error_msg += f"\n   💡 Possible causes:\n"
                error_msg += f"      1. Model '{model}' doesn't exist on this provider\n"
                error_msg += f"      2. Incorrect base URL (missing /v1 path?)\n"
                error_msg += f"      3. Provider endpoint has changed\n"
                error_msg += f"\n   💡 Suggested fixes:\n"
                error_msg += f"      - Verify model name is correct for {provider_name}\n"
                error_msg += f"      - Check provider documentation for available models\n"
                error_msg += f"      - Ensure base URL includes correct API version path\n"

                return False, error_msg

            elif "429" in error_str or "rate limit" in error_str:
                error_msg = f"⚠️  Rate limited (429 Too Many Requests)\n"
                error_msg += f"   Provider: {provider_name}\n"
                error_msg += f"\n   💡 The application will automatically retry with exponential backoff.\n"
                error_msg += f"   💡 If this persists, consider:\n"
                error_msg += f"      - Reducing LLM_MAX_CONCURRENT (currently set in .env)\n"
                error_msg += f"      - Increasing RETRY_INITIAL_DELAY\n"
                error_msg += f"      - Upgrading your API plan with the provider\n"

                return False, error_msg

            else:
                error_msg = f"❌ Connection failed\n"
                error_msg += f"   Provider: {provider_name}\n"
                error_msg += f"   Error: {str(e)}\n"
                error_msg += f"\n   💡 Check:\n"
                error_msg += f"      - Network connectivity\n"
                error_msg += f"      - Provider service status\n"
                error_msg += f"      - Firewall/proxy settings\n"

                return False, error_msg

    @classmethod
    async def validate_all_providers(cls) -> Tuple[bool, dict]:
        """
        Validate all provider configurations including connection tests.

        Returns:
            Tuple[bool, dict]: (all_valid, validation_results)
        """
        results = {
            'primary': {'valid': False, 'message': ''},
            'fallback_1': {'valid': False, 'message': ''},
            'fallback_2': {'valid': False, 'message': ''},
            'errors': []
        }

        # Validate primary provider configuration
        primary_valid, primary_msg = cls.validate_primary_provider()
        results['primary']['valid'] = primary_valid
        results['primary']['message'] = primary_msg

        if not primary_valid:
            results['errors'].append(f"Primary provider: {primary_msg}")

        # Validate fallback providers configuration
        fallback_valid, fallback_errors = cls.validate_fallback_providers()
        if not fallback_valid:
            results['errors'].extend(fallback_errors)

        # Test actual connections (optional, can be skipped in some environments)
        try:
            primary_connection = await cls.test_provider_connection({
                'provider': settings.llm_provider,
                'api_key': settings.llm_api_key,
                'base_url': settings.llm_base_url,
                'model': settings.llm_model
            })
            results['primary']['connection_test'] = {
                'success': primary_connection[0],
                'message': primary_connection[1]
            }

            if not primary_connection[0]:
                results['errors'].append(f"Primary connection test: {primary_connection[1]}")
        except Exception as e:
            logger.warning(f"Could not perform connection test for primary provider: {e}")
            results['primary']['connection_test'] = {
                'success': False,
                'message': f"Connection test failed: {str(e)}"
            }

        return len(results['errors']) == 0, results


async def validate_providers_at_startup():
    """
    Convenience function to validate all providers at application startup.
    """
    logger.info("Validating LLM provider configurations at startup...")

    is_valid, results = await ProviderValidator.validate_all_providers()

    if is_valid:
        logger.info("All provider configurations are valid")
    else:
        logger.warning(f"Provider validation completed with issues: {results['errors']}")
        for error in results['errors']:
            logger.error(error)

    return is_valid, results