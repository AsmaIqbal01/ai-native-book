"""
Retry decorator with exponential backoff and jitter.

Automatically retries failed operations with increasing delays and random jitter.
"""

import asyncio
import functools
import random
import time
from typing import Callable, Type, Tuple, Union, Any
import logging

# Import the rate limit configuration

from app.config import rate_limit_config


logger = logging.getLogger(__name__)


def retry_with_exponential_backoff(
    max_retries: int = None,
    initial_delay: float = None,
    exponential_base: float = 2.0,
    jitter: bool = True,
    max_delay: float = None,
    exceptions: Tuple[Type[Exception], ...] = (Exception,),
    detect_429: bool = True,
    detect_auth_errors: bool = True,  # NEW: Detect authentication errors that should not be retried
    detect_network_errors: bool = True,  # NEW: Detect network-related errors for specific handling,
):
    """
    Retry decorator with exponential backoff and optional jitter.

    Args:
        max_retries: Maximum number of retry attempts (default: from config).
        initial_delay: Initial delay in seconds (default: from config).
        exponential_base: Base for exponential backoff (default: 2.0).
        jitter: Whether to add random jitter to delays (default: True).
        max_delay: Maximum delay cap in seconds (default: from config).
        exceptions: Tuple of exceptions to catch and retry (default: all).
        detect_429: Whether to specifically detect and handle HTTP 429 errors (default: True).
        detect_auth_errors: Whether to detect authentication errors that should fail fast (default: True).
        detect_network_errors: Whether to detect network-related errors for specific handling (default: True).

    Returns:
        Decorated function with retry logic.

    Example:
        @retry_with_exponential_backoff(max_retries=3, initial_delay=1.0)
        async def call_api():
            # API call that might fail
            pass
    """
    # Use config values if not provided
    if max_retries is None:
        max_retries = rate_limit_config.get_max_retries()
    if initial_delay is None:
        initial_delay = rate_limit_config.get_initial_retry_delay()
    if max_delay is None:
        max_delay = rate_limit_config.get_max_retry_delay()

    def decorator(func: Callable):
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            delay = initial_delay
            # Generate a unique trace ID for this function call
            trace_id = f"{func.__name__}_{int(time.time() * 1000000)}_{random.randint(1000, 9999)}"

            for attempt in range(max_retries + 1):
                try:
                    start_time = time.time()
                    result = await func(*args, **kwargs)
                    duration = time.time() - start_time
                    logger.debug(
                        f"Function {func.__name__} succeeded on attempt {attempt + 1} "
                        f"(trace_id: {trace_id}, duration: {duration:.2f}s)"
                    )
                    return result
                except exceptions as e:
                    error_str = str(e).lower()

                    # Check for authentication errors that should fail fast (401)
                    is_auth_error = detect_auth_errors and (
                        "401" in error_str or
                        "invalid_api_key" in error_str or
                        "invalid api key" in error_str or
                        "authentication" in error_str or
                        "unauthorized" in error_str or
                        "token expired" in error_str or
                        "expired token" in error_str or
                        ("api key" in error_str and ("invalid" in error_str or "expired" in error_str))
                    )

                    # Check for network errors (500, fetch failed, network/transport failures)
                    is_network_error = detect_network_errors and (
                        "500" in error_str or
                        "fetch failed" in error_str or
                        "network error" in error_str or
                        "connection error" in error_str or
                        "connection timeout" in error_str or
                        "transport error" in error_str or
                        "server disconnected" in error_str or
                        "ssl error" in error_str
                    )

                    is_429_error = detect_429 and (
                        "429" in error_str or
                        "rate limit" in error_str or
                        "too many requests" in error_str or
                        "please wait and try again later" in error_str or
                        "rate_limit_exceeded" in error_str
                    )

                    # If it's an authentication error, fail immediately without retrying
                    if is_auth_error:
                        logger.error(
                            f"Authentication error in function {func.__name__} (trace_id: {trace_id}): {str(e)}. "
                            f"Authentication errors fail fast - not retrying."
                        )
                        raise

                    if attempt == max_retries:
                        # Last attempt failed, raise the exception
                        logger.error(
                            f"Function {func.__name__} failed after {max_retries} retries "
                            f"(trace_id: {trace_id}): {str(e)}"
                        )
                        raise

                    # Log retry attempt with trace ID
                    error_type = "network" if is_network_error else "rate_limit" if is_429_error else "other"
                    logger.warning(
                        f"Function {func.__name__} failed (attempt {attempt + 1}/{max_retries}, "
                        f"trace_id: {trace_id}, error_type: {error_type}): {str(e)}. "
                        f"Retrying in {delay:.2f}s..."
                    )

                    # Wait before retrying
                    await asyncio.sleep(delay)

                    # Calculate next delay with exponential backoff
                    delay *= exponential_base

                    # Different delay strategies based on error type
                    if is_429_error:
                        # For 429 errors, use a more conservative approach with longer delays
                        delay = min(delay * 1.5, max_delay)  # Additional multiplier for 429
                    elif is_network_error:
                        # For network errors, use moderate backoff
                        delay = min(delay * 1.2, max_delay)  # Moderate multiplier for network errors
                    else:
                        # Add jitter to prevent thundering herd for other errors
                        if jitter:
                            delay = min(delay * (0.5 + random.random()), max_delay)  # Random factor between 0.5-1.5
                        else:
                            delay = min(delay, max_delay)

        return wrapper

    return decorator
