"""
Retry decorator with exponential backoff.

Automatically retries failed operations with increasing delays.
"""

import asyncio
import functools
from typing import Callable, Type, Tuple
import logging

logger = logging.getLogger(__name__)


def retry_with_exponential_backoff(
    max_retries: int = 3,
    initial_delay: float = 1.0,
    exponential_base: float = 2.0,
    exceptions: Tuple[Type[Exception], ...] = (Exception,),
):
    """
    Retry decorator with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts (default: 3).
        initial_delay: Initial delay in seconds (default: 1.0).
        exponential_base: Base for exponential backoff (default: 2.0).
        exceptions: Tuple of exceptions to catch and retry (default: all).

    Returns:
        Decorated function with retry logic.

    Example:
        @retry_with_exponential_backoff(max_retries=3, initial_delay=1.0)
        async def call_api():
            # API call that might fail
            pass
    """

    def decorator(func: Callable):
        @functools.wraps(func)
        async def wrapper(*args, **kwargs):
            delay = initial_delay

            for attempt in range(max_retries + 1):
                try:
                    return await func(*args, **kwargs)
                except exceptions as e:
                    if attempt == max_retries:
                        # Last attempt failed, raise the exception
                        logger.error(
                            f"Function {func.__name__} failed after {max_retries} retries: {str(e)}"
                        )
                        raise

                    # Log retry attempt
                    logger.warning(
                        f"Function {func.__name__} failed (attempt {attempt + 1}/{max_retries}): {str(e)}. "
                        f"Retrying in {delay:.2f}s..."
                    )

                    # Wait before retrying
                    await asyncio.sleep(delay)

                    # Increase delay exponentially
                    delay *= exponential_base

        return wrapper

    return decorator
