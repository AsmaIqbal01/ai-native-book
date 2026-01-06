"""
Enhanced logging utilities for LLM provider operations.

This module provides enhanced logging functionality with provider details,
trace IDs, and structured logging for better observability and debugging.
"""

import logging
import time
import random
from typing import Optional, Dict, Any
from contextvars import ContextVar

# Context variable to track current operation details
current_trace_id: ContextVar[Optional[str]] = ContextVar('current_trace_id', default=None)
current_provider: ContextVar[Optional[str]] = ContextVar('current_provider', default=None)
current_operation: ContextVar[Optional[str]] = ContextVar('current_operation', default=None)


class ProviderLogger:
    """
    Enhanced logger with provider-specific context and trace IDs.
    """

    def __init__(self, name: str):
        self.logger = logging.getLogger(name)
        self.handler = None

    def _generate_trace_id(self, operation: str) -> str:
        """Generate a unique trace ID for the operation."""
        return f"{operation}_{int(time.time() * 1000000)}_{random.randint(1000, 9999)}"

    def _get_context_dict(self, trace_id: Optional[str] = None, provider: Optional[str] = None) -> Dict[str, Any]:
        """Get context dictionary with current and provided context."""
        context = {
            'trace_id': trace_id or current_trace_id.get(),
            'provider': provider or current_provider.get(),
            'operation': current_operation.get()
        }
        # Remove None values
        return {k: v for k, v in context.items() if v is not None}

    def debug(self, message: str, trace_id: Optional[str] = None, provider: Optional[str] = None, **kwargs):
        """Log debug message with context."""
        context = self._get_context_dict(trace_id, provider)
        context.update(kwargs)
        self.logger.debug(message, extra=context)

    def info(self, message: str, trace_id: Optional[str] = None, provider: Optional[str] = None, **kwargs):
        """Log info message with context."""
        context = self._get_context_dict(trace_id, provider)
        context.update(kwargs)
        self.logger.info(message, extra=context)

    def warning(self, message: str, trace_id: Optional[str] = None, provider: Optional[str] = None, **kwargs):
        """Log warning message with context."""
        context = self._get_context_dict(trace_id, provider)
        context.update(kwargs)
        self.logger.warning(message, extra=context)

    def error(self, message: str, trace_id: Optional[str] = None, provider: Optional[str] = None, **kwargs):
        """Log error message with context."""
        context = self._get_context_dict(trace_id, provider)
        context.update(kwargs)
        self.logger.error(message, extra=context)

    def critical(self, message: str, trace_id: Optional[str] = None, provider: Optional[str] = None, **kwargs):
        """Log critical message with context."""
        context = self._get_context_dict(trace_id, provider)
        context.update(kwargs)
        self.logger.critical(message, extra=context)

    def log_with_context(self, level: int, message: str, operation: str,
                        provider: Optional[str] = None, **kwargs):
        """
        Log message with automatic trace ID generation and context setup.

        Args:
            level: Logging level
            message: Message to log
            operation: Operation name for trace ID
            provider: Provider name to include in context
            **kwargs: Additional context data
        """
        trace_id = self._generate_trace_id(operation)

        # Set context variables
        token_trace = current_trace_id.set(trace_id)
        token_provider = current_provider.set(provider) if provider else None
        token_operation = current_operation.set(operation)

        try:
            context = self._get_context_dict(trace_id, provider)
            context.update(kwargs)
            self.logger.log(level, message, extra=context)
            return trace_id
        finally:
            # Reset context variables
            current_trace_id.reset(token_trace)
            if token_provider:
                current_provider.reset(token_provider)
            current_operation.reset(token_operation)


# Global provider logger instance
provider_logger = ProviderLogger(__name__)


def get_provider_logger(name: str) -> ProviderLogger:
    """
    Get a provider logger instance with the given name.

    Args:
        name: Name for the logger

    Returns:
        ProviderLogger instance
    """
    return ProviderLogger(name)


def setup_structured_logging():
    """
    Setup structured logging with JSON formatter if needed.
    """
    import json
    import sys

    class JSONFormatter(logging.Formatter):
        def format(self, record):
            log_entry = {
                'timestamp': self.formatTime(record),
                'level': record.levelname,
                'message': record.getMessage(),
                'module': record.module,
                'function': record.funcName,
                'line': record.lineno
            }

            # Add extra fields if present
            for key, value in record.__dict__.items():
                if key not in ['name', 'msg', 'args', 'levelname', 'levelno', 'pathname',
                              'filename', 'module', 'lineno', 'funcName', 'created',
                              'msecs', 'relativeCreated', 'thread', 'threadName',
                              'processName', 'process', 'getMessage', 'exc_info',
                              'exc_text', 'stack_info']:
                    log_entry[key] = value

            return json.dumps(log_entry)

    # Configure root logger with JSON formatter
    root_logger = logging.getLogger()
    if not root_logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        formatter = JSONFormatter()
        handler.setFormatter(formatter)
        root_logger.addHandler(handler)
        root_logger.setLevel(logging.INFO)