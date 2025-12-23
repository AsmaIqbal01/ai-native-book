"""
Structured JSON logging utility.

Provides consistent logging format across all endpoints.
"""

import logging
import json
from datetime import datetime
from typing import Any, Dict, Optional


class StructuredJSONLogger:
    """Structured logger that outputs JSON for easy parsing."""

    def __init__(self, name: str, level: str = "INFO"):
        """
        Initialize structured logger.

        Args:
            name: Logger name (typically __name__).
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))

        # Remove existing handlers to avoid duplicates
        self.logger.handlers = []

        # Create console handler with JSON formatter
        handler = logging.StreamHandler()
        handler.setFormatter(JSONFormatter())
        self.logger.addHandler(handler)

    def _log(self, level: str, message: str, context: Optional[Dict[str, Any]] = None):
        """
        Internal log method.

        Args:
            level: Log level.
            message: Log message.
            context: Additional context as dictionary.
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": level,
            "message": message,
            "context": context or {},
        }
        getattr(self.logger, level.lower())(json.dumps(log_data))

    def debug(self, message: str, context: Optional[Dict[str, Any]] = None):
        """Log debug message."""
        self._log("DEBUG", message, context)

    def info(self, message: str, context: Optional[Dict[str, Any]] = None):
        """Log info message."""
        self._log("INFO", message, context)

    def warning(self, message: str, context: Optional[Dict[str, Any]] = None):
        """Log warning message."""
        self._log("WARNING", message, context)

    def error(self, message: str, context: Optional[Dict[str, Any]] = None):
        """Log error message."""
        self._log("ERROR", message, context)

    def critical(self, message: str, context: Optional[Dict[str, Any]] = None):
        """Log critical message."""
        self._log("CRITICAL", message, context)


class JSONFormatter(logging.Formatter):
    """Custom formatter for JSON output."""

    def format(self, record):
        """Format log record as JSON string."""
        # The record.msg is already JSON from StructuredJSONLogger
        return record.msg


def get_logger(name: str, level: str = "INFO") -> StructuredJSONLogger:
    """
    Get or create a structured logger.

    Args:
        name: Logger name.
        level: Log level.

    Returns:
        StructuredJSONLogger instance.
    """
    return StructuredJSONLogger(name, level)
