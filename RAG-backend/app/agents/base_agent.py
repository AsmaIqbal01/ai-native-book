"""
Base Agent Abstract Class

Defines the contract that all RAG agents must implement.
Enforces single responsibility principle and provides shared error handling.
"""

from abc import ABC, abstractmethod
from typing import Generic, TypeVar, Any
from pydantic import BaseModel
import logging

# Generic types for input and output
TInput = TypeVar('TInput', bound=BaseModel)
TOutput = TypeVar('TOutput', bound=BaseModel)

logger = logging.getLogger(__name__)


class ValidationResult(BaseModel):
    """Result of input validation."""
    is_valid: bool
    error_message: str | None = None
    normalized_input: Any | None = None


class BaseAgent(ABC, Generic[TInput, TOutput]):
    """
    Abstract base class for all RAG agents.

    All agents must implement:
    - validate_input(): Validate and normalize input data
    - execute(): Core agent logic
    - handle_error(): Error recovery logic

    This ensures consistent interfaces across all agents and enables
    clean orchestration in RAGChatAgent.
    """

    def __init__(self):
        """Initialize agent with logger."""
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def validate_input(self, input_data: TInput) -> ValidationResult:
        """
        Validate input before execution.

        Args:
            input_data: Input data to validate

        Returns:
            ValidationResult with is_valid flag and optional error message
        """
        pass

    @abstractmethod
    async def execute(self, input_data: TInput) -> TOutput:
        """
        Execute agent's core logic.

        Args:
            input_data: Validated input data

        Returns:
            Agent-specific output data

        Raises:
            Exception: If execution fails (handled by ErrorRecoveryAgent)
        """
        pass

    async def run(self, input_data: TInput) -> TOutput:
        """
        Run full agent lifecycle: validate → execute.

        This method coordinates validation and execution,
        ensuring inputs are validated before processing.

        Args:
            input_data: Input data to process

        Returns:
            Agent output

        Raises:
            ValueError: If validation fails
            Exception: If execution fails
        """
        # Validate input
        validation_result = await self.validate_input(input_data)

        if not validation_result.is_valid:
            self.logger.error(
                f"{self.__class__.__name__} validation failed",
                extra={"error": validation_result.error_message}
            )
            raise ValueError(validation_result.error_message)

        # Execute with validated input
        self.logger.info(f"{self.__class__.__name__} executing")
        result = await self.execute(input_data)
        self.logger.info(f"{self.__class__.__name__} completed successfully")

        return result
