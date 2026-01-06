"""
Application configuration using Pydantic Settings.

Loads all environment variables from .env file and provides typed access.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Primary LLM Provider Configuration
    # Defaults to Anthropic Claude Sonnet 4.5 (recommended for production RAG)
    llm_provider: str = "anthropic"
    llm_api_key: str
    llm_base_url: str = "https://api.anthropic.com/v1"
    llm_model: str = "claude-sonnet-4-5-20250929"

    # Fallback LLM Providers (optional - for high availability)
    # Only used if API key is provided
    llm_provider_fallback_1: str = "openai"
    llm_api_key_fallback_1: str | None = None
    llm_base_url_fallback_1: str = "https://api.openai.com/v1"
    llm_model_fallback_1: str = "gpt-4o-mini"

    llm_provider_fallback_2: str = "gemini"
    llm_api_key_fallback_2: str | None = None
    llm_base_url_fallback_2: str = "https://generativelanguage.googleapis.com/v1beta/openai/"
    llm_model_fallback_2: str = "gemini-2.0-flash-exp"

    # OpenAI (for embeddings - required unless using Cohere)
    openai_api_key: str | None = None

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str

    # Neon Serverless Postgres
    neon_database_url: str

    # Application Configuration
    app_env: str = "development"
    log_level: str = "INFO"

    # Optional: Cohere (alternative to OpenAI for embeddings)
    cohere_api_key: str | None = None

    # Optional: GitHub Token
    github_token: str | None = None

    # CORS Configuration
    cors_origins: str = "*"  # Comma-separated list of allowed origins

    # Feature Flags
    use_multi_agent_system: bool = False  # Enable multi-agent architecture (FR-004 to FR-008)
    use_openai_agents_sdk: bool = False  # Enable OpenAI Agents SDK (Spec 3 compliance)

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",  # Ignore extra environment variables
    )

    def get_cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        if self.cors_origins == "*":
            return ["*"]
        return [origin.strip() for origin in self.cors_origins.split(",") if origin.strip()]


# Global settings instance
settings = Settings()

