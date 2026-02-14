from pydantic_settings import BaseSettings
from pydantic import field_validator
from typing import Optional
from src.utils.exceptions import ConfigurationError


class Settings(BaseSettings):
    # Server Configuration
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    RELOAD: bool = True

    # API Keys
    OPENAI_API_KEY: Optional[str] = None
    GEMINI_API_KEY: Optional[str] = None  # Keep for compatibility but prefer OpenAI
    BASE_URL: Optional[str] = None  # For using OpenAI SDK with Gemini
    COHERE_API_KEY: Optional[str] = None
    QDRANT_URL: Optional[str] = None
    QDRANT_API_KEY: Optional[str] = None

    # Model Configuration
    MODEL_NAME: str = "gemini-3-flash-preview"  # Use Gemini model when using Gemini API
    TEMPERATURE: float = 0.7
    MAX_TOKENS: int = 2048
    RETRIEVAL_THRESHOLD: float = 0.3
    TIMEOUT_SECONDS: int = 30
    ENABLE_CITATIONS: bool = True

    # Additional Configuration
    DATABASE_URL: Optional[str] = "sqlite:///./coursebook.db"
    APP_ENV: str = "development"
    LOG_LEVEL: str = "info"

    @field_validator('OPENAI_API_KEY', 'GEMINI_API_KEY', 'COHERE_API_KEY', 'QDRANT_API_KEY', 'QDRANT_URL', 'BASE_URL', mode='before')
    @classmethod
    def strip_whitespace(cls, v):
        """Strip whitespace from API keys and URLs to prevent header errors"""
        if v is not None and isinstance(v, str):
            return v.strip()
        return v

    class Config:
        env_file = ".env"
        env_file_encoding = 'utf-8'
        case_sensitive = True
        # Allow .env file to be optional (for Hugging Face deployment)
        # Environment variables will always take precedence over .env
        extra = 'ignore'

    def get_agent_config_dict(self) -> dict:
        """
        Get the configuration as a dictionary suitable for AgentConfiguration model.
        """
        return {
            "api_key": self.OPENAI_API_KEY or self.GEMINI_API_KEY,
            "model_name": self.MODEL_NAME,
            "temperature": self.TEMPERATURE,
            "max_tokens": self.MAX_TOKENS,
            "retrieval_threshold": self.RETRIEVAL_THRESHOLD,
            "timeout_seconds": self.TIMEOUT_SECONDS,
            "enable_citations": self.ENABLE_CITATIONS
        }

    def validate_config(self) -> bool:
        """
        Validate that the configuration is complete and correct.
        """
        if not self.OPENAI_API_KEY and not self.GEMINI_API_KEY:
            raise ConfigurationError("Either OPENAI_API_KEY or GEMINI_API_KEY is required")

        if not 0.0 <= self.TEMPERATURE <= 1.0:
            raise ConfigurationError("TEMPERATURE must be between 0.0 and 1.0")

        if not 1 <= self.MAX_TOKENS <= 4096:
            raise ConfigurationError("MAX_TOKENS must be between 1 and 4096")

        if not 0.0 <= self.RETRIEVAL_THRESHOLD <= 1.0:
            raise ConfigurationError("RETRIEVAL_THRESHOLD must be between 0.0 and 1.0")

        if not 1 <= self.TIMEOUT_SECONDS <= 120:
            raise ConfigurationError("TIMEOUT_SECONDS must be between 1 and 120")

        return True


# Create a singleton instance
settings = Settings()