from pydantic_settings import BaseSettings
from typing import Optional, List


class Settings(BaseSettings):
    # API Keys
    gemini_api_key: Optional[str] = None
    cohere_api_key: Optional[str] = None

    # Database
    database_url: str = "sqlite:///./coursebook.db"

    # Qdrant
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None

    # Application
    app_env: str = "development"
    log_level: str = "info"

    class Config:
        env_file = [".env.local", ".env"]
        case_sensitive = True
        extra = "ignore"  # Ignore extra fields that are not defined in the model


settings = Settings()