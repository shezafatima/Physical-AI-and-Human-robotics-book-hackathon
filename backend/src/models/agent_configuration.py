from pydantic import BaseModel, Field, validator
from typing import Optional


class AgentConfiguration(BaseModel):
    """
    Model for settings including API keys, model parameters, and retrieval thresholds
    that control agent behavior
    """
    api_key: str = Field(..., description="Language model API key")
    model_name: str = Field(default="gemini-pro", description="Name of the language model to use")
    temperature: float = Field(default=0.7, ge=0.0, le=1.0, description="Temperature parameter for response generation")
    max_tokens: int = Field(default=2048, ge=1, le=4096, description="Maximum tokens in generated response")
    retrieval_threshold: float = Field(default=0.5, ge=0.0, le=1.0, description="Minimum confidence score for retrieved chunks")
    timeout_seconds: int = Field(default=30, ge=1, le=120, description="Timeout for API calls")
    enable_citations: bool = Field(default=True, description="Whether to enable citation generation")

    class Config:
        json_schema_extra = {
            "example": {
                "api_key": "your-api-key-here",
                "model_name": "gemini-pro",
                "temperature": 0.7,
                "max_tokens": 2048,
                "retrieval_threshold": 0.5,
                "timeout_seconds": 30,
                "enable_citations": True
            }
        }

    @validator('api_key')
    def validate_api_key(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('API key must not be empty')
        return v

    @validator('temperature')
    def validate_temperature(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Temperature must be between 0.0 and 1.0')
        return v

    @validator('max_tokens')
    def validate_max_tokens(cls, v):
        if not 1 <= v <= 4096:
            raise ValueError('Max tokens must be between 1 and 4096')
        return v

    @validator('retrieval_threshold')
    def validate_retrieval_threshold(cls, v):
        if not 0.0 <= v <= 1.0:
            raise ValueError('Retrieval threshold must be between 0.0 and 1.0')
        return v

    @validator('timeout_seconds')
    def validate_timeout_seconds(cls, v):
        if not 1 <= v <= 120:
            raise ValueError('Timeout seconds must be between 1 and 120')
        return v