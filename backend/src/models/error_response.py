from pydantic import BaseModel, Field
from typing import Optional, Dict, Any


class ErrorResponse(BaseModel):
    """
    Model for error responses returned by the API
    Based on the API contract specification
    """
    error: str = Field(
        ...,
        description="Error code",
        example="EMBEDDING_ERROR"
    )
    message: str = Field(
        ...,
        description="Human-readable error message",
        example="Failed to generate embeddings for the query"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional error details (optional)"
    )


class APIErrorDetail(BaseModel):
    """
    Model for detailed error information
    """
    type: str = Field(
        ...,
        description="Type of error",
        example="ValueError"
    )
    message: str = Field(
        ...,
        description="Detailed error message"
    )
    location: Optional[str] = Field(
        None,
        description="Location where the error occurred",
        example="embedding_service.generate_embedding"
    )