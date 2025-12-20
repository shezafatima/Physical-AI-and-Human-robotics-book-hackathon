from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid
from src.models.context_chunk import ContextChunk


class QueryRequestSchema(BaseModel):
    """
    Schema for the query endpoint request
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The user's query about book content")
    user_id: Optional[str] = Field(None, description="Optional identifier for the user")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata for the query")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the key principles of physical AI?",
                "user_id": "user-123",
                "metadata": {"session_id": "session-abc", "source": "web"}
            }
        }


class SourceChunkSchema(BaseModel):
    """
    Schema for source chunk in the response
    """
    chunk_id: str
    content: str
    source_document: str
    page_number: Optional[int] = None
    section_title: Optional[str] = None
    confidence_score: float

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "chunk-xyz789",
                "content": "Physical AI is characterized by the integration of perception, action, and learning in embodied systems...",
                "source_document": "physical_ai_chapter_3.pdf",
                "page_number": 45,
                "section_title": "Embodied Intelligence Principles",
                "confidence_score": 0.87
            }
        }


class QueryResponseSchema(BaseModel):
    """
    Schema for the query endpoint response
    """
    id: str
    query: str
    response: str
    confidence_level: str = Field(..., pattern=r"^(high|medium|low|insufficient_data)$")
    sources: List[SourceChunkSchema]
    timestamp: datetime
    metadata: Optional[Dict[str, Any]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "id": "resp-abc123",
                "query": "What are the key principles of physical AI?",
                "response": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
                "confidence_level": "high",
                "sources": [
                    {
                        "chunk_id": "chunk-xyz789",
                        "content": "Physical AI is characterized by the integration of perception, action, and learning in embodied systems...",
                        "source_document": "physical_ai_chapter_3.pdf",
                        "page_number": 45,
                        "section_title": "Embodied Intelligence Principles",
                        "confidence_score": 0.87
                    }
                ],
                "timestamp": "2025-12-18T10:30:00Z",
                "metadata": {
                    "retrieval_time_ms": 250,
                    "generation_time_ms": 1200
                }
            }
        }


class ErrorResponseSchema(BaseModel):
    """
    Schema for error responses
    """
    error: str
    message: str
    details: Optional[Dict[str, Any]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "error": "QUERY_TOO_SHORT",
                "message": "Query must be at least 5 characters long",
                "details": {}
            }
        }


class HealthResponseSchema(BaseModel):
    """
    Schema for health check endpoint response
    """
    status: str = Field(..., pattern=r"^(healthy|degraded|unhealthy)$")
    timestamp: datetime
    details: Optional[Dict[str, Any]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-12-18T10:30:00Z",
                "details": {
                    "retrieval_service": "connected",
                    "language_model": "connected",
                    "response_time_ms": 15
                }
            }
        }