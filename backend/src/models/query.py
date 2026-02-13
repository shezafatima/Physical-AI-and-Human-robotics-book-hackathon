from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
import uuid


class QueryRequest(BaseModel):
    """
    Model for query requests to the retrieval system
    Based on the API contract specification
    """
    query: str = Field(
        ...,
        description="The natural language query to search for",
        example="What are the key principles of humanoid robotics?",
        min_length=1,
        max_length=1000
    )
    top_k: int = Field(
        5,
        description="Number of top results to retrieve (default: 5, min: 1, max: 100)",
        ge=1,
        le=100
    )
    score_threshold: float = Field(
        0.0,
        description="Minimum relevance score threshold (default: 0.0, min: 0.0, max: 1.0)",
        ge=0.0,
        le=1.0
    )
    user_id: Optional[str] = Field(None, description="Optional identifier for the user")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata for the query")


class QueryResponse(BaseModel):
    """
    Model for query response from the retrieval system
    """
    query: str
    results: list
    response_time_ms: int
    total_results: int


class Query(BaseModel):
    """
    Model for user input requesting information from book content,
    containing the question or information request
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the query")
    content: str = Field(..., min_length=1, max_length=1000, description="The actual query text from the user")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the query was submitted")
    user_id: Optional[str] = Field(None, description="Identifier for the user making the query")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional query metadata")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "abc123",
                "content": "What are the key principles of physical AI?",
                "timestamp": "2025-12-18T10:30:00Z",
                "user_id": "user-123",
                "metadata": {"session_id": "session-abc", "source": "web"}
            }
        }


class RAGQueryRequest(BaseModel):
    """
    Request model for the RAG agent query endpoint
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The user's query about book content")
    selected_text: Optional[str] = Field(
        None,
        min_length=10,
        max_length=5000,
        description="Text selected by user for context (optional)"
    )
    context_mode: str = Field(
        "full_content",
        description="How to process the query",
        pattern=r"^(full_content|selected_text)$"
    )
    top_k: int = Field(
        5,
        description="Number of top results to retrieve (default: 5, min: 1, max: 100)",
        ge=1,
        le=100
    )
    threshold: float = Field(
        0.3,
        description="Minimum relevance threshold (default: 0.3, min: 0.0, max: 1.0)",
        ge=0.0,
        le=1.0
    )
    user_id: Optional[str] = Field(None, description="Optional identifier for the user")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional metadata for the query")

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the key principles of physical AI?",
                "selected_text": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
                "context_mode": "selected_text",
                "top_k": 5,
                "threshold": 0.5,
                "user_id": "user-123",
                "metadata": {"session_id": "session-abc", "source": "web"}
            }
        }