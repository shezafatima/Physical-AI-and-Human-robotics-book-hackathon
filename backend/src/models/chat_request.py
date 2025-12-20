from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class ChatRequest(BaseModel):
    """
    Model for chat requests from the frontend, including support for selected text context
    """
    message: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's question or message"
    )
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
    session_id: Optional[str] = Field(
        None,
        description="Session identifier for maintaining conversation context"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="When the request was made"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What are the key principles of physical AI?",
                "selected_text": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
                "context_mode": "selected_text",
                "session_id": "sess-abc123",
                "timestamp": "2025-12-18T10:30:00Z"
            }
        }


class ChatResponse(BaseModel):
    """
    Model for chat responses from the backend RAG agent
    """
    response: str = Field(
        ...,
        min_length=10,
        max_length=10000,
        description="The AI-generated answer to the user's query"
    )
    sources: Optional[List[str]] = Field(
        None,
        description="Citations to source documents used in the response"
    )
    confidence: str = Field(
        ...,
        description="Confidence level of the response",
        pattern=r"^(high|medium|low|insufficient_data)$"
    )
    error: Optional[str] = Field(
        None,
        description="Error message if processing failed"
    )
    status: str = Field(
        ...,
        description="Processing status",
        pattern=r"^(success|error|timeout)$"
    )
    context: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional context information including retrieved documents"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="ISO 8601 timestamp of response generation"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "response": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
                "sources": ["Chapter 3: Embodied Cognition", "Chapter 5: Sensorimotor Learning"],
                "confidence": "high",
                "status": "success",
                "context": {
                    "retrieved_docs_count": 3,
                    "source_docs": ["doc1", "doc2", "doc3"],
                    "confidence_scores": [0.8, 0.7, 0.6]
                },
                "timestamp": "2025-12-18T10:30:00Z"
            }
        }


class ChatSession(BaseModel):
    """
    Model for maintaining chat session state
    """
    session_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the chat session"
    )
    messages: List[Dict[str, Any]] = Field(
        default_factory=list,
        description="Array of message objects representing the conversation history"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="ISO 8601 formatted timestamp when the session started"
    )
    active: bool = Field(
        True,
        description="Indicates if the session is currently active"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess-abc123",
                "messages": [
                    {
                        "id": "msg-1",
                        "role": "user",
                        "content": "What are the key principles of physical AI?",
                        "timestamp": "2025-12-18T10:30:00Z"
                    },
                    {
                        "id": "msg-2",
                        "role": "assistant",
                        "content": "The key principles include embodied cognition...",
                        "timestamp": "2025-12-18T10:30:05Z",
                        "sources": ["Chapter 3: Embodied Cognition"]
                    }
                ],
                "timestamp": "2025-12-18T10:30:00Z",
                "active": True
            }
        }