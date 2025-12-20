from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid


class GeneratedResponse(BaseModel):
    """
    Model for the final output provided to the user, grounded in the retrieved context with proper attribution
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the response")
    query_id: str = Field(..., description="Reference to the original query")
    content: str = Field(..., min_length=10, max_length=10000, description="The generated response text")
    sources: List[str] = Field(..., description="References to source chunks used")
    generation_timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the response was generated")
    confidence_level: str = Field(..., description="Confidence level", pattern=r"^(high|medium|low|insufficient_data)$")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional generation metadata")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "resp-abc123",
                "query_id": "abc123",
                "content": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
                "sources": ["chunk-xyz789"],
                "generation_timestamp": "2025-12-18T10:30:00Z",
                "confidence_level": "high",
                "metadata": {
                    "retrieval_time_ms": 250,
                    "generation_time_ms": 1200,
                    "model_name": "gemini-pro"
                }
            }
        }