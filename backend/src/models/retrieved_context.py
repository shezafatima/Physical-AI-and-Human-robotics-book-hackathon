from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
import uuid
from .context_chunk import ContextChunk


class RetrievedContext(BaseModel):
    """
    Model for relevant book passages and chunks extracted from the validated retrieval pipeline
    that inform the response
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the context set")
    query_id: str = Field(..., description="Reference to the original query")
    chunks: List[ContextChunk] = Field(default=[], description="Retrieved text chunks", max_items=10)
    confidence_scores: List[float] = Field(default=[], description="Confidence scores for each chunk", max_items=10)
    retrieval_timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the retrieval was performed")
    metadata: Optional[Dict[str, Any]] = Field(None, description="Additional retrieval metadata")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "ctx-123",
                "query_id": "abc123",
                "chunks": [
                    {
                        "chunk_id": "chunk-xyz789",
                        "content": "Physical AI is characterized by the integration of perception, action, and learning in embodied systems...",
                        "source_document": "physical_ai_chapter_3.pdf",
                        "page_number": 45,
                        "section_title": "Embodied Intelligence Principles",
                        "confidence_score": 0.87
                    }
                ],
                "confidence_scores": [0.87],
                "retrieval_timestamp": "2025-12-18T10:30:00Z",
                "metadata": {"retrieval_method": "vector_search", "model_version": "v1.2"}
            }
        }

    def __init__(self, **data):
        super().__init__(**data)
        # Validate that chunks and confidence_scores have the same length (only when both have items)
        if len(self.chunks) != len(self.confidence_scores):
            raise ValueError("Number of chunks must match number of confidence scores")

        # Validate that all confidence scores are between 0.0 and 1.0
        for score in self.confidence_scores:
            if not 0.0 <= score <= 1.0:
                raise ValueError(f"Confidence score {score} is not between 0.0 and 1.0")