from pydantic import BaseModel, Field
from typing import Optional


class ContextChunk(BaseModel):
    """
    Model for individual text chunk from the book content that is relevant to the query
    """
    chunk_id: str = Field(..., description="Identifier for the specific chunk")
    content: str = Field(..., min_length=10, max_length=5000, description="The actual text content of the chunk")
    source_document: str = Field(..., description="Reference to the source document")
    page_number: Optional[int] = Field(None, description="Page number in the source document")
    section_title: Optional[str] = Field(None, description="Title of the section containing the chunk")
    confidence_score: float = Field(..., ge=0.0, le=1.0, description="Confidence score for this chunk's relevance")

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