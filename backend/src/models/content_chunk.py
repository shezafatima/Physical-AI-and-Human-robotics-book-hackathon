from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class ContentChunk(BaseModel):
    """
    Model for content chunks stored in the vector database
    Based on the data model specification
    """
    id: str = Field(
        ...,
        description="Unique identifier for the content chunk",
        example="chunk_12345"
    )
    content: str = Field(
        ...,
        description="The actual text content of the chunk",
        example="Humanoid robotics is a field that focuses on creating robots with human-like characteristics..."
    )
    metadata: dict = Field(
        ...,
        description="Additional information about the content"
    )
    score: float = Field(
        0.0,
        description="Relevance score for this chunk",
        ge=0.0,
        le=1.0,
        example=0.85
    )


class ContentChunkMetadata(BaseModel):
    """
    Model for metadata associated with content chunks
    """
    source_url: Optional[str] = Field(
        None,
        description="URL where the content originated",
        example="https://example.com/book/chapter1"
    )
    title: Optional[str] = Field(
        None,
        description="Title of the content section",
        example="Introduction to Humanoid Robotics"
    )
    chapter: Optional[str] = Field(
        None,
        description="Chapter or section identifier",
        example="Chapter 1"
    )
    created_at: Optional[datetime] = Field(
        None,
        description="Timestamp when the chunk was created"
    )