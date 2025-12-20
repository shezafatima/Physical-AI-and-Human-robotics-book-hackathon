from pydantic import BaseModel, Field
from typing import List
from .content_chunk import ContentChunk
from .embedding import Embedding
from .query import QueryRequest


class SearchParams(BaseModel):
    """
    Model for search parameters used in retrieval
    """
    top_k: int = Field(
        ...,
        description="Number of results requested"
    )
    score_threshold: float = Field(
        0.0,
        description="Score threshold used for filtering"
    )


class RetrievalResult(BaseModel):
    """
    Model for retrieval results returned by the system
    Based on the API contract specification
    """
    results: List[ContentChunk] = Field(
        default_factory=list,
        description="The retrieved content chunks"
    )
    query_embedding: Embedding = Field(
        ...,
        description="The embedding of the original query"
    )
    search_params: SearchParams = Field(
        ...,
        description="Parameters used for the search"
    )
    response_time_ms: int = Field(
        ...,
        description="Time taken to process the query in milliseconds",
        ge=0
    )
    total_results: int = Field(
        ...,
        description="Total number of results returned",
        ge=0
    )


class QueryResult(BaseModel):
    """
    Simplified model for query results (without embedding details for response)
    """
    results: List[ContentChunk] = Field(
        default_factory=list,
        description="The retrieved content chunks with metadata and relevance scores"
    )
    response_time_ms: int = Field(
        ...,
        description="Time taken to process the query in milliseconds",
        ge=0
    )
    total_results: int = Field(
        ...,
        description="Total number of results returned",
        ge=0
    )