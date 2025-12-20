from pydantic import BaseModel, Field
from typing import List, Literal


class Embedding(BaseModel):
    """
    Model for embedding vectors used in semantic similarity
    Based on the data model specification
    """
    vector: List[float] = Field(
        ...,
        description="The embedding vector (1024 dimensions for Cohere embed-english-v3.0)",
        # Example truncated for readability
    )
    model: str = Field(
        ...,
        description="The model used to generate the embedding",
        example="embed-english-v3.0"
    )
    input_type: Literal["search_query", "search_document"] = Field(
        ...,
        description="The type of input used",
        example="search_query"
    )

    class Config:
        # Allow 1024 dimensions for Cohere embeddings
        schema_extra = {
            "example": {
                "vector": [0.1, 0.2, 0.3] + [0.0] * 1021,  # truncated for example
                "model": "embed-english-v3.0",
                "input_type": "search_query"
            }
        }