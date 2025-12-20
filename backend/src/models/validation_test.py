from pydantic import BaseModel, Field
from typing import List, Optional, Literal
from .query import QueryRequest
from .content_chunk import ContentChunk


class ValidationCriteria(BaseModel):
    """
    Model for validation criteria used to determine if a test passes
    """
    min_relevance_score: float = Field(
        0.7,
        description="Minimum acceptable relevance score",
        ge=0.0,
        le=1.0
    )
    max_response_time_ms: int = Field(
        2000,
        description="Maximum acceptable response time in milliseconds",
        ge=0
    )


class ValidationTest(BaseModel):
    """
    Model for validation tests of the retrieval pipeline
    Based on the data model specification
    """
    id: Optional[str] = Field(
        None,
        description="Unique identifier for the validation test"
    )
    query: QueryRequest = Field(
        ...,
        description="The test query to execute"
    )
    expected_results: Optional[List[ContentChunk]] = Field(
        None,
        description="Expected results for validation (optional)"
    )
    test_type: Literal["automated", "manual"] = Field(
        ...,
        description="Type of validation test",
        example="automated"
    )
    validation_criteria: ValidationCriteria = Field(
        default_factory=ValidationCriteria,
        description="Criteria for determining if test passed"
    )


class ValidationTestRequest(BaseModel):
    """
    Model for validation test requests to the API
    """
    query: QueryRequest = Field(
        ...,
        description="The test query to execute"
    )
    test_type: Literal["automated", "manual"] = Field(
        "automated",
        description="Type of validation test"
    )
    validation_criteria: Optional[ValidationCriteria] = Field(
        None,
        description="Criteria for determining if test passed (uses defaults if not provided)"
    )