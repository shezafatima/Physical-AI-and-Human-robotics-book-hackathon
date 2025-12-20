from pydantic import BaseModel, Field
from typing import Optional, Literal
from .retrieval_result import RetrievalResult


class ValidationMetrics(BaseModel):
    """
    Model for validation metrics returned in validation results
    """
    avg_relevance_score: Optional[float] = Field(
        None,
        description="Average relevance score of results"
    )
    response_time_ms: Optional[int] = Field(
        None,
        description="Actual response time"
    )
    semantic_relevance: Optional[str] = Field(
        None,
        description="Manual assessment of semantic relevance"
    )


class ValidationResult(BaseModel):
    """
    Model for validation results returned by the system
    Based on the API contract specification
    """
    test_id: str = Field(
        ...,
        description="Unique identifier for the validation test"
    )
    passed: bool = Field(
        ...,
        description="Whether the test passed validation"
    )
    details: dict = Field(
        ...,
        description="Detailed results of the validation"
    )


class ValidationResponse(BaseModel):
    """
    Model for validation response returned by the API
    """
    test_id: str = Field(
        ...,
        description="Unique identifier for the validation test"
    )
    passed: bool = Field(
        ...,
        description="Whether the test passed validation"
    )
    actual_results: RetrievalResult = Field(
        ...,
        description="The actual results from the retrieval system"
    )
    validation_metrics: ValidationMetrics = Field(
        ...,
        description="Metrics used to evaluate the validation"
    )
    validation_criteria: dict = Field(
        ...,
        description="The criteria that were used for validation"
    )