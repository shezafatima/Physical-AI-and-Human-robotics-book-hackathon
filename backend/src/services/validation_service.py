from typing import Dict, Any
from ..models.validation_test import ValidationTestRequest, ValidationCriteria
from ..models.validation_result import ValidationResponse, ValidationMetrics
from ..models.retrieval_result import RetrievalResult
from .retrieval_service import RetrievalService
import time


class ValidationService:
    """
    Service class for validating the retrieval pipeline
    """

    def __init__(self):
        self.retrieval_service = RetrievalService()

    async def execute_validation_test(self, validation_request: ValidationTestRequest) -> ValidationResponse:
        """
        Execute a validation test on the retrieval pipeline
        """
        import uuid
        test_id = str(uuid.uuid4())

        # Use default validation criteria if not provided
        criteria = validation_request.validation_criteria or ValidationCriteria()

        # Execute the query to get results
        start_time = time.time()
        query_result = await self.retrieval_service.retrieve(validation_request.query)
        actual_time = (time.time() - start_time) * 1000

        # Calculate validation metrics
        avg_relevance_score = sum(chunk.score for chunk in query_result.results) / len(query_result.results) if query_result.results else 0

        # Evaluate validation criteria
        passed = (
            avg_relevance_score >= criteria.min_relevance_score and
            actual_time <= criteria.max_response_time_ms
        )

        # Create validation metrics
        validation_metrics = ValidationMetrics(
            avg_relevance_score=avg_relevance_score,
            response_time_ms=actual_time,
            semantic_relevance="Manual assessment needed"  # This would be filled in for manual validation
        )

        # Create the response
        return ValidationResponse(
            test_id=test_id,
            passed=passed,
            actual_results=query_result,  # Note: QueryResult is not exactly RetrievalResult, but close enough for now
            validation_metrics=validation_metrics,
            validation_criteria=criteria.dict()
        )

    def evaluate_validation_criteria(self,
                                   avg_relevance_score: float,
                                   response_time_ms: int,
                                   criteria: ValidationCriteria) -> Dict[str, Any]:
        """
        Evaluate validation criteria against actual results
        """
        results = {
            "avg_relevance_score_met": avg_relevance_score >= criteria.min_relevance_score,
            "response_time_met": response_time_ms <= criteria.max_response_time_ms,
            "overall_pass": (
                avg_relevance_score >= criteria.min_relevance_score and
                response_time_ms <= criteria.max_response_time_ms
            ),
            "details": {
                "actual_avg_relevance_score": avg_relevance_score,
                "required_min_relevance_score": criteria.min_relevance_score,
                "actual_response_time_ms": response_time_ms,
                "required_max_response_time_ms": criteria.max_response_time_ms
            }
        }
        return results