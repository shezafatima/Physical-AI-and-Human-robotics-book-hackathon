import pytest
import asyncio
from src.models.validation_test import ValidationTestRequest, ValidationCriteria
from src.models.query import QueryRequest
from src.services.validation_service import ValidationService


class TestValidationService:
    """
    Contract tests for the validation service
    """

    @pytest.fixture
    def validation_service(self):
        """Create a validation service instance for testing"""
        return ValidationService()

    @pytest.mark.asyncio
    async def test_validation_service_initialization(self, validation_service):
        """Test that the validation service initializes correctly"""
        assert validation_service is not None
        assert validation_service.retrieval_service is not None

    @pytest.mark.asyncio
    async def test_execute_validation_test_with_default_criteria(self, validation_service):
        """Test executing a validation test with default criteria"""
        query_request = QueryRequest(
            query="What is humanoid robotics?",
            top_k=3,
            score_threshold=0.1
        )
        validation_request = ValidationTestRequest(
            query=query_request,
            test_type="automated"
        )

        result = await validation_service.execute_validation_test(validation_request)

        # Validate the response structure
        assert result.test_id is not None
        assert isinstance(result.passed, bool)
        assert result.validation_metrics is not None
        assert result.validation_criteria is not None

        # Check that validation criteria match expected defaults
        assert result.validation_criteria["min_relevance_score"] == 0.7
        assert result.validation_criteria["max_response_time_ms"] == 2000

    @pytest.mark.asyncio
    async def test_execute_validation_test_with_custom_criteria(self, validation_service):
        """Test executing a validation test with custom criteria"""
        query_request = QueryRequest(
            query="What are the principles of AI?",
            top_k=5,
            score_threshold=0.0
        )
        custom_criteria = ValidationCriteria(
            min_relevance_score=0.5,
            max_response_time_ms=1500
        )
        validation_request = ValidationTestRequest(
            query=query_request,
            test_type="automated",
            validation_criteria=custom_criteria
        )

        result = await validation_service.execute_validation_test(validation_request)

        # Validate the response structure
        assert result.test_id is not None
        assert isinstance(result.passed, bool)
        assert result.validation_metrics is not None
        assert result.validation_criteria is not None

        # Check that validation criteria match provided custom values
        assert result.validation_criteria["min_relevance_score"] == 0.5
        assert result.validation_criteria["max_response_time_ms"] == 1500

    @pytest.mark.asyncio
    async def test_evaluate_validation_criteria(self, validation_service):
        """Test validation criteria evaluation"""
        criteria = ValidationCriteria(
            min_relevance_score=0.6,
            max_response_time_ms=1800
        )

        # Test case where criteria are met
        results = validation_service.evaluate_validation_criteria(
            avg_relevance_score=0.7,
            response_time_ms=1500,
            criteria=criteria
        )

        assert results["overall_pass"] is True
        assert results["avg_relevance_score_met"] is True
        assert results["response_time_met"] is True

        # Test case where relevance score is too low
        results = validation_service.evaluate_validation_criteria(
            avg_relevance_score=0.5,
            response_time_ms=1500,
            criteria=criteria
        )

        assert results["overall_pass"] is False
        assert results["avg_relevance_score_met"] is False
        assert results["response_time_met"] is True

        # Test case where response time is too high
        results = validation_service.evaluate_validation_criteria(
            avg_relevance_score=0.7,
            response_time_ms=2000,
            criteria=criteria
        )

        assert results["overall_pass"] is False
        assert results["avg_relevance_score_met"] is True
        assert results["response_time_met"] is False


# Run the tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])