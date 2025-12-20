import pytest
import asyncio
import time
from src.models.query import QueryRequest
from src.services.retrieval_service import RetrievalService
from src.services.validation_service import ValidationService


class TestPerformance:
    """
    Performance and benchmark tests for the retrieval system
    """

    @pytest.fixture
    def retrieval_service(self):
        """Create a retrieval service instance for testing"""
        return RetrievalService()

    @pytest.fixture
    def validation_service(self):
        """Create a validation service instance for testing"""
        return ValidationService()

    @pytest.mark.asyncio
    async def test_retrieve_response_time_under_threshold(self, retrieval_service):
        """Test that retrieval response time is under the performance threshold"""
        query_request = QueryRequest(
            query="What is machine learning?",
            top_k=5,
            score_threshold=0.0
        )

        start_time = time.time()
        result = await retrieval_service.retrieve(query_request)
        end_time = time.time()

        response_time_ms = (end_time - start_time) * 1000

        # Ensure response time is under 2 seconds (2000ms) for 90% of requests
        assert response_time_ms < 2000, f"Response time {response_time_ms}ms exceeds threshold"

    @pytest.mark.asyncio
    @pytest.mark.benchmark
    async def test_multiple_queries_performance(self, retrieval_service):
        """Test performance under multiple concurrent queries"""
        queries = [
            "What is artificial intelligence?",
            "Explain neural networks",
            "What are the applications of robotics?",
            "How does deep learning work?",
            "What is computer vision?"
        ]

        start_time = time.time()

        # Execute multiple queries concurrently
        tasks = []
        for query_text in queries:
            query_request = QueryRequest(
                query=query_text,
                top_k=3,
                score_threshold=0.0
            )
            task = retrieval_service.retrieve(query_request)
            tasks.append(task)

        results = await asyncio.gather(*tasks)

        end_time = time.time()
        total_time = (end_time - start_time) * 1000

        # Verify all queries returned results
        assert len(results) == len(queries)
        for result in results:
            assert result is not None
            assert hasattr(result, 'results')
            assert hasattr(result, 'response_time_ms')
            assert result.total_results >= 0

        # Ensure total time for all queries is reasonable
        assert total_time < 5000, f"Total time for {len(queries)} queries was {total_time}ms"

    @pytest.mark.asyncio
    async def test_validation_response_time(self, validation_service):
        """Test that validation response time meets performance requirements"""
        query_request = QueryRequest(
            query="What is the purpose of validation?",
            top_k=2,
            score_threshold=0.0
        )
        validation_request = QueryRequest(
            query=query_request.query,
            top_k=query_request.top_k,
            score_threshold=query_request.score_threshold
        )

        start_time = time.time()
        result = await validation_service.execute_validation_test(validation_request)
        end_time = time.time()

        response_time_ms = (end_time - start_time) * 1000

        # Ensure validation response time is under 3 seconds (3000ms)
        assert response_time_ms < 3000, f"Validation response time {response_time_ms}ms exceeds threshold"

    @pytest.mark.asyncio
    async def test_top_k_performance_impact(self, retrieval_service):
        """Test how top_k parameter affects performance"""
        query_text = "What is robotics?"

        # Test with smaller top_k
        query_request_small = QueryRequest(
            query=query_text,
            top_k=3,
            score_threshold=0.0
        )
        start_time = time.time()
        result_small = await retrieval_service.retrieve(query_request_small)
        time_small = (time.time() - start_time) * 1000

        # Test with larger top_k
        query_request_large = QueryRequest(
            query=query_text,
            top_k=10,
            score_threshold=0.0
        )
        start_time = time.time()
        result_large = await retrieval_service.retrieve(query_request_large)
        time_large = (time.time() - start_time) * 1000

        # The larger top_k should take more time but still be reasonable
        assert time_small < 2000, f"Small top_k response time {time_small}ms exceeds threshold"
        assert time_large < 3000, f"Large top_k response time {time_large}ms exceeds threshold"

        # Results should match requested top_k values
        assert result_small.total_results <= 3
        assert result_large.total_results <= 10

    @pytest.mark.asyncio
    async def test_score_threshold_performance(self, retrieval_service):
        """Test how score_threshold affects performance"""
        query_request = QueryRequest(
            query="What are the principles of AI?",
            top_k=5,
            score_threshold=0.5  # Higher threshold
        )

        start_time = time.time()
        result = await retrieval_service.retrieve(query_request)
        end_time = time.time()

        response_time_ms = (end_time - start_time) * 1000

        # Ensure response time is under threshold even with score filtering
        assert response_time_ms < 2000, f"Response time with threshold {response_time_ms}ms exceeds threshold"


# Run the tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])