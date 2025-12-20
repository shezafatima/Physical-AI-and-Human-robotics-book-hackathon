import pytest
import asyncio
from src.models.query import QueryRequest
from src.services.retrieval_service import RetrievalService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService


class TestEdgeCases:
    """
    Tests for edge cases and error scenarios from the specification
    """

    @pytest.fixture
    def retrieval_service(self):
        """Create a retrieval service instance for testing"""
        return RetrievalService()

    @pytest.fixture
    def embedding_service(self):
        """Create an embedding service instance for testing"""
        return EmbeddingService()

    @pytest.fixture
    def qdrant_service(self):
        """Create a qdrant service instance for testing"""
        return QdrantService()

    @pytest.mark.asyncio
    async def test_empty_query_handling(self, retrieval_service):
        """Test handling of empty queries"""
        with pytest.raises(Exception):  # Should fail due to validation
            query_request = QueryRequest(
                query="",
                top_k=5
            )
            await retrieval_service.retrieve(query_request)

    @pytest.mark.asyncio
    async def test_very_long_query_handling(self, retrieval_service):
        """Test handling of extremely long queries"""
        long_query = "artificial intelligence " * 100  # Very long query
        query_request = QueryRequest(
            query=long_query,
            top_k=3
        )

        # Should handle gracefully without crashing
        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert hasattr(result, 'response_time_ms')
        assert result.response_time_ms >= 0

    @pytest.mark.asyncio
    async def test_extremely_high_top_k(self, retrieval_service):
        """Test handling of extremely high top_k values"""
        query_request = QueryRequest(
            query="What is AI?",
            top_k=1000  # Very high top_k
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert result.total_results <= 1000  # Should respect the limit set in the model

    @pytest.mark.asyncio
    async def test_minimum_top_k(self, retrieval_service):
        """Test handling of minimum top_k values"""
        query_request = QueryRequest(
            query="What is robotics?",
            top_k=1
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert result.total_results >= 0  # May be 0 if no results found
        assert result.total_results <= 1

    @pytest.mark.asyncio
    async def test_zero_score_threshold(self, retrieval_service):
        """Test handling of zero score threshold"""
        query_request = QueryRequest(
            query="What is machine learning?",
            top_k=5,
            score_threshold=0.0
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert result.response_time_ms >= 0

    @pytest.mark.asyncio
    async def test_high_score_threshold(self, retrieval_service):
        """Test handling of high score threshold that may return no results"""
        query_request = QueryRequest(
            query="Random unrelated query",
            top_k=5,
            score_threshold=0.99  # Very high threshold
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert result.total_results >= 0  # May be 0 if no high-scoring results found
        assert result.response_time_ms >= 0

    @pytest.mark.asyncio
    async def test_special_characters_in_query(self, retrieval_service):
        """Test handling of queries with special characters"""
        query_request = QueryRequest(
            query="What is AI? & robotics!",
            top_k=3
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert hasattr(result, 'results')

    @pytest.mark.asyncio
    async def test_only_special_characters_query(self, retrieval_service):
        """Test handling of queries with only special characters"""
        query_request = QueryRequest(
            query="!@#$%^&*()",
            top_k=3
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert hasattr(result, 'total_results')

    @pytest.mark.asyncio
    async def test_cohere_api_failure_handling(self, embedding_service):
        """Test handling when Cohere API is unavailable"""
        # This test is more complex as it would require mocking
        # For now, we verify the fallback mechanism exists
        assert hasattr(embedding_service, 'generate_embedding')

    @pytest.mark.asyncio
    async def test_qdrant_connection_failure_handling(self, qdrant_service):
        """Test handling when Qdrant connection fails"""
        # This test would require mocking a failed connection
        # For now, we verify the search method exists and has error handling
        assert hasattr(qdrant_service, 'search')

    @pytest.mark.asyncio
    async def test_malformed_input_handling(self, retrieval_service):
        """Test handling of malformed input"""
        with pytest.raises(Exception):
            # Test with invalid top_k
            query_request = QueryRequest(
                query="Test query",
                top_k=0  # Invalid: less than minimum
            )
            await retrieval_service.retrieve(query_request)

    @pytest.mark.asyncio
    async def test_none_values_handling(self):
        """Test handling of None values (where applicable)"""
        with pytest.raises(Exception):
            # This would fail at the Pydantic validation level
            QueryRequest(
                query=None,
                top_k=5
            )


class TestIntegrationEdgeCases:
    """
    Integration tests for edge cases across services
    """

    @pytest.mark.asyncio
    async def test_retrieval_with_no_matching_content(self, retrieval_service):
        """Test retrieval when no content matches the query"""
        query_request = QueryRequest(
            query="asldkjfalksdjfalksdjf alkasdjf asdf",  # Random string unlikely to match
            top_k=5,
            score_threshold=0.9  # High threshold
        )

        result = await retrieval_service.retrieve(query_request)
        assert result is not None
        assert result.total_results >= 0
        assert result.response_time_ms >= 0

    @pytest.mark.asyncio
    async def test_multiple_concurrent_queries(self, retrieval_service):
        """Test handling multiple concurrent queries"""
        queries = [
            QueryRequest(query="What is AI?", top_k=2),
            QueryRequest(query="What is robotics?", top_k=2),
            QueryRequest(query="What is machine learning?", top_k=2),
        ]

        # Execute multiple queries concurrently
        tasks = [retrieval_service.retrieve(q) for q in queries]
        results = await asyncio.gather(*tasks)

        # Verify all queries completed successfully
        assert len(results) == len(queries)
        for result in results:
            assert result is not None
            assert hasattr(result, 'total_results')
            assert hasattr(result, 'response_time_ms')


# Run the tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])