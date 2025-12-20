import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.rag_agent import RAGAgent
from src.models.query import RAGQueryRequest
from src.models.context_chunk import ContextChunk
from src.models.retrieved_context import RetrievedContext
from datetime import datetime
import uuid


@pytest.fixture
def rag_agent():
    """Create a RAGAgent instance for testing"""
    agent = RAGAgent()
    # Mock the gemini_client to avoid actual API calls
    agent.gemini_client = MagicMock()
    agent.gemini_client.generate_content = AsyncMock(return_value="Test response")
    return agent


@pytest.fixture
def mock_retrieval_service():
    """Create a mock retrieval service"""
    mock_service = AsyncMock()
    mock_service.retrieve = AsyncMock()
    return mock_service


@pytest.fixture
def sample_query_request():
    """Sample query request for testing"""
    return RAGQueryRequest(
        query="What are the principles of physical AI?",
        user_id="test_user_123"
    )


@pytest.fixture
def sample_context_chunk():
    """Sample context chunk for testing"""
    return ContextChunk(
        chunk_id="test_chunk_1",
        content="Physical AI combines perception, action, and learning in embodied systems.",
        source_document="physical_ai_principles.pdf",
        page_number=45,
        section_title="Introduction to Physical AI",
        confidence_score=0.85
    )


@pytest.mark.asyncio
async def test_process_query_success(rag_agent, mock_retrieval_service, sample_query_request, sample_context_chunk):
    """Test successful query processing"""
    # Set up mock retrieval service
    retrieved_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[sample_context_chunk],
        confidence_scores=[0.85],
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    mock_retrieval_service.retrieve.return_value = retrieved_context
    rag_agent.set_retrieval_service(mock_retrieval_service)

    # Process the query
    response, context = await rag_agent.process_query(sample_query_request)

    # Verify the response
    assert response is not None
    assert response.content == "Test response"
    assert response.confidence_level in ["high", "medium", "low", "insufficient_data"]
    assert context == retrieved_context


@pytest.mark.asyncio
async def test_process_query_empty_context(rag_agent, mock_retrieval_service, sample_query_request):
    """Test query processing when no context is retrieved"""
    # Set up mock retrieval service to return empty context
    empty_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[],
        confidence_scores=[],
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    mock_retrieval_service.retrieve.return_value = empty_context
    rag_agent.set_retrieval_service(mock_retrieval_service)

    # Process the query
    response, context = await rag_agent.process_query(sample_query_request)

    # Verify the response handles empty context appropriately
    assert response is not None
    assert response.confidence_level == "insufficient_data"
    assert context == empty_context


@pytest.mark.asyncio
async def test_process_query_retrieval_error(rag_agent, sample_query_request):
    """Test query processing when retrieval service fails"""
    # Set up mock retrieval service to raise an error
    mock_retrieval_service = AsyncMock()
    mock_retrieval_service.retrieve.side_effect = Exception("Retrieval failed")

    rag_agent.set_retrieval_service(mock_retrieval_service)

    # Process the query - should return fallback response
    response, context = await rag_agent.process_query(sample_query_request)

    # Verify fallback response is returned
    assert response is not None
    assert response.confidence_level == "insufficient_data"
    assert "error" in response.metadata


def test_format_context_for_model_with_chunks(rag_agent, sample_context_chunk):
    """Test formatting context with chunks"""
    retrieved_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[sample_context_chunk],
        confidence_scores=[0.85],
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    formatted_context = rag_agent._format_context_for_model(retrieved_context)

    assert "[Source 1]" in formatted_context
    assert sample_context_chunk.source_document in formatted_context
    assert sample_context_chunk.content in formatted_context


def test_format_context_for_model_empty(rag_agent):
    """Test formatting context when no chunks available"""
    empty_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[],
        confidence_scores=[],
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    formatted_context = rag_agent._format_context_for_model(empty_context)

    assert "No relevant information found" in formatted_context


def test_determine_confidence_level_high(rag_agent, sample_context_chunk):
    """Test confidence level determination for high confidence"""
    retrieved_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[sample_context_chunk],
        confidence_scores=[0.85],  # High confidence
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    confidence_level = rag_agent._determine_confidence_level(retrieved_context)

    assert confidence_level == "high"


def test_determine_confidence_level_medium(rag_agent, sample_context_chunk):
    """Test confidence level determination for medium confidence"""
    # Modify the chunk to have medium confidence
    medium_chunk = sample_context_chunk.copy(update={"confidence_score": 0.55})

    retrieved_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[medium_chunk],
        confidence_scores=[0.55],  # Medium confidence
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    confidence_level = rag_agent._determine_confidence_level(retrieved_context)

    assert confidence_level == "medium"


def test_determine_confidence_level_low(rag_agent, sample_context_chunk):
    """Test confidence level determination for low confidence"""
    # Modify the chunk to have low confidence
    low_chunk = sample_context_chunk.copy(update={"confidence_score": 0.3})

    retrieved_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[low_chunk],
        confidence_scores=[0.3],  # Low confidence
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    confidence_level = rag_agent._determine_confidence_level(retrieved_context)

    assert confidence_level == "low"


def test_determine_confidence_level_insufficient_data(rag_agent):
    """Test confidence level determination for no chunks"""
    empty_context = RetrievedContext(
        id=str(uuid.uuid4()),
        query_id=str(uuid.uuid4()),
        chunks=[],
        confidence_scores=[],
        retrieval_timestamp=datetime.utcnow(),
        metadata={"test": True}
    )

    confidence_level = rag_agent._determine_confidence_level(empty_context)

    assert confidence_level == "insufficient_data"