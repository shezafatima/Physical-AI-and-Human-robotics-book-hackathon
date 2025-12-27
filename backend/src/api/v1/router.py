from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
from datetime import datetime
import uuid

from src.models.query import RAGQueryRequest
from src.api.schemas import (
    QueryRequestSchema,
    QueryResponseSchema,
    HealthResponseSchema,
    ErrorResponseSchema
)
from src.services.rag_agent import RAGAgent
from src.services.retrieval_service import RetrievalService
from src.config.settings import settings
from src.utils.exceptions import (
    RetrievalError,
    GenerationError,
    ValidationError,
    handle_retrieval_error,
    handle_generation_error
)
from src.utils.logger import setup_logger

# Create API router
router = APIRouter()

# Initialize services
rag_agent = RAGAgent()
retrieval_service = RetrievalService()

# Set up the retrieval service in the RAG agent
rag_agent.set_retrieval_service(retrieval_service)

# Initialize logger
logger = setup_logger("api_router")


@router.post("/query", response_model=QueryResponseSchema, summary="Submit a query to the RAG agent")
async def query_endpoint(query_request: QueryRequestSchema):
    """
    Process a user query against book content and return a grounded response.
    """
    try:
        logger.info(f"Received query: {query_request.query[:50]}...")

        # Validate the query length
        if len(query_request.query.strip()) < 3:
            raise ValidationError("Query must be at least 3 characters long", "QUERY_TOO_SHORT")

        # Create RAGQueryRequest object
        rag_query_request = RAGQueryRequest(
            query=query_request.query,
            user_id=query_request.user_id,
            metadata=query_request.metadata
        )

        # Process the query through the RAG agent (returns both response and context)
        generated_response, retrieved_context = await rag_agent.process_query(rag_query_request)

        # Format the response according to the schema
        response = QueryResponseSchema(
            id=generated_response.id,
            query=rag_query_request.query,
            response=generated_response.content,
            confidence_level=generated_response.confidence_level,
            sources=[
                {
                    "chunk_id": chunk.chunk_id,
                    "content": chunk.content,
                    "source_document": chunk.source_document,
                    "page_number": chunk.page_number,
                    "section_title": chunk.section_title,
                    "confidence_score": chunk.confidence_score
                }
                for chunk in _format_sources_for_response(generated_response.sources, retrieved_context)
            ],
            timestamp=generated_response.generation_timestamp,
            metadata=generated_response.metadata
        )

        logger.info(f"Successfully processed query, response ID: {response.id}")
        return response

    except ValidationError as e:
        logger.error(f"Validation error: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except RetrievalError as e:
        logger.error(f"Retrieval error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Retrieval error: {str(e)}")
    except GenerationError as e:
        logger.error(f"Generation error: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Generation error: {str(e)}")
    except Exception as e:
        logger.error(f"Unexpected error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/health", response_model=HealthResponseSchema, summary="Health check endpoint")
async def health_check():
    """
    Check the health status of the RAG agent service.
    """
    try:
        # Check if we can access the retrieval service
        retrieval_ok = True
        try:
            # Try a simple retrieval to test the service
            test_results = await retrieval_service.validate_retrieval_threshold("test")
        except Exception:
            retrieval_ok = False

        # Check if we can access the generation service
        generation_ok = True
        try:
            # Try to validate the model configuration
            if not settings.GEMINI_API_KEY:
                generation_ok = False
        except Exception:
            generation_ok = False

        status = "healthy"
        if not retrieval_ok or not generation_ok:
            status = "degraded"

        details = {
            "retrieval_service": "connected" if retrieval_ok else "disconnected",
            "language_model": "configured" if generation_ok else "not configured",
            "timestamp": datetime.utcnow().isoformat()
        }

        response = HealthResponseSchema(
            status=status,
            timestamp=datetime.utcnow(),
            details=details
        )

        logger.info(f"Health check completed with status: {status}")
        return response

    except Exception as e:
        logger.error(f"Health check error: {str(e)}")
        return HealthResponseSchema(
            status="unhealthy",
            timestamp=datetime.utcnow(),
            details={"error": str(e)}
        )


@router.get("/gemini-health", response_model=HealthResponseSchema, summary="Gemini API connectivity health check")
async def gemini_health_check():
    """
    Check the health status of the Gemini API connectivity.
    """
    try:
        from src.services.gemini_client import GeminiClient

        # Test the Gemini API connection
        gemini_client = GeminiClient()
        connection_ok = await gemini_client.check_connection()

        status = "healthy" if connection_ok else "unhealthy"

        details = {
            "gemini_api": "connected" if connection_ok else "disconnected",
            "model_name": settings.MODEL_NAME,
            "timestamp": datetime.utcnow().isoformat()
        }

        response = HealthResponseSchema(
            status=status,
            timestamp=datetime.utcnow(),
            details=details
        )

        logger.info(f"Gemini health check completed with status: {status}")
        return response

    except Exception as e:
        logger.error(f"Gemini health check error: {str(e)}")
        return HealthResponseSchema(
            status="unhealthy",
            timestamp=datetime.utcnow(),
            details={"gemini_api_error": str(e)}
        )


# Helper function to format sources for response
def _format_sources_for_response(source_ids: list, retrieved_context):
    """
    Helper function to format sources for the response.
    In a real implementation, this would retrieve chunks by ID from storage.
    """
    # For now, return the chunks from the retrieved context
    # In a real implementation, you would fetch the specific chunks by ID
    if retrieved_context and hasattr(retrieved_context, 'chunks'):
        return retrieved_context.chunks
    return []