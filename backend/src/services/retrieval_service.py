import logging
from typing import List, Optional
from ..models.query import QueryRequest
from ..models.retrieval_result import RetrievalResult, QueryResult
from ..models.content_chunk import ContentChunk
from ..models.embedding import Embedding
from .embedding_service import EmbeddingService
from .qdrant_service import QdrantService
import time


# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class RetrievalService:
    """
    Service class for handling retrieval operations
    Combines embedding generation and Qdrant search functionality
    """

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()

    async def retrieve(self, query_request: QueryRequest) -> QueryResult:
        """
        Main retrieval method that processes a query and returns relevant content chunks
        """
        logger.info(f"Starting retrieval for query: '{query_request.query[:50]}...'")
        start_time = time.time()

        try:
            # Generate embedding for the query
            query_embedding = await self.embedding_service.generate_embedding(
                query_request.query,
                input_type="search_query"
            )
            logger.debug(f"Generated embedding for query, vector length: {len(query_embedding.vector)}")

            # Search Qdrant for similar content
            content_chunks = await self.qdrant_service.search(
                query_embedding.vector,
                top_k=query_request.top_k,
                score_threshold=query_request.score_threshold
            )
            logger.info(f"Found {len(content_chunks)} relevant content chunks")

            # Calculate response time
            response_time_ms = int((time.time() - start_time) * 1000)
            logger.info(f"Retrieval completed in {response_time_ms}ms")

            # Return formatted results
            result = QueryResult(
                results=content_chunks,
                response_time_ms=response_time_ms,
                total_results=len(content_chunks)
            )

            logger.debug(f"Returning {result.total_results} results with response time {result.response_time_ms}ms")
            return result
        except Exception as e:
            logger.error(f"Error during retrieval: {str(e)}", exc_info=True)
            raise

    async def validate_embedding_compatibility(self, text: str) -> bool:
        """
        Validate that query embeddings are compatible with stored content embeddings
        """
        # Generate embedding using query input type
        query_embedding = await self.embedding_service.generate_embedding(
            text,
            input_type="search_query"
        )

        # Generate embedding using document input type
        document_embedding = await self.embedding_service.generate_embedding(
            text,
            input_type="search_document"
        )

        # Check if dimensions match (should be 1024 for Cohere embed-english-v3.0)
        return len(query_embedding.vector) == len(document_embedding.vector) == 1024

    async def validate_retrieval_performance(self, query_request: QueryRequest) -> dict:
        """
        Validate retrieval performance metrics
        """
        start_time = time.time()

        # Perform retrieval
        result = await self.retrieve(query_request)

        # Calculate performance metrics
        actual_time = (time.time() - start_time) * 1000

        return {
            "response_time_ms": actual_time,
            "total_results": result.total_results,
            "within_threshold": actual_time <= 2000  # 2 second threshold
        }