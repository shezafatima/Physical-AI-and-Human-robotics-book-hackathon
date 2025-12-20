import logging
from typing import List, Optional
import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from ..models.content_chunk import ContentChunk, ContentChunkMetadata
from ..config.settings import settings

# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class QdrantService:
    """
    Service class for handling Qdrant vector database operations
    """

    def __init__(self, collection_name: str = "coursebook_content"):
        # Use environment variables if settings values are None
        qdrant_url = settings.QDRANT_URL or os.getenv("QDRANT_URL")
        qdrant_api_key = settings.QDRANT_API_KEY or os.getenv("QDRANT_API_KEY")

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False
        )
        self.collection_name = collection_name
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """Create the collection if it doesn't exist"""
        try:
            # Try to get collection info to see if it exists
            self.client.get_collection(self.collection_name)
        except:
            # If collection doesn't exist, create it
            vector_size = 1024  # Using 1024 dimensions for Cohere embeddings
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE
                )
            )

    async def search(self, query_vector: List[float], top_k: int = 5, score_threshold: float = 0.0) -> List[ContentChunk]:
        """
        Search for similar vectors in Qdrant and return content chunks
        """
        logger.info(f"Starting Qdrant search with top_k={top_k}, score_threshold={score_threshold}")
        try:
            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold
            )
            logger.debug(f"Qdrant search returned {len(search_results)} results")

            # Convert search results to ContentChunk objects
            content_chunks = []
            for result in search_results:
                payload = result.payload
                metadata_dict = payload.get("metadata", {})

                # Create ContentChunkMetadata from the payload
                metadata = ContentChunkMetadata(
                    source_url=metadata_dict.get("source_url"),
                    title=metadata_dict.get("title"),
                    chapter=metadata_dict.get("chapter"),
                    created_at=metadata_dict.get("created_at")
                )

                # Create ContentChunk object
                content_chunk = ContentChunk(
                    id=str(result.id),
                    content=payload.get("content", ""),
                    metadata=metadata.dict(),
                    score=result.score
                )
                content_chunks.append(content_chunk)

            logger.info(f"Successfully converted {len(content_chunks)} search results to ContentChunk objects")
            return content_chunks
        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}", exc_info=True)
            return []

    async def add_content_chunk(self, content: str, metadata: dict, vector: List[float]):
        """
        Add a content chunk with its embedding to Qdrant
        """
        try:
            # Generate a unique ID for this chunk
            import hashlib
            content_hash = hashlib.md5(content.encode()).hexdigest()

            # Prepare the point for insertion
            point = models.PointStruct(
                id=content_hash,
                vector=vector,
                payload={
                    "content": content,
                    "metadata": metadata
                }
            )

            # Upload the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )
            return True
        except Exception as e:
            print(f"Error adding content chunk to Qdrant: {e}")
            return False

    async def batch_add_content_chunks(self, contents: List[tuple]):
        """
        Add multiple content chunks with their embeddings to Qdrant
        Each tuple should contain (content, metadata, vector)
        """
        try:
            points = []
            for i, (content, metadata, vector) in enumerate(contents):
                # Generate a unique ID for this chunk
                import hashlib
                content_hash = hashlib.md5(f"{content}_{i}".encode()).hexdigest()

                # Prepare the point for insertion
                point = models.PointStruct(
                    id=content_hash,
                    vector=vector,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
                points.append(point)

            # Upload all points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            print(f"Error adding content chunks to Qdrant: {e}")
            return False

    def get_collection_info(self):
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return None