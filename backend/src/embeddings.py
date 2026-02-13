from typing import List, Optional
from .config.settings import settings
import asyncio
import threading

class EmbedderAdapter:
    """
    Adapter class that wraps the EmbeddingService to maintain backward compatibility
    with the existing embedder interface while using the new robust embedding service.
    """

    def __init__(self):
        # Initialize the new EmbeddingService
        from .services.embedding_service import EmbeddingService
        try:
            self.service = EmbeddingService()
            print("Initialized EmbeddingService with Cohere/local fallback capability.")
        except Exception as e:
            print(f"Warning: Could not initialize embedding service: {e}")
            raise

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for a single text using the EmbeddingService
        """
        try:
            # Run the async method synchronously
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            embedding_obj = loop.run_until_complete(
                self.service.generate_embedding(text, input_type="search_document")
            )
            loop.close()
            return embedding_obj.vector
        except Exception as e:
            print(f"Error generating embedding for text: {e}")
            # Return a zero vector as fallback
            return [0.0] * 1024  # Cohere embeddings are 1024-dimensional

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts using the EmbeddingService
        """
        if not texts:
            return []

        try:
            # Run the async method synchronously
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            embedding_objects = loop.run_until_complete(
                self.service.generate_embeddings(texts, input_type="search_document")
            )
            loop.close()

            return [emb.vector for emb in embedding_objects]
        except Exception as e:
            print(f"Error generating embeddings for texts: {e}")
            # Return zero vectors as fallback
            return [[0.0] * 1024 for _ in texts]


# Singleton instance
embedder = EmbedderAdapter()