import logging
import time
from typing import List, Literal
import os
import cohere
from ..models.embedding import Embedding
from ..config.settings import settings

# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class EmbeddingService:
    """
    Service class for handling embedding generation using Cohere with fallback to local embeddings
    Includes retry logic for rate limiting and graceful fallback to local embeddings
    """

    def __init__(self):
        # Try to initialize Cohere client first
        api_key = settings.COHERE_API_KEY or os.getenv("COHERE_API_KEY")
        self.use_cohere = False
        self.cohere_initialized = False
        self.local_service = None

        if api_key:
            try:
                self.co = cohere.Client(api_key)
                self.use_cohere = True
                self.cohere_initialized = True
                logger.info("Cohere client initialized successfully")
            except Exception as e:
                logger.warning(f"Failed to initialize Cohere client: {e}. Falling back to local embeddings.")
                self._init_local_embeddings()
        else:
            logger.info("COHERE_API_KEY not provided. Using local embeddings.")
            self._init_local_embeddings()

    def _init_local_embeddings(self):
        """Initialize local embedding service as fallback"""
        try:
            from .local_embedding_service import LocalEmbeddingService
            self.local_service = LocalEmbeddingService()
            logger.info("Local embedding service initialized successfully")
        except ImportError as e:
            logger.error(f"Failed to import local embedding service: {e}")
            raise ValueError("Neither Cohere API key nor local embedding service is available")
        except Exception as e:
            logger.error(f"Failed to initialize local embedding service: {e}")
            raise ValueError("Neither Cohere API key nor local embedding service is available")

    async def generate_embedding(self, text: str, input_type: Literal["search_query", "search_document"] = "search_query") -> Embedding:
        """
        Generate embedding for a single text using Cohere with fallback to local embeddings
        Includes retry logic for rate limiting
        """
        logger.info(f"Generating embedding for text (input_type: {input_type})")

        # If Cohere is not available, use local embeddings
        if not self.cohere_initialized:
            if self.local_service:
                logger.info("Using local embedding service")
                return await self.local_service.generate_embedding(text, input_type)
            else:
                logger.error("No embedding service available")
                return Embedding(
                    vector=[0.0] * 384,  # Default size for local embeddings
                    model="fallback-zero",
                    input_type=input_type
                )

        # Try Cohere with retry logic for rate limiting
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = self.co.embed(
                    texts=[text],
                    model="embed-english-v3.0",
                    input_type=input_type
                )
                embedding_vector = response.embeddings[0]  # Return the first (and only) embedding
                logger.debug(f"Successfully generated embedding with {len(embedding_vector)} dimensions")

                return Embedding(
                    vector=embedding_vector,
                    model="embed-english-v3.0",
                    input_type=input_type
                )
            except Exception as e:
                # Check if it's a rate limit error by looking at the error message
                error_msg = str(e).lower()
                if "rate limit" in error_msg or "too many requests" in error_msg or "429" in str(e):
                    logger.warning(f"Rate limit hit on attempt {attempt + 1}, waiting before retry...")
                    wait_time = 2 ** attempt  # Exponential backoff
                    time.sleep(wait_time)
                    continue
                else:
                    logger.error(f"Cohere API error: {e}", exc_info=True)
                    break

        # If all retries failed, fall back to local embeddings
        if self.local_service:
            logger.info("Falling back to local embedding service")
            return await self.local_service.generate_embedding(text, input_type)
        else:
            logger.error("No embedding service available after retries")
            # Return a zero vector as final fallback
            return Embedding(
                vector=[0.0] * 1024,  # Cohere embeddings are 1024-dimensional
                model="embed-english-v3.0",
                input_type=input_type
            )

    async def generate_embeddings(self, texts: List[str], input_type: Literal["search_query", "search_document"] = "search_document") -> List[Embedding]:
        """
        Generate embeddings for multiple texts using Cohere with fallback to local embeddings
        Includes retry logic for rate limiting
        """
        if not texts:
            return []

        # If Cohere is not available, use local embeddings
        if not self.cohere_initialized:
            if self.local_service:
                logger.info("Using local embedding service for multiple texts")
                return await self.local_service.generate_embeddings(texts, input_type)
            else:
                logger.error("No embedding service available")
                return [
                    Embedding(
                        vector=[0.0] * 384,  # Default size for local embeddings
                        model="fallback-zero",
                        input_type=input_type
                    ) for _ in texts
                ]

        # Try Cohere with retry logic for rate limiting
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # Cohere has limits on batch size, so process in chunks if needed
                all_embeddings = []
                batch_size = 96  # Cohere's max batch size

                for i in range(0, len(texts), batch_size):
                    batch = texts[i:i + batch_size]
                    response = self.co.embed(
                        texts=batch,
                        model="embed-english-v3.0",
                        input_type=input_type
                    )
                    batch_embeddings = response.embeddings

                    # Convert to Embedding objects
                    for embedding_vector in batch_embeddings:
                        all_embeddings.append(
                            Embedding(
                                vector=embedding_vector,
                                model="embed-english-v3.0",
                                input_type=input_type
                            )
                        )

                return all_embeddings
            except Exception as e:
                # Check if it's a rate limit error by looking at the error message
                error_msg = str(e).lower()
                if "rate limit" in error_msg or "too many requests" in error_msg or "429" in str(e):
                    logger.warning(f"Rate limit hit on attempt {attempt + 1}, waiting before retry...")
                    wait_time = 2 ** attempt  # Exponential backoff
                    time.sleep(wait_time)
                    continue
                else:
                    logger.error(f"Cohere API error: {e}", exc_info=True)
                    break

        # If all retries failed, fall back to local embeddings
        if self.local_service:
            logger.info("Falling back to local embedding service for multiple texts")
            return await self.local_service.generate_embeddings(texts, input_type)
        else:
            logger.error("No embedding service available after retries")
            # Return zero vectors as fallback
            return [
                Embedding(
                    vector=[0.0] * 1024,  # Cohere embeddings are 1024-dimensional
                    model="embed-english-v3.0",
                    input_type=input_type
                ) for _ in texts
            ]

    def compare_embeddings(self, embedding1: Embedding, embedding2: Embedding, threshold: float = 0.7) -> float:
        """
        Compare two embeddings using cosine similarity
        """
        # If local service is available, delegate to it
        if self.local_service:
            return self.local_service.compare_embeddings(embedding1, embedding2, threshold)

        # Otherwise, use the original implementation
        # Calculate cosine similarity between two vectors
        dot_product = sum(a * b for a, b in zip(embedding1.vector, embedding2.vector))
        magnitude1 = sum(a * a for a in embedding1.vector) ** 0.5
        magnitude2 = sum(b * b for b in embedding2.vector) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        cosine_similarity = dot_product / (magnitude1 * magnitude2)
        return cosine_similarity

    def validate_embedding_dimensions(self, embedding: Embedding) -> bool:
        """
        Validate that the embedding has the correct dimensions (1024 for Cohere)
        """
        # If local service is available, delegate to it
        if self.local_service:
            return self.local_service.validate_embedding_dimensions(embedding)

        # Otherwise, use the original implementation
        return len(embedding.vector) == 1024