import logging
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
    Service class for handling embedding generation using Cohere
    """

    def __init__(self):
        # Initialize Cohere client
        api_key = settings.COHERE_API_KEY or os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.co = cohere.Client(api_key)

    async def generate_embedding(self, text: str, input_type: Literal["search_query", "search_document"] = "search_query") -> Embedding:
        """
        Generate embedding for a single text using Cohere
        """
        logger.info(f"Generating embedding for text (input_type: {input_type})")
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
            logger.error(f"Error generating embedding for text: {e}", exc_info=True)
            # Return a zero vector as fallback
            return Embedding(
                vector=[0.0] * 1024,  # Cohere embeddings are 1024-dimensional
                model="embed-english-v3.0",
                input_type=input_type
            )

    async def generate_embeddings(self, texts: List[str], input_type: Literal["search_query", "search_document"] = "search_document") -> List[Embedding]:
        """
        Generate embeddings for multiple texts using Cohere
        """
        if not texts:
            return []

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
            print(f"Error generating embeddings for texts: {e}")
            # Return zero vectors as fallback
            return [
                Embedding(
                    vector=[0.0] * 1024,
                    model="embed-english-v3.0",
                    input_type=input_type
                ) for _ in texts
            ]

    def compare_embeddings(self, embedding1: Embedding, embedding2: Embedding, threshold: float = 0.7) -> float:
        """
        Compare two embeddings using cosine similarity
        """
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
        return len(embedding.vector) == 1024