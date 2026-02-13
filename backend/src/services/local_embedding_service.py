import logging
from typing import List, Literal
import numpy as np
from sentence_transformers import SentenceTransformer
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


class LocalEmbeddingService:
    """
    Service class for handling embedding generation using local models (Sentence Transformers)
    This avoids API rate limits and provides offline capability
    """

    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize with a local sentence transformer model
        Default model 'all-MiniLM-L6-v2' is lightweight but effective
        Other options: 'all-mpnet-base-v2', 'multi-qa-mpnet-base-dot-v1'
        """
        try:
            self.model = SentenceTransformer(model_name)
            logger.info(f"Loaded local embedding model: {model_name}")
            # Get embedding dimension from the model
            sample_embedding = self.model.encode(["test"])
            self.embedding_dim = len(sample_embedding[0])
            logger.info(f"Embedding dimension: {self.embedding_dim}")
        except Exception as e:
            logger.error(f"Error loading local embedding model {model_name}: {e}")
            raise

    async def generate_embedding(self, text: str, input_type: Literal["search_query", "search_document"] = "search_query") -> Embedding:
        """
        Generate embedding for a single text using local model
        """
        logger.info(f"Generating local embedding for text (input_type: {input_type})")
        try:
            # Encode the text using the local model
            embedding_vector = self.model.encode([text])[0].tolist()  # Convert to list for compatibility
            logger.debug(f"Successfully generated local embedding with {len(embedding_vector)} dimensions")

            return Embedding(
                vector=embedding_vector,
                model=self.model[0].get('name', 'local-transformer') if hasattr(self.model, '__getitem__') else 'local-transformer',
                input_type=input_type
            )
        except Exception as e:
            logger.error(f"Error generating local embedding for text: {e}", exc_info=True)
            # Return a zero vector as fallback
            return Embedding(
                vector=[0.0] * self.embedding_dim,
                model='local-transformer',
                input_type=input_type
            )

    async def generate_embeddings(self, texts: List[str], input_type: Literal["search_query", "search_document"] = "search_document") -> List[Embedding]:
        """
        Generate embeddings for multiple texts using local model
        """
        if not texts:
            return []

        try:
            # Encode all texts at once for efficiency
            embedding_vectors = self.model.encode(texts).tolist()  # Convert to list for compatibility

            # Convert to Embedding objects
            embeddings = []
            for embedding_vector in embedding_vectors:
                embeddings.append(
                    Embedding(
                        vector=embedding_vector,
                        model='local-transformer',
                        input_type=input_type
                    )
                )

            return embeddings
        except Exception as e:
            logger.error(f"Error generating local embeddings for texts: {e}", exc_info=True)
            # Return zero vectors as fallback
            return [
                Embedding(
                    vector=[0.0] * self.embedding_dim,
                    model='local-transformer',
                    input_type=input_type
                ) for _ in texts
            ]

    def compare_embeddings(self, embedding1: Embedding, embedding2: Embedding, threshold: float = 0.7) -> float:
        """
        Compare two embeddings using cosine similarity
        """
        # Calculate cosine similarity between two vectors
        v1 = np.array(embedding1.vector)
        v2 = np.array(embedding2.vector)

        # Calculate cosine similarity
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0

        cosine_similarity = dot_product / (norm_v1 * norm_v2)
        return cosine_similarity

    def validate_embedding_dimensions(self, embedding: Embedding) -> bool:
        """
        Validate that the embedding has the correct dimensions
        """
        return len(embedding.vector) == self.embedding_dim