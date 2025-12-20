from typing import List, Optional
from .config.settings import settings
import os
import cohere

class Embedder:
    def __init__(self):
        # Initialize Cohere client for embeddings
        api_key = settings.COHERE_API_KEY or os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        self.co = cohere.Client(api_key)
        print("Using Cohere embeddings for semantic similarity.")

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for a single text using Cohere
        """
        try:
            response = self.co.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_document"
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            print(f"Error generating embedding for text: {e}")
            # Return a zero vector as fallback
            return [0.0] * 1024  # Cohere embeddings are 1024-dimensional

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
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
                    input_type="search_document"
                )
                all_embeddings.extend(response.embeddings)

            return all_embeddings
        except Exception as e:
            print(f"Error generating embeddings for texts: {e}")
            # Return zero vectors as fallback
            return [[0.0] * 1024 for _ in texts]


# Singleton instance
embedder = Embedder()