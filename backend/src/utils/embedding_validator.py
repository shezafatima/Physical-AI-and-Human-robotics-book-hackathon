import numpy as np
from typing import List
from ..models.embedding import Embedding


def cosine_similarity(embedding1: Embedding, embedding2: Embedding) -> float:
    """
    Calculate cosine similarity between two embeddings
    """
    vec1 = np.array(embedding1.vector)
    vec2 = np.array(embedding2.vector)

    # Calculate cosine similarity
    dot_product = np.dot(vec1, vec2)
    norm1 = np.linalg.norm(vec1)
    norm2 = np.linalg.norm(vec2)

    if norm1 == 0 or norm2 == 0:
        return 0.0

    return float(dot_product / (norm1 * norm2))


def euclidean_distance(embedding1: Embedding, embedding2: Embedding) -> float:
    """
    Calculate Euclidean distance between two embeddings
    """
    vec1 = np.array(embedding1.vector)
    vec2 = np.array(embedding2.vector)

    return float(np.linalg.norm(vec1 - vec2))


def validate_embedding_dimensions(embedding: Embedding, expected_dims: int = 1024) -> bool:
    """
    Validate that an embedding has the expected number of dimensions
    """
    return len(embedding.vector) == expected_dims


def validate_embedding_values(embedding: Embedding, min_val: float = -2.0, max_val: float = 2.0) -> bool:
    """
    Validate that embedding values are within reasonable bounds
    """
    for val in embedding.vector:
        if val < min_val or val > max_val:
            return False
    return True


def get_embedding_statistics(embeddings: List[Embedding]) -> dict:
    """
    Get statistics about a list of embeddings
    """
    if not embeddings:
        return {}

    # Convert to numpy array for easier computation
    vectors = np.array([emb.vector for emb in embeddings])

    return {
        "count": len(embeddings),
        "dimensions": len(embeddings[0].vector),
        "mean_values": np.mean(vectors, axis=0).tolist(),
        "std_values": np.std(vectors, axis=0).tolist(),
        "min_values": np.min(vectors, axis=0).tolist(),
        "max_values": np.max(vectors, axis=0).tolist(),
        "mean_magnitude": float(np.mean(np.linalg.norm(vectors, axis=1))),
        "std_magnitude": float(np.std(np.linalg.norm(vectors, axis=1)))
    }


def find_similar_embeddings(target_embedding: Embedding,
                          candidate_embeddings: List[Embedding],
                          threshold: float = 0.7) -> List[tuple]:
    """
    Find embeddings similar to the target embedding above the threshold
    Returns list of (embedding, similarity_score) tuples
    """
    similar = []
    for candidate in candidate_embeddings:
        similarity = cosine_similarity(target_embedding, candidate)
        if similarity >= threshold:
            similar.append((candidate, similarity))

    # Sort by similarity score (descending)
    similar.sort(key=lambda x: x[1], reverse=True)
    return similar