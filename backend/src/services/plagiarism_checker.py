from typing import List, Dict, Any
from ..config import settings
import hashlib
import requests
from ..rag import qdrant_rag


class PlagiarismChecker:
    def __init__(self):
        self.content_store = {}  # In production, this would be a database

    def check_content(self, content: str, source: str = "unknown") -> Dict[str, Any]:
        """
        Check content for potential plagiarism by comparing against known content
        """
        # Generate hash of the content for quick comparison
        content_hash = hashlib.md5(content.encode()).hexdigest()

        # Check similarity against stored content
        similarity_score = self._calculate_similarity(content)

        # Check against external services if API key is available
        external_check = self._check_external_sources(content)

        result = {
            "content_hash": content_hash,
            "similarity_score": similarity_score,
            "is_plagiarism_risk": similarity_score > 0.8,  # Threshold of 80%
            "external_matches": external_check,
            "source": source,
            "timestamp": "2025-12-10T00:00:00Z"
        }

        # Store the content for future checks
        self.content_store[content_hash] = {
            "content": content,
            "source": source
        }

        return result

    def _calculate_similarity(self, content: str) -> float:
        """
        Calculate similarity against existing content in the system
        """
        if not self.content_store:
            return 0.0

        # Simple similarity check - in reality this would use more sophisticated methods
        content_lower = content.lower()
        max_similarity = 0.0

        for stored_hash, stored_data in self.content_store.items():
            stored_content = stored_data["content"].lower()

            # Simple word overlap similarity
            content_words = set(content_lower.split())
            stored_words = set(stored_content.split())

            if content_words and stored_words:
                overlap = len(content_words.intersection(stored_words))
                union = len(content_words.union(stored_words))
                similarity = overlap / union if union > 0 else 0.0
                max_similarity = max(max_similarity, similarity)

        return max_similarity

    def _check_external_sources(self, content: str) -> List[Dict[str, str]]:
        """
        Check content against external sources (placeholder implementation)
        """
        # This is a placeholder - in a real implementation, you would check against
        # external plagiarism detection services like Copyleaks, Turnitin, etc.
        # or use search APIs to check for similar content

        # For now, return an empty list
        return []

    def add_content_to_check(self, content: str, source: str = "coursebook") -> Dict[str, Any]:
        """
        Add content to be checked for plagiarism
        """
        return self.check_content(content, source)


# Singleton instance
plagiarism_checker = PlagiarismChecker()