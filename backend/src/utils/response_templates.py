from typing import Dict, List


class ResponseTemplates:
    """
    Collection of standardized response templates for different scenarios.
    """

    # Templates for insufficient data scenarios
    INSUFFICIENT_DATA_TEMPLATES = {
        "no_relevant_content": [
            "I don't have enough information from the provided sources to answer this question.",
            "Based on the available content, I cannot provide a relevant answer to your query.",
            "The provided materials do not contain sufficient information to address your question."
        ],
        "low_confidence": [
            "I found some information related to your query, but the confidence in the results is low.",
            "The retrieved content has low relevance to your question, so I cannot provide a confident answer.",
            "The sources found have weak connections to your query, making it difficult to provide an accurate response."
        ],
        "no_content_found": [
            "No relevant content was found in the book materials for your query.",
            "I couldn't locate any documents that match your question in the knowledge base.",
            "The search through the book content did not return any matching results for your query."
        ]
    }

    # Templates for high-quality responses
    HIGH_QUALITY_TEMPLATES_PREFIX = [
        "Based on the provided materials:",
        "According to the book content:",
        "From the course materials:"
    ]

    @staticmethod
    def get_insufficient_data_response(scenario: str = "no_relevant_content") -> str:
        """
        Get a standardized response for insufficient data scenarios.

        Args:
            scenario: The type of insufficient data scenario

        Returns:
            A standardized response string
        """
        templates = ResponseTemplates.INSUFFICIENT_DATA_TEMPLATES.get(scenario,
                                                                    ResponseTemplates.INSUFFICIENT_DATA_TEMPLATES["no_relevant_content"])
        # Return the first template as a default; in a real implementation you might randomize this
        return templates[0] if templates else "I cannot provide an answer based on the available information."

    @staticmethod
    def get_low_confidence_warning() -> str:
        """
        Get a standardized warning for low-confidence responses.

        Returns:
            A warning string about confidence level
        """
        return ("Note: The confidence in this response is low. The information provided "
                "may not fully address your query due to limited matching content.")

    @staticmethod
    def format_sources_list(sources: List[Dict]) -> str:
        """
        Format a list of sources into a readable string.

        Args:
            sources: List of source dictionaries

        Returns:
            Formatted sources string
        """
        if not sources:
            return ""

        formatted_sources = ["\nSources:"]
        for i, source in enumerate(sources, 1):
            doc_info = source.get('source_document', 'Unknown Document')
            page_info = f" (page {source.get('page_number')})" if source.get('page_number') else ""
            formatted_sources.append(f"  {i}. {doc_info}{page_info}")

        return "\n".join(formatted_sources)

    @staticmethod
    def format_citation_reference(chunk_id: str, source_document: str) -> str:
        """
        Format a citation reference for use in responses.

        Args:
            chunk_id: The ID of the content chunk
            source_document: The source document name

        Returns:
            Formatted citation string
        """
        return f"[Reference: {source_document}]"