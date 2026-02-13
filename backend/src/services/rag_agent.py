from typing import List, Optional, Dict, Any
from openai import AsyncOpenAI
from dotenv import load_dotenv
from src.models.query import RAGQueryRequest
from src.models.retrieved_context import RetrievedContext
from src.models.generated_response import GeneratedResponse
from src.models.context_chunk import ContextChunk
from src.config.settings import settings
from src.utils.exceptions import GenerationError, RetrievalError
from src.utils.logger import setup_logger
from datetime import datetime
import uuid
import os
from .openai_chat_completions_model import OpenAIChatCompletionsModel

# Load environment variables
load_dotenv()


class RAGAgent:
    """
    Service class that manages the interaction between retrieval and generation
    for the RAG agent functionality.
    """

    def __init__(self):
        self.logger = setup_logger("rag_agent")

        # Use the exact configuration pattern specified by the user
        key = settings.GEMINI_API_KEY or settings.OPENAI_API_KEY
        base_url = settings.BASE_URL

        if not key:
            raise ValueError("Either GEMINI_API_KEY or OPENAI_API_KEY environment variable must be set")

        # If using Gemini and no custom base URL is provided, use Google's Gemini API base URL
        if not base_url and settings.GEMINI_API_KEY:
            # Google's Gemini API base URL
            base_url = "https://generativelanguage.googleapis.com/v1beta/"

        # If using a custom base URL, we'll use that
        # If no base URL is set and we're not using Gemini, use OpenAI's default
        if not base_url and settings.OPENAI_API_KEY and not settings.GEMINI_API_KEY:
            # Use OpenAI's default base URL
            base_url = "https://api.openai.com/v1"

        # Create AsyncOpenAI client without proxy-related environment variable issues
        # Use a custom httpx client to avoid proxy conflicts
        import httpx
        http_client = httpx.AsyncClient(
            base_url=base_url,
            headers={"Authorization": f"Bearer {key}"}
        )

        self.openai_client = AsyncOpenAI(
            api_key=key,
            base_url=base_url,
            http_client=http_client
        )

        # Create the model wrapper as specified by the user
        self.model = OpenAIChatCompletionsModel(
            openai_client=self.openai_client,
            model=settings.MODEL_NAME  # Use the configured model name (defaults to gemini-1.5-flash)
        )

        # Initialize retrieval service (to be injected or imported)
        self.retrieval_service = None

    def set_retrieval_service(self, retrieval_service):
        """Set the retrieval service dependency"""
        self.retrieval_service = retrieval_service

    async def process_query(
        self,
        query_request: RAGQueryRequest
    ) -> tuple[GeneratedResponse, RetrievedContext]:
        """
        Process a user query through the RAG pipeline:
        1. Retrieve relevant context
        2. Generate response based on context
        3. Return both response and context
        """
        try:
            self.logger.info(f"Processing query: {query_request.query[:50]}...")

            # Sanitize inputs to prevent security vulnerabilities
            sanitized_query = self._sanitize_input(query_request.query)
            sanitized_selected_text = self._sanitize_input(query_request.selected_text) if query_request.selected_text else None

            # Step 1: Retrieve relevant context
            if not self.retrieval_service:
                raise RetrievalError("Retrieval service not configured")

            # Determine what to use for retrieval based on context_mode
            retrieval_query = sanitized_query
            if query_request.context_mode == "selected_text" and sanitized_selected_text:
                # When context_mode is selected_text, prioritize the selected text in the query
                retrieval_query = f"Based on this selected text: '{sanitized_selected_text}', answer this question: {sanitized_query}"

            # Create a QueryRequest object for the retrieval service
            from src.models.query import QueryRequest
            query_request_obj = QueryRequest(
                query=retrieval_query,
                top_k=query_request.top_k if hasattr(query_request, 'top_k') else 5,
                score_threshold=query_request.threshold if hasattr(query_request, 'threshold') else settings.RETRIEVAL_THRESHOLD
            )
            query_result = await self.retrieval_service.retrieve(query_request_obj)

            # Convert QueryResult to RetrievedContext
            from src.models.retrieved_context import RetrievedContext
            from src.models.context_chunk import ContextChunk
            from datetime import datetime
            import uuid

            # Convert ContentChunk objects to ContextChunk objects
            context_chunks = []
            confidence_scores = []

            for content_chunk in query_result.results:
                # Extract source information from metadata
                source_document = content_chunk.metadata.get('source', 'unknown') if hasattr(content_chunk, 'metadata') else 'unknown'

                # Use the id field from ContentChunk as chunk_id
                chunk_id = content_chunk.id if hasattr(content_chunk, 'id') else 'unknown'

                # Use the score field from ContentChunk as confidence_score
                confidence_score = content_chunk.score if hasattr(content_chunk, 'score') else 0.0

                # Extract additional metadata fields if they exist
                page_number = content_chunk.metadata.get('page_number') if hasattr(content_chunk, 'metadata') and isinstance(content_chunk.metadata, dict) else None
                section_title = content_chunk.metadata.get('section_title') if hasattr(content_chunk, 'metadata') and isinstance(content_chunk.metadata, dict) else None

                # Convert ContentChunk to ContextChunk
                context_chunk = ContextChunk(
                    chunk_id=chunk_id,
                    content=content_chunk.content,
                    source_document=source_document,
                    page_number=page_number,
                    section_title=section_title,
                    confidence_score=confidence_score
                )
                context_chunks.append(context_chunk)
                confidence_scores.append(confidence_score)

            retrieved_context = RetrievedContext(
                query_id=str(uuid.uuid4()),
                chunks=context_chunks,
                confidence_scores=confidence_scores,
                retrieval_timestamp=datetime.utcnow(),
                id=str(uuid.uuid4())
            )

            # Step 1.5: Check if we have any context to work with
            if not retrieved_context.chunks or len(retrieved_context.chunks) == 0:
                # Handle the case where no relevant content was found
                # First check if this is a simple greeting that we can respond to generically
                fallback_response = await self._handle_empty_retrieval(query_request)
                self.logger.info(f"Handled query with no relevant context, response ID: {fallback_response.id}")
                return fallback_response, retrieved_context

            # Step 2: Generate response based on retrieved context
            generated_response = await self._generate_response(
                query_request,
                retrieved_context
            )

            self.logger.info(f"Successfully processed query, response ID: {generated_response.id}")
            return generated_response, retrieved_context

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            # Return a safe fallback response instead of raising an error
            fallback_response = GeneratedResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                content="I'm sorry, but I encountered an error while processing your request. Please try again later.",
                sources=[],
                generation_timestamp=datetime.utcnow(),
                confidence_level="insufficient_data",
                metadata={
                    "model_name": settings.MODEL_NAME,
                    "error": str(e),
                    "retrieval_chunks_count": 0
                }
            )
            # Create a placeholder chunk to satisfy the min_items=1 requirement
            placeholder_chunk = ContextChunk(
                chunk_id="error-context",
                content="No context available due to processing error.",
                source_document="none",
                page_number=None,
                section_title="Error Context",
                confidence_score=0.0
            )

            empty_context = RetrievedContext(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                chunks=[placeholder_chunk],
                confidence_scores=[0.0],
                retrieval_timestamp=datetime.utcnow(),
                metadata={"error": str(e)}
            )
            return fallback_response, empty_context

    async def _generate_response(
        self,
        query_request: RAGQueryRequest,
        retrieved_context: RetrievedContext
    ) -> GeneratedResponse:
        """
        Generate a response using the configured model based on the retrieved context.
        """
        try:
            # Format the context for the model
            context_text = self._format_context_for_model(retrieved_context)

            # Create the prompt with the query, context, and selected text if available
            prompt = self._create_prompt(query_request.query, context_text, query_request.selected_text)

            # Generate response using the model wrapper (following the exact pattern)
            response_text = await self.model.generate_content(
                prompt,
                temperature=settings.TEMPERATURE,
                max_tokens=settings.MAX_TOKENS,
                timeout=settings.TIMEOUT_SECONDS
            )

            if not response_text or not response_text.strip():
                response_text = "I couldn't find relevant information to answer your query."

            # Determine confidence level based on retrieved context quality
            confidence_level = self._determine_confidence_level(retrieved_context)

            # Create and return the generated response object
            generated_response = GeneratedResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),  # Generate a query ID for tracking
                content=response_text,
                sources=[chunk.chunk_id for chunk in retrieved_context.chunks],
                generation_timestamp=datetime.utcnow(),
                confidence_level=confidence_level,
                metadata={
                    "model_name": settings.MODEL_NAME,
                    "temperature": settings.TEMPERATURE,
                    "max_tokens": settings.MAX_TOKENS,
                    "retrieval_chunks_count": len(retrieved_context.chunks)
                }
            )

            return generated_response

        except Exception as e:
            self.logger.error(f"Error generating response: {str(e)}")
            # Return a fallback response instead of raising an error
            return GeneratedResponse(
                id=str(uuid.uuid4()),
                query_id=str(uuid.uuid4()),
                content="I'm sorry, but I encountered an error while processing your request. Please try again later.",
                sources=[],
                generation_timestamp=datetime.utcnow(),
                confidence_level="insufficient_data",
                metadata={
                    "model_name": settings.MODEL_NAME,
                    "error": str(e),
                    "retrieval_chunks_count": len(retrieved_context.chunks)
                }
            )

    def _format_context_for_model(self, retrieved_context: RetrievedContext) -> str:
        """
        Format the retrieved context for input to the language model.
        """
        if not retrieved_context.chunks:
            return "No relevant information found in the book content.\n\n"

        formatted_context = "Relevant information from the book:\n\n"

        for i, chunk in enumerate(retrieved_context.chunks):
            formatted_context += f"[Source {i+1}] ({chunk.source_document}):\n"
            formatted_context += f"  Content: {chunk.content}\n"
            formatted_context += f"  Confidence: {chunk.confidence_score}\n\n"

        return formatted_context

    def _create_prompt(self, query: str, context: str, selected_text: Optional[str] = None) -> str:
        """
        Create a prompt that includes the context and query for the language model.
        """
        if selected_text:
            # When selected text is provided, emphasize that the answer should be about the selected content
            prompt = f"""
            {context}

            The user has selected the following text and is asking a question about it:
            Selected Text: {selected_text}

            Based on the information provided in the context above, please answer the user's question about the selected text:
            {query}

            Instructions:
            - Only use information provided in the context above
            - If the context doesn't contain enough information to answer the question, respond with "I don't have enough information from the provided sources to answer this question."
            - Do not make up information or hallucinate
            - When providing an answer, clearly cite which sources you used by referencing them as [Source X] where X is the source number
            - Structure your response with:
              1. The answer to the question
              2. A "Sources:" section that lists the specific sources used
            """
        else:
            # Standard prompt without selected text context
            prompt = f"""
            {context}

            Based on the information provided above, please answer the following question:
            {query}

            Instructions:
            - Only use information provided in the context above
            - If the context doesn't contain enough information to answer the question, respond with "I don't have enough information from the provided sources to answer this question."
            - Do not make up information or hallucinate
            - When providing an answer, clearly cite which sources you used by referencing them as [Source X] where X is the source number
            - Structure your response with:
              1. The answer to the question
              2. A "Sources:" section that lists the specific sources used
            """

        return prompt.strip()

    def _determine_confidence_level(self, retrieved_context: RetrievedContext) -> str:
        """
        Determine the confidence level based on the quality of retrieved context.
        """
        if not retrieved_context.chunks or len(retrieved_context.chunks) == 0:
            return "insufficient_data"

        avg_confidence = sum(retrieved_context.confidence_scores) / len(retrieved_context.confidence_scores)

        if avg_confidence >= 0.7:
            return "high"
        elif avg_confidence >= 0.4:
            return "medium"
        else:
            return "low"

    async def _handle_empty_retrieval(self, query_request: RAGQueryRequest) -> GeneratedResponse:
        """
        Handle the case where no relevant context is retrieved for a query.
        For simple greetings, respond with a friendly message instead of insufficient data message.
        """
        from src.utils.response_templates import ResponseTemplates
        import re

        # Check if the query is a simple greeting
        query_lower = query_request.query.lower().strip()
        greeting_patterns = [
            r'^hello', r'^hi', r'^hey', r'^greetings', r'^good morning',
            r'^good afternoon', r'^good evening', r'^good day', r'how are you',
            r'howdy', r'what\'s up', r'how\'s it going'
        ]

        is_greeting = any(re.match(pattern, query_lower) for pattern in greeting_patterns)

        if is_greeting:
            # For greetings, provide a friendly response
            greeting_response = "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics course. How can I help you today?"
            content = greeting_response
        else:
            # For non-greetings, use the standard insufficient data response
            content = ResponseTemplates.get_insufficient_data_response("no_relevant_content")

        return GeneratedResponse(
            id=str(uuid.uuid4()),
            query_id=str(uuid.uuid4()),
            content=content,
            sources=[],
            generation_timestamp=datetime.utcnow(),
            confidence_level="insufficient_data" if not is_greeting else "medium",  # Higher confidence for greetings
            metadata={
                "model_name": settings.MODEL_NAME,
                "retrieval_chunks_count": 0,
                "reason": "no_relevant_content_found" if not is_greeting else "greeting_response"
            }
        )

    async def validate_response_grounding(
        self,
        response: str,
        context_chunks: List[ContextChunk]
    ) -> bool:
        """
        Validate that the generated response is grounded in the provided context.
        """
        # This is a basic implementation - in a production system, you might want
        # more sophisticated validation techniques
        response_lower = response.lower()

        # Check if key terms from context appear in the response
        for chunk in context_chunks:
            chunk_content_lower = chunk.content.lower()
            # Check if at least some content from the chunk appears in the response
            # This is a simplified check - real implementation would need more sophisticated NLP
            if len(chunk_content_lower) > 10:  # Only check substantial chunks
                sample_words = chunk_content_lower[:100]  # Take first 100 chars as sample
                if len(sample_words.split()) > 5:  # If it's at least 5 words
                    sample_term = sample_words.split()[0]  # Take first word as sample
                    if sample_term in response_lower:
                        return True

        # If no clear grounding found, return False
        # In a real system, we'd want more sophisticated validation
        return len(context_chunks) == 0  # If no context, then it's valid to return False

    def _sanitize_input(self, text: Optional[str]) -> Optional[str]:
        """
        Sanitize input text to prevent security vulnerabilities.
        """
        if not text:
            return text

        # Remove potentially dangerous content
        sanitized = text

        # Remove script tags and other potentially dangerous HTML
        sanitized = sanitized.replace('<script', '&lt;script').replace('</script>', '&lt;/script>')

        # Remove javascript: and other dangerous protocols
        dangerous_protocols = ['javascript:', 'data:', 'vbscript:', 'file:', 'ftp:']
        for protocol in dangerous_protocols:
            sanitized = sanitized.replace(protocol, f'blocked_{protocol}')

        # Remove potential SQL injection patterns
        dangerous_sql = ['--', '/*', '*/', 'xp_', 'sp_', 'exec', 'execute', 'select', 'insert', 'update', 'delete']
        for sql_pattern in dangerous_sql:
            # Only replace if it's a complete word (not part of another word)
            import re
            sanitized = re.sub(rf'\b{re.escape(sql_pattern)}\b', f'[{sql_pattern}_blocked]', sanitized, flags=re.IGNORECASE)

        return sanitized