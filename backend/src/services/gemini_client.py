import google.generativeai as genai
from typing import Optional, List, Dict, Any
from src.config.settings import settings
from src.utils.exceptions import APIConnectionError, ConfigurationError
from src.utils.logger import setup_logger
import asyncio


class GeminiClient:
    """
    Client for interacting with the Google Gemini API.
    Provides methods for generating content, embedding, and other Gemini features.
    """

    def __init__(self, api_key: Optional[str] = None):
        self.logger = setup_logger("gemini_client")

        # Use provided API key or get from settings
        self.api_key = api_key or settings.GEMINI_API_KEY

        if not self.api_key:
            raise ConfigurationError("GEMINI_API_KEY is required to initialize GeminiClient")

        # Configure the API
        genai.configure(api_key=self.api_key)

        # Initialize the model
        self.model_name = settings.MODEL_NAME
        self.model = genai.GenerativeModel(self.model_name)

        # Set default generation parameters
        self.default_temperature = settings.TEMPERATURE
        self.default_max_tokens = settings.MAX_TOKENS
        self.default_timeout = settings.TIMEOUT_SECONDS

    async def generate_content(
        self,
        prompt: str,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        timeout: Optional[int] = None
    ) -> str:
        """
        Generate content using the Gemini model asynchronously.

        Args:
            prompt: The input prompt for content generation
            temperature: Controls randomness (0.0-1.0), higher is more random
            max_tokens: Maximum number of tokens to generate
            timeout: Request timeout in seconds

        Returns:
            Generated content as a string
        """
        try:
            # Use defaults if not provided
            temp = temperature if temperature is not None else self.default_temperature
            max_tok = max_tokens if max_tokens is not None else self.default_max_tokens
            timeout_val = timeout if timeout is not None else self.default_timeout

            # Validate parameters
            if not 0.0 <= temp <= 1.0:
                raise ValueError(f"Temperature must be between 0.0 and 1.0, got {temp}")
            if not 1 <= max_tok <= 4096:
                raise ValueError(f"Max tokens must be between 1 and 4096, got {max_tok}")
            if timeout_val <= 0:
                raise ValueError(f"Timeout must be positive, got {timeout_val}")

            # Generate content
            response = await self.model.generate_content_async(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=temp,
                    max_output_tokens=max_tok,
                    response_mime_type="text/plain"
                ),
                request_options={"timeout": timeout_val}
            )

            # Extract text from response
            if response.candidates and response.candidates[0].content.parts:
                return response.candidates[0].content.parts[0].text
            else:
                self.logger.warning("No content returned from Gemini API")
                return ""

        except Exception as e:
            self.logger.error(f"Error generating content with Gemini: {str(e)}")
            raise APIConnectionError(f"Failed to generate content with Gemini API: {str(e)}")

    async def embed_content(
        self,
        content: str,
        task_type: str = "RETRIEVAL_QUERY"
    ) -> List[float]:
        """
        Generate embeddings for the given content.

        Args:
            content: Text to generate embeddings for
            task_type: Type of task (RETRIEVAL_QUERY, RETRIEVAL_DOCUMENT, etc.)

        Returns:
            List of embedding values
        """
        try:
            result = genai.embed_content(
                model="models/embedding-001",
                content=content,
                task_type=task_type
            )
            return result['embedding']
        except Exception as e:
            self.logger.error(f"Error generating embeddings with Gemini: {str(e)}")
            raise APIConnectionError(f"Failed to generate embeddings with Gemini API: {str(e)}")

    async def check_connection(self) -> bool:
        """
        Check if the Gemini API connection is working.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try a simple generation to test the connection
            test_response = await self.generate_content(
                "Hello, are you there?",
                temperature=0.1,
                max_tokens=10,
                timeout=10
            )

            # If we get a response, connection is working
            return bool(test_response and len(test_response.strip()) > 0)
        except Exception as e:
            self.logger.error(f"Gemini API connection test failed: {str(e)}")
            return False

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the configured model.

        Returns:
            Dictionary with model information
        """
        return {
            "model_name": self.model_name,
            "temperature": self.default_temperature,
            "max_tokens": self.default_max_tokens,
            "timeout": self.default_timeout
        }

    async def batch_generate_content(
        self,
        prompts: List[str],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None
    ) -> List[str]:
        """
        Generate content for multiple prompts in a batch.

        Args:
            prompts: List of input prompts
            temperature: Controls randomness (0.0-1.0)
            max_tokens: Maximum number of tokens to generate per prompt

        Returns:
            List of generated content strings
        """
        results = []
        for prompt in prompts:
            try:
                content = await self.generate_content(prompt, temperature, max_tokens)
                results.append(content)
            except Exception as e:
                self.logger.error(f"Error generating content for prompt '{prompt[:50]}...': {str(e)}")
                results.append("")  # Add empty string as fallback

        return results

    async def validate_api_key(self) -> bool:
        """
        Validate the API key by attempting a simple operation.

        Returns:
            True if API key is valid, False otherwise
        """
        try:
            # Attempt a simple generation to validate the API key
            response = await self.model.generate_content_async(
                "test",
                generation_config=genai.types.GenerationConfig(
                    temperature=0.1,
                    max_output_tokens=5
                ),
                request_options={"timeout": 10}
            )

            # If we get a response, the API key is valid
            return True
        except Exception as e:
            self.logger.error(f"API key validation failed: {str(e)}")
            return False