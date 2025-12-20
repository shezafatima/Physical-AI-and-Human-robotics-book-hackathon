import openai
from typing import Optional, List, Dict, Any
from src.config.settings import settings
from src.utils.exceptions import APIConnectionError, ConfigurationError
from src.utils.logger import setup_logger
import asyncio


class OpenAIClient:
    """
    Client for interacting with the OpenAI API.
    Provides methods for generating content, embeddings, and other OpenAI features.
    """

    def __init__(self, api_key: Optional[str] = None):
        self.logger = setup_logger("openai_client")

        # Use provided API key or get from settings
        self.api_key = api_key or settings.OPENAI_API_KEY

        if not self.api_key:
            raise ConfigurationError("OPENAI_API_KEY is required to initialize OpenAIClient")

        # Configure the OpenAI client
        openai.api_key = self.api_key

        # Set default generation parameters
        self.default_model = settings.MODEL_NAME
        self.default_temperature = settings.TEMPERATURE
        self.default_max_tokens = settings.MAX_TOKENS
        self.default_timeout = settings.TIMEOUT_SECONDS

    async def generate_content(
        self,
        prompt: str,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        timeout: Optional[int] = None,
        model: Optional[str] = None
    ) -> str:
        """
        Generate content using the OpenAI model asynchronously.

        Args:
            prompt: The input prompt for content generation
            temperature: Controls randomness (0.0-1.0), higher is more random
            max_tokens: Maximum number of tokens to generate
            timeout: Request timeout in seconds
            model: Model to use (defaults to configured model)

        Returns:
            Generated content as a string
        """
        try:
            # Use defaults if not provided
            temp = temperature if temperature is not None else self.default_temperature
            max_tok = max_tokens if max_tokens is not None else self.default_max_tokens
            timeout_val = timeout if timeout is not None else self.default_timeout
            model_name = model if model is not None else self.default_model

            # Validate parameters
            if not 0.0 <= temp <= 1.0:
                raise ValueError(f"Temperature must be between 0.0 and 1.0, got {temp}")
            if not 1 <= max_tok <= 4096:
                raise ValueError(f"Max tokens must be between 1 and 4096, got {max_tok}")
            if timeout_val <= 0:
                raise ValueError(f"Timeout must be positive, got {timeout_val}")

            # Prepare the messages for chat completion
            messages = [
                {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Only use information from the context provided, and do not make up information. If the context doesn't contain enough information, say so."},
                {"role": "user", "content": prompt}
            ]

            # Generate content using OpenAI ChatCompletion API
            response = await openai.AsyncOpenAI(api_key=self.api_key).chat.completions.create(
                model=model_name,
                messages=messages,
                temperature=temp,
                max_tokens=max_tok,
                timeout=timeout_val
            )

            # Extract text from response
            content = response.choices[0].message.content
            if content:
                return content
            else:
                self.logger.warning("No content returned from OpenAI API")
                return ""

        except Exception as e:
            self.logger.error(f"Error generating content with OpenAI: {str(e)}")
            raise APIConnectionError(f"Failed to generate content with OpenAI API: {str(e)}")

    async def embed_content(
        self,
        content: str,
        model: str = "text-embedding-ada-002"
    ) -> List[float]:
        """
        Generate embeddings for the given content.

        Args:
            content: Text to generate embeddings for
            model: Embedding model to use

        Returns:
            List of embedding values
        """
        try:
            response = await openai.AsyncOpenAI(api_key=self.api_key).embeddings.create(
                input=content,
                model=model
            )
            return response.data[0].embedding
        except Exception as e:
            self.logger.error(f"Error generating embeddings with OpenAI: {str(e)}")
            raise APIConnectionError(f"Failed to generate embeddings with OpenAI API: {str(e)}")

    async def check_connection(self) -> bool:
        """
        Check if the OpenAI API connection is working.

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
            self.logger.error(f"OpenAI API connection test failed: {str(e)}")
            return False

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the configured model.

        Returns:
            Dictionary with model information
        """
        return {
            "model_name": self.default_model,
            "temperature": self.default_temperature,
            "max_tokens": self.default_max_tokens,
            "timeout": self.default_timeout
        }

    async def batch_generate_content(
        self,
        prompts: List[str],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        model: Optional[str] = None
    ) -> List[str]:
        """
        Generate content for multiple prompts in a batch.

        Args:
            prompts: List of input prompts
            temperature: Controls randomness (0.0-1.0)
            max_tokens: Maximum number of tokens to generate per prompt
            model: Model to use (defaults to configured model)

        Returns:
            List of generated content strings
        """
        results = []
        for prompt in prompts:
            try:
                content = await self.generate_content(prompt, temperature, max_tokens, model=model)
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
            response = await openai.AsyncOpenAI(api_key=self.api_key).chat.completions.create(
                model=self.default_model,
                messages=[{"role": "user", "content": "test"}],
                temperature=0.1,
                max_tokens=5,
                timeout=10
            )

            # If we get a response, the API key is valid
            return True
        except Exception as e:
            self.logger.error(f"API key validation failed: {str(e)}")
            return False