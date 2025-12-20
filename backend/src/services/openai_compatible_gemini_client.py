import google.generativeai as genai
from typing import Optional, List, Dict, Any, Union
from src.config.settings import settings
from src.utils.exceptions import APIConnectionError, ConfigurationError
from src.utils.logger import setup_logger
import asyncio


class OpenAICompatibleGeminiClient:
    """
    A client that uses Google's Generative AI (Gemini) but implements an OpenAI-like interface.
    This allows using the OpenAI SDK patterns while working with Gemini models.
    """

    def __init__(self, api_key: Optional[str] = None):
        self.logger = setup_logger("openai_compatible_gemini_client")

        # Use provided API key or get from settings
        self.api_key = api_key or settings.GEMINI_API_KEY or settings.OPENAI_API_KEY

        if not self.api_key:
            raise ConfigurationError("GEMINI_API_KEY or OPENAI_API_KEY is required to initialize OpenAICompatibleGeminiClient")

        # Configure the Google Generative AI client
        genai.configure(api_key=self.api_key)

        # Initialize the model
        self.model_name = settings.MODEL_NAME  # Will default to gpt-4-turbo but use gemini
        # Override to use a Gemini model if it's not already configured as one
        if not self.model_name.startswith("gemini"):
            self.model_name = "gemini-pro"  # Use Gemini model instead

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
        Generate content using the Gemini model with OpenAI-like interface.

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

            # Generate content using the Gemini model
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

    async def chat_completions_create(
        self,
        messages: List[Dict[str, str]],
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        timeout: Optional[int] = None
    ) -> str:
        """
        Simulate OpenAI's chat.completions.create method using Gemini.
        This method formats chat messages for the Gemini model.

        Args:
            messages: List of messages in the format [{"role": "role", "content": "content"}]
            temperature: Controls randomness (0.0-1.0)
            max_tokens: Maximum number of tokens to generate
            timeout: Request timeout in seconds

        Returns:
            Generated content as a string
        """
        try:
            # Combine messages into a single prompt for Gemini
            # Gemini doesn't natively support chat format like OpenAI, so we'll format it
            formatted_prompt = ""
            for message in messages:
                role = message.get("role", "user")
                content = message.get("content", "")
                formatted_prompt += f"{role.capitalize()}: {content}\n"

            # Add instruction for the AI to respond appropriately
            formatted_prompt += "\nAssistant: "

            # Use the generate_content method
            return await self.generate_content(
                formatted_prompt,
                temperature=temperature,
                max_tokens=max_tokens,
                timeout=timeout
            )
        except Exception as e:
            self.logger.error(f"Error in chat_completions_create: {str(e)}")
            raise APIConnectionError(f"Failed to generate chat completion: {str(e)}")

    async def embed_content(
        self,
        content: str,
        task_type: str = "RETRIEVAL_QUERY"
    ) -> List[float]:
        """
        Generate embeddings for the given content using Gemini's embedding capabilities.

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