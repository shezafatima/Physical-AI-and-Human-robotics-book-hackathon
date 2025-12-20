from typing import Optional, List, Dict, Any
from openai import AsyncOpenAI
from src.config.settings import settings
from src.utils.exceptions import APIConnectionError, ConfigurationError
from src.utils.logger import setup_logger
import asyncio
import json


class OpenAIChatCompletionsModel:
    """
    A wrapper model that uses OpenAI SDK to interface with Gemini API
    following the user's specified configuration pattern.
    """

    def __init__(self, openai_client: AsyncOpenAI, model: str = "gemini-1.5-flash"):
        self.logger = setup_logger("openai_chat_completions_model")
        self.openai_client = openai_client
        self.model = model

    async def generate_content(
        self,
        prompt: str,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        timeout: Optional[int] = None
    ) -> str:
        """
        Generate content using the configured model through OpenAI SDK.

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
            temp = temperature if temperature is not None else settings.TEMPERATURE
            max_tok = max_tokens if max_tokens is not None else settings.MAX_TOKENS
            timeout_val = timeout if timeout is not None else settings.TIMEOUT_SECONDS

            # Validate parameters
            if not 0.0 <= temp <= 1.0:
                raise ValueError(f"Temperature must be between 0.0 and 1.0, got {temp}")
            if not 1 <= max_tok <= 4096:
                raise ValueError(f"Max tokens must be between 1 and 4096, got {max_tok}")
            if timeout_val <= 0:
                raise ValueError(f"Timeout must be positive, got {timeout_val}")

            # Check if we're using the Google API base URL (for Gemini)
            # If so, we need to format the request appropriately
            if "generativelanguage.googleapis.com" in str(self.openai_client.base_url):
                # For Google's Gemini API, we need to format the model name differently
                # Google's API expects "models/gemini-1.5-flash" format
                if not self.model.startswith("models/"):
                    gemini_model = f"models/{self.model}"
                else:
                    gemini_model = self.model

                # For Gemini API, we'll use a direct API call
                # We need to extract the API key from the client
                api_key = self.openai_client.api_key
                import httpx

                # Construct the Gemini API URL
                gemini_url = f"https://generativelanguage.googleapis.com/v1beta/{gemini_model}:generateContent?key={api_key}"

                # Prepare the request payload for Gemini
                gemini_payload = {
                    "contents": [{
                        "parts": [{
                            "text": prompt
                        }]
                    }],
                    "generationConfig": {
                        "temperature": temp,
                        "maxOutputTokens": max_tok,
                        "stopSequences": []
                    }
                }

                # Make the API call using httpx
                async with httpx.AsyncClient(timeout=timeout_val) as client:
                    response = await client.post(
                        gemini_url,
                        json=gemini_payload,
                        headers={"Content-Type": "application/json"}
                    )

                    if response.status_code != 200:
                        raise APIConnectionError(f"Gemini API returned status {response.status_code}: {response.text}")

                    response_data = response.json()

                    # Extract the text from the response
                    if "candidates" in response_data and len(response_data["candidates"]) > 0:
                        candidate = response_data["candidates"][0]
                        if "content" in candidate and "parts" in candidate["content"] and len(candidate["content"]["parts"]) > 0:
                            part = candidate["content"]["parts"][0]
                            if "text" in part:
                                return part["text"]

                    self.logger.warning("No content returned from Gemini API")
                    return ""
            else:
                # Use standard OpenAI API format
                # Prepare the messages for chat completion
                messages = [
                    {"role": "system", "content": "You are a helpful assistant that answers questions based on provided context. Only use information from the context provided, and do not make up information. If the context doesn't contain enough information, say so."},
                    {"role": "user", "content": prompt}
                ]

                # Generate content using the OpenAI client
                response = await self.openai_client.chat.completions.create(
                    model=self.model,
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
                    self.logger.warning("No content returned from API")
                    return ""

        except Exception as e:
            self.logger.error(f"Error generating content: {str(e)}")
            raise APIConnectionError(f"Failed to generate content: {str(e)}")