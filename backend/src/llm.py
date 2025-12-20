import os
from typing import List, Dict, Any
from openai import AsyncOpenAI
from .config import settings


class GeminiLLM:
    def __init__(self):
        # Initialize the OpenAI client with Gemini-compatible endpoint
        # Don't check for API key here, check it when the method is called
        self.model_name = "gemini-1.5-flash"

    async def generate_response(self, query: str, context: List[Dict[str, Any]] = None) -> str:
        """Generate a response using the LLM with optional context"""
        if not settings.gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        # Initialize the OpenAI client with Gemini-compatible endpoint
        client = AsyncOpenAI(
            api_key=settings.gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )

        if context:
            # Format the context for the prompt
            context_str = "\n\n".join([doc["content"] for doc in context])
            messages = [
                {
                    "role": "system",
                    "content": "You are an AI assistant for a Physical AI & Humanoid Robotics coursebook. Provide helpful, accurate, and educational responses."
                },
                {
                    "role": "user",
                    "content": f"""
                    You are an AI assistant for a Physical AI & Humanoid Robotics coursebook.
                    Answer the user's question based on the provided course materials.

                    Course Materials:
                    {context_str}

                    User Question: {query}

                    Please provide a helpful and accurate answer based on the course materials.
                    If the information is not available in the provided context, say so clearly.
                    """
                }
            ]
        else:
            messages = [
                {
                    "role": "system",
                    "content": "You are an AI assistant for a Physical AI & Humanoid Robotics coursebook. Provide helpful, accurate, and educational responses."
                },
                {
                    "role": "user",
                    "content": f"""
                    You are an AI assistant for a Physical AI & Humanoid Robotics coursebook.
                    Answer the user's question: {query}

                    Provide a helpful response based on general knowledge of robotics and AI.
                    """
                }
            ]

        try:
            response = await client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                temperature=0.3,
                max_tokens=1000
            )
            return response.choices[0].message.content
        except Exception as e:
            # Return a fallback response if API call fails
            if context:
                context_str = "\n\n".join([doc["content"][:200] + "..." for doc in context])
                return f"Based on the course materials, I found this information related to your question '{query}':\n\n{context_str}\n\nThis is a simplified response because there was an issue with the AI model: {str(e)}"
            else:
                return f"I received your message: '{query}'. I couldn't find specific information in the course materials, but I'm here to help with questions about Physical AI & Humanoid Robotics. Error: {str(e)}"


# Note: We can't create a singleton instance here because __init__ doesn't support async
# The instance will be created in main.py when needed