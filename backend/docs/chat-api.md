# Chat API Documentation

## Overview

The Chat API provides a RESTful interface for interacting with the RAG (Retrieval-Augmented Generation) chatbot. It allows users to ask questions about course content and receive AI-generated responses based on the available documentation.

## Base URL

```
http://localhost:8000 (development)
https://api.example.com (production)
```

## Authentication

The API does not require authentication for basic functionality. However, the backend may require API keys for the LLM service (e.g., Gemini or OpenAI) which are configured server-side.

## Endpoints

### POST /chat

Process a user query and return a RAG-generated response.

#### Request

**Content-Type**: `application/json`

**Body**:
```json
{
  "message": "string (required) - The user's question",
  "selected_text": "string (optional) - Text selected by the user for context",
  "context_mode": "enum (required) - How to process the query ('full_content' or 'selected_text')",
  "session_id": "string (optional) - Session identifier for maintaining conversation context"
}
```

**Example Request**:
```json
{
  "message": "What are the key principles of physical AI?",
  "selected_text": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
  "context_mode": "selected_text",
  "session_id": "sess-abc123"
}
```

#### Response

**Success Response (200 OK)**:
```json
{
  "response": "string - The AI-generated answer to the user's query",
  "sources": "array of strings - Citations to source documents used in the response",
  "confidence": "enum - Confidence level of the response ('high', 'medium', 'low', 'insufficient_data')",
  "status": "enum - Processing status ('success', 'error', 'timeout')",
  "context": {
    "retrieved_docs_count": "integer - Number of documents retrieved for context",
    "source_docs": "array of strings - List of source documents used",
    "confidence_scores": "array of numbers - Confidence scores for retrieved documents",
    "generation_metadata": "object - Additional metadata about the generation process"
  },
  "timestamp": "string - ISO 8601 timestamp of response generation"
}
```

**Example Success Response**:
```json
{
  "response": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
  "sources": ["Chapter 3: Embodied Cognition", "Chapter 5: Sensorimotor Learning"],
  "confidence": "high",
  "status": "success",
  "context": {
    "retrieved_docs_count": 3,
    "source_docs": ["doc1", "doc2", "doc3"],
    "confidence_scores": [0.8, 0.7, 0.6],
    "generation_metadata": {
      "model_name": "gemini-1.5-flash",
      "temperature": 0.7,
      "max_tokens": 1000,
      "retrieval_chunks_count": 3
    }
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

**Error Response (400, 408, 500)**:
```json
{
  "response": "string - Error message describing what went wrong",
  "sources": [],
  "confidence": "insufficient_data",
  "status": "error",
  "context": {},
  "timestamp": "string - ISO 8601 timestamp of error"
}
```

#### Error Codes

- **400 Bad Request**: Invalid request format or missing required fields
- **408 Request Timeout**: Request took too long to process
- **500 Internal Server Error**: Backend processing error occurred

## Usage Examples

### Basic Question

Ask a general question about the course content:

```json
{
  "message": "What is embodied cognition?",
  "selected_text": null,
  "context_mode": "full_content",
  "session_id": "sess-12345"
}
```

### Context-Aware Question

Ask a question about specific selected text:

```json
{
  "message": "Can you explain this concept further?",
  "selected_text": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
  "context_mode": "selected_text",
  "session_id": "sess-12345"
}
```

## Best Practices

1. **Context Modes**:
   - Use `full_content` for general questions about the course material
   - Use `selected_text` when the user has highlighted specific content they want to ask about

2. **Session Management**:
   - Include a `session_id` to maintain conversation context across multiple requests
   - Generate a unique session ID for each new conversation

3. **Error Handling**:
   - Implement retry logic for failed requests
   - Handle different error statuses appropriately in the UI
   - Provide user-friendly error messages

4. **Selected Text**:
   - Capture selected text when users highlight content on the page
   - Sanitize selected text to prevent security vulnerabilities
   - Limit selected text length to prevent overly long requests

## Security Considerations

- Input sanitization is performed server-side to prevent injection attacks
- Selected text is sanitized before processing
- The API implements rate limiting (not shown in this basic implementation)
- All requests should be made over HTTPS in production