# Quickstart Guide: RAG Agent Development

## Overview
This guide provides instructions for setting up and running the RAG (Retrieval-Augmented Generation) agent that integrates with book content using a language model.

## Prerequisites
- Python 3.11 or higher
- pip package manager
- Access to Google Gemini API (API key)
- Access to the validated retrieval pipeline from Spec 2

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r backend/requirements.txt
```

### 4. Environment Configuration
Create a `.env` file in the backend directory with the following variables:

```env
GEMINI_API_KEY=your_gemini_api_key_here
RETRIEVAL_SERVICE_URL=http://localhost:8001  # URL of the retrieval service from Spec 2
LOG_LEVEL=INFO
DEBUG=False
```

## Running the Service

### 1. Start the RAG Agent Service
```bash
cd backend
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Verify the Service is Running
Open your browser or use curl to check the health endpoint:
```bash
curl http://localhost:8000/v1/health
```

## Making Requests

### Submit a Query
```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-api-key" \
  -d '{
    "query": "What are the key principles of physical AI?",
    "user_id": "user-123"
  }'
```

### Expected Response
```json
{
  "id": "resp-abc123",
  "query": "What are the key principles of physical AI?",
  "response": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
  "confidence_level": "high",
  "sources": [
    {
      "chunk_id": "chunk-xyz789",
      "content": "Physical AI is characterized by the integration of perception, action, and learning in embodied systems...",
      "source_document": "physical_ai_chapter_3.pdf",
      "page_number": 45,
      "section_title": "Embodied Intelligence Principles",
      "confidence_score": 0.87
    }
  ],
  "timestamp": "2025-12-18T10:30:00Z",
  "metadata": {
    "retrieval_time_ms": 250,
    "generation_time_ms": 1200
  }
}
```

## Configuration Options

### Environment Variables
- `GEMINI_API_KEY`: Your Google Gemini API key (required)
- `RETRIEVAL_SERVICE_URL`: URL of the retrieval service from Spec 2 (required)
- `MODEL_NAME`: Name of the language model to use (default: "gemini-pro")
- `TEMPERATURE`: Temperature parameter for response generation (default: 0.7)
- `MAX_TOKENS`: Maximum tokens in generated response (default: 2048)
- `RETRIEVAL_THRESHOLD`: Minimum confidence score for retrieved chunks (default: 0.5)
- `TIMEOUT_SECONDS`: Timeout for API calls in seconds (default: 30)
- `ENABLE_CITATIONS`: Whether to enable citation generation (default: true)

## Testing

### Run Unit Tests
```bash
cd backend
python -m pytest tests/unit/
```

### Run Integration Tests
```bash
cd backend
python -m pytest tests/integration/
```

## Architecture Overview

The RAG agent follows this workflow:
1. Receive user query via API endpoint
2. Query the retrieval service to get relevant book content chunks
3. Format the retrieved context for the language model
4. Generate a response using the language model
5. Validate that the response is grounded in the provided context
6. Return the response with source citations

## Troubleshooting

### Common Issues
- **API Key Error**: Ensure GEMINI_API_KEY is set correctly in environment variables
- **Retrieval Service Unavailable**: Check that the retrieval service from Spec 2 is running
- **Slow Responses**: Check network connectivity and API rate limits

### Logging
The service logs to stdout with configurable log levels. Check logs for detailed error information.