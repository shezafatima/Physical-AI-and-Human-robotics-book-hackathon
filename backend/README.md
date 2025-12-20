# RAG Agent API Backend

This backend service implements a Retrieval-Augmented Generation (RAG) agent that processes user queries using book content and generates grounded responses using the Gemini language model.

## Features

- Query processing with RAG pipeline
- Integration with validated retrieval pipeline
- Gemini model integration via API key
- Grounded responses based on retrieved book content
- Proper citation and source attribution
- Error handling for low-confidence retrieval results

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create environment file:
   ```bash
   cp .env.example .env
   ```

3. Update `.env` with your API keys and configuration

4. Start the server:
   ```bash
   uvicorn main:app --reload
   ```

## API Endpoints

- `POST /v1/query` - Submit a query to the RAG agent
- `GET /v1/health` - Health check endpoint
- Other existing endpoints from the coursebook API

## Environment Variables

- `GEMINI_API_KEY` - Your Google Gemini API key
- `COHERE_API_KEY` - Your Cohere API key (for embeddings)
- `QDRANT_URL` - URL for Qdrant vector database
- `QDRANT_API_KEY` - API key for Qdrant
- `HOST` - Server host (default: 0.0.0.0)
- `PORT` - Server port (default: 8000)
- `MODEL_NAME` - Name of the language model to use
- `TEMPERATURE` - Temperature parameter for response generation
- `MAX_TOKENS` - Maximum tokens in generated response
- `RETRIEVAL_THRESHOLD` - Minimum confidence score for retrieved chunks
- `TIMEOUT_SECONDS` - Timeout for API calls
- `ENABLE_CITATIONS` - Whether to enable citation generation


## API Usage Examples

### Query the RAG Agent

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of physical AI?",
    "user_id": "user-123"
  }'
```

Example response:
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

### Health Check

```bash
curl http://localhost:8000/v1/health
```

### Gemini API Health Check

```bash
curl http://localhost:8000/v1/gemini-health
```