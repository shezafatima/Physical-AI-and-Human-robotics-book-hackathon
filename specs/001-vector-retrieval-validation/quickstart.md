# Quickstart: Vector Retrieval Pipeline and Data Validation for RAG

## Overview
This guide helps you get started with the semantic retrieval pipeline that queries Qdrant using user questions, retrieves relevant embedded book content, and validates the pipeline's correctness and relevance.

## Prerequisites
- Python 3.13 or higher
- Cohere API key
- Qdrant Cloud instance access
- Backend service running (port 8000)

## Setup

### 1. Environment Configuration
```bash
# Set up environment variables
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_URL="your-qdrant-cloud-url"
export QDRANT_API_KEY="your-qdrant-api-key"
```

### 2. Install Dependencies
```bash
pip install cohere qdrant-client fastapi uvicorn pydantic python-dotenv
```

## Basic Usage

### 1. Query for Relevant Content
```bash
curl -X POST http://localhost:8000/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the principles of humanoid robotics?",
    "top_k": 5
  }'
```

### 2. Validate Retrieval Pipeline
```bash
curl -X POST http://localhost:8000/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": {
      "query": "What are the key concepts in physical AI?",
      "top_k": 5
    },
    "test_type": "automated",
    "validation_criteria": {
      "min_relevance_score": 0.7,
      "max_response_time_ms": 2000
    }
  }'
```

## Key Endpoints

### `/retrieve` (POST)
- **Purpose**: Retrieve relevant content chunks for a query
- **Request**: Query text with optional parameters (top_k, score_threshold)
- **Response**: Content chunks with metadata and relevance scores

### `/validate` (POST)
- **Purpose**: Validate the retrieval pipeline with test queries
- **Request**: Validation test configuration
- **Response**: Validation results with pass/fail status

## Validation Process

### 1. Automated Validation
- Runs queries with known expected results
- Measures response time and relevance scores
- Validates against performance criteria

### 2. Manual Validation
- Human assessment of semantic relevance
- Quality checks on retrieved content
- Verification that content matches query intent

## Troubleshooting

### Common Issues
- **Cohere API errors**: Verify COHERE_API_KEY is set correctly
- **Qdrant connection failures**: Check QDRANT_URL and QDRANT_API_KEY
- **Slow response times**: May indicate large vector database or network latency

### Performance Tips
- Use appropriate top_k values (5-10 for most queries)
- Set score_threshold to filter low-quality results
- Monitor response times to ensure <2 second performance goal