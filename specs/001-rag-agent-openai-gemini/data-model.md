# Data Model: RAG Agent Development with Language Model Integration

## Overview
This document defines the data models for the RAG agent system, including the entities, their attributes, relationships, and validation rules derived from the feature specification.

## Entity: Query
**Description**: User input requesting information from book content, containing the question or information request

**Attributes**:
- `id`: String (UUID) - Unique identifier for the query
- `content`: String (required) - The actual query text from the user
- `timestamp`: DateTime - When the query was submitted
- `user_id`: String (optional) - Identifier for the user making the query
- `metadata`: JSON (optional) - Additional query metadata

**Validation Rules**:
- `content` must be between 1 and 1000 characters
- `content` must not be empty or contain only whitespace
- `timestamp` must be in ISO 8601 format

## Entity: RetrievedContext
**Description**: Relevant book passages and chunks extracted from the validated retrieval pipeline that inform the response

**Attributes**:
- `id`: String (UUID) - Unique identifier for the context set
- `query_id`: String (UUID) - Reference to the original query
- `chunks`: Array of ContextChunk objects - Retrieved text chunks
- `confidence_scores`: Array of Float - Confidence scores for each chunk
- `retrieval_timestamp`: DateTime - When the retrieval was performed
- `metadata`: JSON (optional) - Additional retrieval metadata

**Validation Rules**:
- `chunks` array must contain at least 1 and at most 10 items
- Each confidence score must be between 0.0 and 1.0
- `query_id` must reference a valid Query entity

## Entity: ContextChunk
**Description**: Individual text chunk from the book content that is relevant to the query

**Attributes**:
- `chunk_id`: String - Identifier for the specific chunk
- `content`: String (required) - The actual text content of the chunk
- `source_document`: String - Reference to the source document
- `page_number`: Integer (optional) - Page number in the source document
- `section_title`: String (optional) - Title of the section containing the chunk
- `confidence_score`: Float - Confidence score for this chunk's relevance

**Validation Rules**:
- `content` must be between 10 and 5000 characters
- `confidence_score` must be between 0.0 and 1.0
- `chunk_id` must be unique within a RetrievedContext

## Entity: GeneratedResponse
**Description**: The final output provided to the user, grounded in the retrieved context with proper attribution

**Attributes**:
- `id`: String (UUID) - Unique identifier for the response
- `query_id`: String (UUID) - Reference to the original query
- `content`: String (required) - The generated response text
- `sources`: Array of String - References to source chunks used
- `generation_timestamp`: DateTime - When the response was generated
- `confidence_level`: String - Confidence level (high, medium, low, insufficient_data)
- `metadata`: JSON (optional) - Additional generation metadata

**Validation Rules**:
- `content` must be between 10 and 10000 characters
- `query_id` must reference a valid Query entity
- `confidence_level` must be one of: "high", "medium", "low", "insufficient_data"
- `sources` must reference valid ContextChunk entities from the same query session

## Entity: AgentConfiguration
**Description**: Settings including API keys, model parameters, and retrieval thresholds that control agent behavior

**Attributes**:
- `api_key`: String (required) - Language model API key
- `model_name`: String - Name of the language model to use
- `temperature`: Float - Temperature parameter for response generation (0.0-1.0)
- `max_tokens`: Integer - Maximum tokens in generated response
- `retrieval_threshold`: Float - Minimum confidence score for retrieved chunks (0.0-1.0)
- `timeout_seconds`: Integer - Timeout for API calls
- `enable_citations`: Boolean - Whether to enable citation generation

**Validation Rules**:
- `api_key` must be provided and not empty
- `temperature` must be between 0.0 and 1.0
- `max_tokens` must be between 1 and 4096
- `retrieval_threshold` must be between 0.0 and 1.0
- `timeout_seconds` must be between 1 and 120

## Relationships

```
Query (1) <---> (1) GeneratedResponse
Query (1) <---> (1) RetrievedContext
RetrievedContext (1) <---> (*) ContextChunk
GeneratedResponse (1) --> (*) ContextChunk (via sources)
```

## State Transitions

### Query Lifecycle
1. **Created**: Query is received by the system
2. **Processing**: Retrieval is in progress
3. **ResponseGenerated**: Response has been generated
4. **Completed**: Response has been returned to user

### Response Confidence Levels
- **High**: Multiple high-confidence chunks were used, response is well-supported
- **Medium**: Some relevant chunks were used but with moderate confidence
- **Low**: Few or low-confidence chunks were used
- **Insufficient Data**: No relevant chunks found, response indicates lack of information