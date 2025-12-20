# Data Model: Vector Retrieval Pipeline and Data Validation for RAG

## Entities

### Query
**Description**: A natural language text input from the user that needs to be semantically matched to content in the vector database

**Fields**:
- query_text: string (required) - The user's natural language question or search query
- top_k: integer (optional, default: 5) - Number of top results to retrieve (min: 1, max: 100)
- filters: object (optional) - Additional filters for the search query

**Validation Rules**:
- query_text must be between 1 and 1000 characters
- top_k must be between 1 and 100
- query_text cannot be empty or contain only whitespace

### Embedding
**Description**: A high-dimensional vector representation of text that enables semantic similarity comparison

**Fields**:
- vector: array of floats (required) - The embedding vector (1024 dimensions for Cohere embed-english-v3.0)
- model: string (required) - The model used to generate the embedding (e.g., "embed-english-v3.0")
- input_type: string (required) - The type of input used ("search_query" for queries, "search_document" for stored content)

**Validation Rules**:
- vector must have exactly 1024 dimensions for Cohere model
- model must be a supported embedding model
- input_type must be one of the allowed values

### Content Chunk
**Description**: A segment of book content that has been processed and stored in the vector database with associated metadata

**Fields**:
- id: string (required) - Unique identifier for the content chunk
- content: string (required) - The actual text content of the chunk
- metadata: object (required) - Additional information about the content
  - source_url: string - URL where the content originated
  - title: string - Title of the content section
  - chapter: string - Chapter or section identifier
  - created_at: datetime - Timestamp when the chunk was created
- embedding: Embedding (required) - The vector representation of the content

**Validation Rules**:
- content must not be empty
- metadata.source_url must be a valid URL
- embedding must be a valid embedding object

### Retrieval Result
**Description**: A set of content chunks with metadata and relevance scores returned in response to a user query

**Fields**:
- results: array of Content Chunk (required) - The retrieved content chunks
- query_embedding: Embedding (required) - The embedding of the original query
- search_params: object (required) - Parameters used for the search
  - top_k: integer - Number of results requested
  - score_threshold: float - Minimum relevance score (optional)
- response_time_ms: integer (required) - Time taken to process the query in milliseconds
- total_results: integer (required) - Total number of results returned

**Validation Rules**:
- results array must contain at least 1 item (when results exist)
- response_time_ms must be non-negative
- total_results must match the actual count of results in the array

### Validation Test
**Description**: A test case used to validate the retrieval pipeline with expected outcomes

**Fields**:
- id: string (required) - Unique identifier for the test
- query: Query (required) - The test query to execute
- expected_results: array of Content Chunk (optional) - Expected results for validation
- test_type: string (required) - Type of test ("automated" or "manual")
- validation_criteria: object (required) - Criteria for determining if test passed
  - min_relevance_score: float - Minimum acceptable relevance score (default: 0.7)
  - max_response_time_ms: integer - Maximum acceptable response time (default: 2000)

**Validation Rules**:
- test_type must be either "automated" or "manual"
- min_relevance_score must be between 0 and 1
- max_response_time_ms must be positive

## Relationships

- A Query generates an Embedding through the embedding process
- An Embedding is used to search against Content Chunks in Qdrant
- Content Chunks are retrieved to form a Retrieval Result
- Validation Tests use Queries to validate the Retrieval Results

## State Transitions

### Query Processing Flow
1. Query received → Query validated → Embedding generated
2. Embedding → Qdrant search → Content Chunks retrieved
3. Content Chunks → Result formatting → Retrieval Result returned

### Validation Flow
1. Validation Test created → Query executed → Retrieval Result obtained
2. Retrieval Result → Criteria evaluation → Test result (pass/fail)