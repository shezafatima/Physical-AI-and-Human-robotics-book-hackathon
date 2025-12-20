# Data Model: Content Ingestion, Embedding Generation, and Vector Storage for RAG

## ContentSegment

Represents a chunk of extracted text from the source documentation.

**Fields**:
- `id`: Unique identifier for the segment
- `url`: Original URL where the content was found
- `content`: The actual text content of the segment
- `title`: Title or heading associated with this content
- `section_hierarchy`: Path/structure information showing where in the documentation this content appears
- `metadata`: Additional metadata about the content (word count, creation date, etc.)

**Validation rules**:
- Content must not be empty
- URL must be a valid format
- Content length should be between 100-1000 words for optimal embedding

## EmbeddingVector

High-dimensional vector representation of content segment, linked to source content and searchable in vector space.

**Fields**:
- `id`: Unique identifier for the embedding
- `vector`: The actual embedding vector (array of floats)
- `segment_id`: Reference to the ContentSegment this embedding represents
- `model`: Name/version of the embedding model used
- `created_at`: Timestamp when the embedding was generated

**Validation rules**:
- Vector must have consistent dimensions
- segment_id must reference an existing ContentSegment

## DocumentMetadata

Information about source document including URL, last modified timestamp, content type, and relevance indicators.

**Fields**:
- `url`: The source URL of the document
- `title`: Title of the document
- `last_modified`: Timestamp of when the document was last modified
- `content_type`: Type of content (e.g., "text/html", "markdown")
- `size`: Size of the content in bytes
- `status`: Status of the extraction (e.g., "success", "failed", "pending")
- `crawl_timestamp`: When the document was crawled

**Validation rules**:
- URL must be valid
- Status must be one of the defined values
- Size must be a positive integer