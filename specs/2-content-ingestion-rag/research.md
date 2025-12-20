# Research Document: Content Ingestion, Embedding Generation, and Vector Storage for RAG

## Decision: Python Project Setup with uv

**Rationale**: The user specifically requested to use uv for initializing the Python project. uv is a fast Python package installer and resolver that provides faster dependency installation than pip.

**Alternatives considered**:
- pip + venv (traditional approach)
- poetry (dependency management with lock files)
- conda (for data science projects)

## Decision: Dependencies for Web Crawling

**Rationale**: For fetching and cleaning text from deployed book URLs, we'll use:
- `requests` for HTTP requests
- `beautifulsoup4` for HTML parsing and content extraction
- `lxml` as a parser for BeautifulSoup (faster than html.parser)

**Alternatives considered**:
- `scrapy` (full-featured web scraping framework - overkill for this task)
- `selenium` (for JavaScript-heavy sites - not needed for static GitHub Pages)
- `playwright` (modern browser automation - unnecessary complexity)

## Decision: Cohere Embedding Model Integration

**Rationale**: Using Cohere's embedding models via their Python client library (`cohere`) provides reliable semantic embeddings with good performance and documentation.

**Alternatives considered**:
- OpenAI embeddings (different pricing model, user specifically requested Cohere)
- Sentence Transformers (local models, higher resource usage)
- Hugging Face transformers (local processing, requires more setup)

## Decision: Qdrant Cloud Integration

**Rationale**: Qdrant Cloud provides managed vector database service with good performance and reliability for the RAG system.

**Alternatives considered**:
- Local Qdrant (requires self-management)
- Pinecone (different managed service)
- Weaviate (alternative vector database)

## Decision: Text Chunking Strategy

**Rationale**: For content chunking, we'll use a sliding window approach with overlap to preserve context across chunks while maintaining semantic meaning. Chunk size will be around 500-1000 words as specified in the success criteria.

**Alternatives considered**:
- Sentence-based chunking (may create very small chunks)
- Paragraph-based chunking (may create very large chunks)
- Character-based chunking (less semantic meaning)

## Decision: URL Discovery Method

**Rationale**: For discovering all URLs on a GitHub Pages site, we'll use a combination of sitemap.xml parsing and HTML link extraction to ensure comprehensive coverage of the Docusaurus-based site.

**Alternatives considered**:
- Robot.txt parsing (not always available or comprehensive)
- API-based discovery (not available for static sites)
- Manual URL list (not automated as required)

## Decision: Error Handling and Rate Limiting

**Rationale**: Implement proper error handling for network requests and rate limiting for Cohere API calls to ensure robust operation and compliance with API quotas.

**Alternatives considered**:
- No rate limiting (would violate API terms and cause failures)
- Simple retry logic (insufficient for production use)