# Quickstart Guide: Content Ingestion, Embedding Generation, and Vector Storage for RAG

## Prerequisites

- Python 3.11 or higher
- uv package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Initialize the project**:
   ```bash
   mkdir backend
   cd backend
   uv init
   ```

2. **Install dependencies**:
   ```bash
   uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv lxml
   ```

3. **Create environment file**:
   ```bash
   touch .env
   ```

4. **Add environment variables to .env**:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Implementation Steps

1. **Create main.py** with the required functions:
   - `get_all_urls()` - Discover all accessible URLs from the GitHub Pages site
   - `extract_text_from_url()` - Extract clean text content from a given URL
   - `chunk_text()` - Split text into appropriately sized chunks
   - `embed()` - Generate embeddings using Cohere
   - `create_collection()` - Create the "rag_embedding" collection in Qdrant
   - `save_chunk_to_qdrant()` - Store embeddings and metadata in Qdrant

2. **Run the main function** to execute the full pipeline:
   - Fetch all URLs from the target site: https://shezafatima.github.io/Physical-AI-and-Human-robotics-book-hackathon/
   - Extract content from each URL
   - Chunk the content
   - Generate embeddings
   - Store in Qdrant vector database

## Running the Application

```bash
cd backend
python main.py
```

## Expected Output

The application should:
- Discover all pages on the GitHub Pages site
- Extract and clean the text content
- Generate embeddings for each content chunk
- Store the embeddings in the Qdrant "rag_embedding" collection
- Provide confirmation of successful processing