# Physical AI & Humanoid Robotics Course Chatbot - Setup Instructions

## Issue Description
The chatbot is not returning answers based on the book content. When asking questions about the course material, it responds with "I don't have enough information from the provided sources to answer this question."

## Root Cause
The book content from the `frontend/docs/` directory was not properly loaded into the Qdrant vector database. The RAG system is functional, but the vector database lacks the course content needed to answer queries.

## Solution
The content loading script (`backend/load_book_content.py`) has been created to load the book content from `frontend/docs/` into the vector database. However, due to Cohere API rate limits (40 calls per minute for trial keys), the loading process must be done in batches with appropriate delays.

## How to Load Content

1. **Ensure environment variables are set** in `backend/.env`:
   - `COHERE_API_KEY` - Your Cohere API key
   - `QDRANT_URL` - Your Qdrant database URL
   - `QDRANT_API_KEY` - Your Qdrant API key

2. **Run the content loader**:
   ```bash
   cd backend
   python load_book_content.py
   ```

   Note: This process will take time due to API rate limits. The script processes content in batches of 30 documents with 1-minute pauses between batches to stay under the rate limit.

3. **Verify content loading**:
   - The script will show progress as it processes files
   - After completion, it will show the total number of vectors in the collection
   - You can test the API endpoint to confirm content is available

## Testing the Fix
After content loading completes, test the API:

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "user_id": "test-user",
    "metadata": {
      "context_mode": "full_content",
      "source": "frontend"
    }
  }'
```

## Troubleshooting
- If you see rate limit errors, wait for the API limits to reset before continuing
- Check that all environment variables are properly set
- Verify that the Qdrant database is accessible
- The script will continue from where it left off if interrupted

## Expected Behavior
After content loading is complete, the chatbot should:
- Return relevant answers based on the book content
- Cite sources from the course materials
- Show confidence scores based on the retrieved context