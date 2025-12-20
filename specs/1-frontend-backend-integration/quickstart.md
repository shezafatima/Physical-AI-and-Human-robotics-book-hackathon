# Quickstart Guide: Frontend-Backend Integration for Embedded RAG Chatbot

**Feature**: 1-frontend-backend-integration
**Created**: 2025-12-18

## Overview

This guide provides instructions for setting up and running the frontend-backend integration for the RAG chatbot. The integration connects a Docusaurus-based frontend with a FastAPI backend to enable users to ask questions about book content and receive AI-generated responses.

## Prerequisites

- Python 3.8+ installed
- Node.js 16+ installed
- Access to API keys (GEMINI_API_KEY, etc.) in environment variables
- Git for version control

## Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Set up environment variables**:
   Create a `.env` file in the backend directory with required API keys:
   ```
   GEMINI_API_KEY=your-gemini-api-key
   BASE_URL=https://generativelanguage.googleapis.com/v1beta/
   # Add other required environment variables
   ```

4. **Run the backend server**:
   ```bash
   uvicorn main:app --reload --port 8000
   ```

## Frontend Setup

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install Node.js dependencies**:
   ```bash
   npm install
   ```

3. **Run the Docusaurus development server**:
   ```bash
   npm run start
   ```

## API Usage

### Chat Endpoint
- **URL**: `POST /v1/chat`
- **Description**: Process user query and return RAG-generated response

**Example Request**:
```json
{
  "message": "What are the key principles of physical AI?",
  "selected_text": "Embodied cognition is the theory that cognitive processes are deeply rooted in the body's interactions with the world.",
  "context_mode": "selected_text"
}
```

**Example Response**:
```json
{
  "response": "The key principles of physical AI include embodied cognition, sensorimotor learning, and adaptive control systems...",
  "sources": ["Chapter 3: Embodied Cognition", "Chapter 5: Sensorimotor Learning"],
  "confidence": "high",
  "status": "success",
  "context": {
    "retrieved_docs_count": 3,
    "source_docs": ["doc1", "doc2", "doc3"],
    "confidence_scores": [0.8, 0.7, 0.6]
  },
  "timestamp": "2025-12-18T10:30:00Z"
}
```

## Frontend Integration

The chatbot component should be integrated into the Docusaurus theme. The component will:
1. Capture user queries from the UI
2. Optionally capture selected text on the page
3. Send requests to the backend API
4. Display responses with appropriate loading and error states
5. Maintain session history within the current page visit

## Testing the Integration

1. Start both backend and frontend servers
2. Navigate to the frontend in your browser
3. Use the embedded chatbot to ask questions about the book content
4. Verify that responses are generated and displayed correctly
5. Test with selected text context to ensure proper functionality
6. Verify error handling works when the backend is unavailable

## Troubleshooting

**Backend not responding**:
- Verify the backend server is running on port 8000
- Check that all required environment variables are set
- Confirm API keys are valid and have necessary permissions

**CORS errors**:
- Verify that the backend allows requests from the frontend origin
- Check the CORS middleware configuration in main.py

**No responses generated**:
- Verify that the RAG agent is properly configured
- Check that the vector database contains the book content
- Confirm the retrieval service is functioning correctly