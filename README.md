# Physical AI & Humanoid Robotics Coursebook

An AI-Native Interactive Coursebook with RAG Chatbot for learning Physical AI and Humanoid Robotics.

## Features

- **Interactive Course Content**: Comprehensive chapters on Physical AI and Humanoid Robotics
- **RAG Chatbot**: AI assistant that can answer questions based on course materials
- **Interactive Quizzes**: Test your knowledge with built-in quizzes
- **Progress Tracking**: Track your learning progress through the course
- **Note Taking**: Add personal notes to each section
- **Responsive Design**: Works on desktop, tablet, and mobile devices
- **Dark/Light Mode**: Choose your preferred theme

## Tech Stack

- **Frontend**: Docusaurus v3.1.0 with React
- **Backend**: FastAPI with Python
- **Database**: (To be implemented)
- **AI/ML**: (RAG system to be connected)

## Project Structure

```
hackathon/
├── backend/
│   ├── main.py                 # FastAPI application
│   ├── requirements.txt        # Python dependencies
│   ├── requirements-dev.txt    # Development dependencies
│   └── src/                    # Backend source code
├── frontend/
│   ├── docs/                   # Course content (markdown files)
│   ├── src/
│   │   ├── components/         # React components
│   │   ├── pages/              # Docusaurus pages
│   │   ├── services/           # API services
│   │   ├── contexts/           # React contexts
│   │   ├── css/                # Global styles
│   │   └── theme/              # Docusaurus theme overrides
│   ├── package.json            # Frontend dependencies
│   ├── docusaurus.config.js    # Docusaurus configuration
│   └── sidebars.js             # Navigation configuration
└── README.md                   # This file
```

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.8 or higher)
- pip (Python package manager)

## Installation

### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```

4. Start the backend server:
   ```bash
   python -m uvicorn main:app --reload
   ```
   The backend will be available at `http://localhost:8000`

### Frontend Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install Node.js dependencies:
   ```bash
   npm install
   ```

3. Start the frontend development server:
   ```bash
   npm run start
   ```
   The frontend will be available at `http://localhost:3000`

## API Endpoints

The backend provides the following endpoints:

- `GET /` - Welcome message
- `GET /health` - Health check
- `POST /chat` - Chat with the RAG bot
- `POST /retrieve` - Retrieve relevant content chunks for a query using semantic search
- `POST /validate` - Validate retrieval pipeline with test queries
- `POST /load-course-content` - Load course content from markdown files into vector database
- `POST /load-sitemap-content` - Load content from sitemap URL into vector database
- `GET /courses/{courseId}` - Get course content
- `GET /chapters/{chapterId}` - Get chapter content
- `POST /quizzes/{quizId}/submit` - Submit quiz answers
- `POST /notes` - Save user notes
- `GET /notes` - Get user notes
- `POST /progress` - Update progress
- `GET /progress` - Get progress
- `POST /plagiarism-check` - Check content for plagiarism
- `GET /content-ingestion/status` - Check content ingestion service status
- `POST /content-ingestion/process-site` - Process content from GitHub Pages site
- `POST /content-ingestion/test-connection` - Test connection to Cohere and Qdrant services

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# API Configuration
API_HOST=0.0.0.0
API_PORT=8000

# Database Configuration (to be implemented)
DATABASE_URL=sqlite:///./coursebook.db

# AI/ML Configuration (to be implemented)
OPENAI_API_KEY=your_openai_api_key_here
VECTOR_DB_URL=your_vector_db_url_here
```

## Running the Application

To run both servers simultaneously:

1. Terminal 1 - Start the backend:
   ```bash
   cd backend && python -m uvicorn main:app --reload
   ```

2. Terminal 2 - Start the frontend:
   ```bash
   cd frontend && npm run start
   ```

## Retrieval and Validation Usage

The system includes advanced retrieval and validation capabilities:

### Content Retrieval
To retrieve relevant content chunks for a query, send a POST request to `/retrieve`:

```bash
curl -X POST http://localhost:8000/retrieve \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the principles of humanoid robotics?",
    "top_k": 5,
    "score_threshold": 0.1
  }'
```

### Pipeline Validation
To validate the retrieval pipeline, send a POST request to `/validate`:

```bash
curl -X POST http://localhost:8000/validate \
  -H "Content-Type: application/json" \
  -d '{
    "query": {
      "query": "What is machine learning?",
      "top_k": 3
    },
    "test_type": "automated",
    "validation_criteria": {
      "min_relevance_score": 0.5,
      "max_response_time_ms": 2000
    }
  }'
```

### Loading Course Content
Before using the retrieval system, load course content:

```bash
curl -X POST http://localhost:8000/load-course-content
```

## Development

### Adding New Content

To add new course content:

1. Create a new markdown file in `frontend/docs/chapters/` or `frontend/docs/advanced/`
2. Update `frontend/sidebars.js` to include the new content in the navigation
3. Restart the frontend server to see changes

### Adding New Components

Custom React components are located in `frontend/src/components/`. To create a new component:

1. Create a new `.jsx` file in the components directory
2. Create a corresponding `.module.css` file for styling
3. Export the component from the file
4. Import and use the component in your pages

### API Integration

API services are located in `frontend/src/services/api.js`. To add new API endpoints:

1. Add the new method to the `ApiService` class
2. Create a helper function for the new endpoint
3. Use the new function in your components

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.