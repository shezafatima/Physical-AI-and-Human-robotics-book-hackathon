from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.v1.router import router
from src.api.middleware import RequestLoggingMiddleware
from src.config.settings import settings

# Create FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="A Retrieval-Augmented Generation API for processing queries against book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add custom middleware
app.add_middleware(
    RequestLoggingMiddleware,
    logger_name="api"
)

# Include API routes with v1 prefix
app.include_router(router, prefix="/v1")

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "RAG Agent API is running!",
        "version": "1.0.0",
        "endpoints": [
            "/v1/query - Submit a query to the RAG agent",
            "/v1/health - Health check endpoint",
            "/v1/gemini-health - Gemini API connectivity health check"
        ]
    }

@app.get("/health")
async def health_check():
    """General health check endpoint"""
    return {
        "status": "healthy",
        "message": "RAG Agent API is running",
        "model": settings.MODEL_NAME
    }

# This allows the app to be run with uvicorn
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.RELOAD
    )