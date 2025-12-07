from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# Initialize the FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Coursebook API",
    description="API for the AI-Native Interactive Coursebook with RAG Chatbot",
    version="0.1.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics Coursebook API",
        "version": "0.1.0"
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)