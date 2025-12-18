from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.routes import search
from src.api.routes import chat
from src.lib.logging_config import setup_logging
import uvicorn


# Setup logging
setup_logging()

# Create FastAPI app
app = FastAPI(
    title="Physical AI Book Search API",
    description="API for semantic search of Physical AI Book content using Qdrant vector database",
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

# Include API routes
app.include_router(search.router, prefix="/api/v1", tags=["search"])
app.include_router(chat.router, prefix="/api/v1", tags=["chat"])

# Add health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy"}


if __name__ == "__main__":
    uvicorn.run("src.api.main:app", host="0.0.0.0", port=8000, reload=True)