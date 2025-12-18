# Quickstart Guide: Data Retrieval from Qdrant Vector Database

**Feature**: Data Retrieval from Qdrant Vector Database
**Date**: 2025-12-17
**Branch**: 001-data-retrieval-from-qdrant

## Overview

This quickstart guide provides the essential information needed to set up, run, and test the Qdrant-based semantic search service for Physical AI Book content.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Qdrant vector database with pre-indexed Physical AI Book content
- Cohere API key for embedding generation
- Docker (optional, for containerized deployment)

## Setup Instructions

### 1. Environment Setup

```bash
# Clone the repository
git clone <repository-url>
cd <repository-directory>

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configuration

Create a `.env` file in the backend root directory:

```env
QDRANT_HOST=qdrant.example.com
QDRANT_PORT=6333
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_COLLECTION_NAME=physical_ai_book_content
EMBEDDING_MODEL_NAME=embed-multilingual-v2.0
SIMILARITY_THRESHOLD=0.7
MAX_RESULTS=10
```

### 3. Install Dependencies

```bash
cd backend
pip install fastapi uvicorn qdrant-client cohere pydantic python-dotenv
```

## Running the Service

### Development Mode

```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Production Mode

```bash
cd backend
uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

### Using Docker

```bash
# Build the image
docker build -t physical-ai-search .

# Run the container
docker run -p 8000:8000 -e QDRANT_HOST=... -e COHERE_API_KEY=... physical-ai-search
```

## API Usage Examples

### Search for Content

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "query": "How do I implement a PID controller for robot motor control?",
    "similarity_threshold": 0.7,
    "top_k": 5
  }'
```

### Search with Filters

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "query": "sensor fusion techniques",
    "filters": {
      "content_types": ["lesson", "exercise"]
    },
    "similarity_threshold": 0.6,
    "top_k": 10
  }'
```

## Testing

### Unit Tests

```bash
cd backend
python -m pytest tests/unit/
```

### Integration Tests

```bash
cd backend
python -m pytest tests/integration/
```

### Contract Tests

```bash
cd backend
python -m pytest tests/contract/
```

## Key Endpoints

- `POST /search` - Perform semantic search on Physical AI Book content
  - Request: JSON with query, filters, and search parameters
  - Response: Ranked list of content chunks with metadata

## Configuration Options

- `QDRANT_HOST`: Hostname of the Qdrant server
- `QDRANT_PORT`: Port of the Qdrant server (default: 6333)
- `QDRANT_API_KEY`: API key for Qdrant authentication
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_COLLECTION_NAME`: Name of the collection containing Physical AI Book content
- `SIMILARITY_THRESHOLD`: Minimum similarity score for results (0.0-1.0)
- `MAX_RESULTS`: Maximum number of results to return per query

## Troubleshooting

### Common Issues

1. **Connection to Qdrant fails**: Verify QDRANT_HOST and QDRANT_API_KEY are correctly set
2. **Embedding generation fails**: Check COHERE_API_KEY is valid and has sufficient quota
3. **No results returned**: Try lowering the similarity threshold
4. **Slow response times**: Verify Qdrant database is properly indexed

### Health Check

```bash
curl http://localhost:8000/health
```

Expected response: `{"status": "healthy", "timestamp": "2025-12-17T10:00:00Z"}`