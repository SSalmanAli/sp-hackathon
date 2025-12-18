# Physical AI Book Search Service

This backend service provides semantic search functionality for the Physical AI Book content using Qdrant vector database and Cohere embeddings.

## Overview

The search service enables users to find relevant Physical AI Book content using natural language queries. It implements semantic search by converting queries to vector embeddings and finding similar content in the Qdrant vector database.

## Architecture

The service follows a clean architecture with distinct layers:

- **Models**: Data validation and serialization using Pydantic
- **Services**: Business logic for embedding generation, Qdrant search, and retrieval orchestration
- **API**: FastAPI endpoints with proper error handling and validation
- **Lib**: Configuration, logging, and exception handling

## Features

1. **Semantic Search** (User Story 1): Search for content using natural language queries
2. **Content Filtering** (User Story 2): Filter results by content type (module, lesson, section, exercise)
3. **Complete Metadata** (User Story 3): All results include module, lesson, section, and chunk identifiers

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url  # Optional, defaults to localhost
   QDRANT_HOST=localhost       # Default for local Qdrant
   QDRANT_PORT=6333           # Default Qdrant port
   QDRANT_API_KEY=your_qdrant_api_key  # Optional
   QDRANT_COLLECTION_NAME=physical_ai_book_content
   ```

## Running the Service

```bash
# Development mode
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000

# Or run the main file directly
python src/api/main.py
```

## API Usage

### Search Endpoint

```
POST /api/v1/search
```

**Headers:**
- `X-API-Key`: Your API key for authentication

**Request Body:**
```json
{
  "query": "How do I implement a PID controller?",
  "content_types": ["lesson", "exercise"],
  "similarity_threshold": 0.7,
  "top_k": 5
}
```

**Response:**
```json
{
  "query": "How do I implement a PID controller?",
  "results": [
    {
      "content_chunk": {
        "chunk_id": "chunk-001",
        "module_id": "module-001",
        "lesson_id": "lesson-003",
        "section_id": "section-002",
        "content": "PID controller implementation for robot motor control...",
        "metadata": {}
      },
      "similarity_score": 0.92,
      "metadata": {
        "module": "module-001",
        "lesson": "lesson-003",
        "section": "section-002",
        "chunk": "chunk-001"
      },
      "relevance_rank": 1
    }
  ],
  "total_results": 1,
  "execution_time_ms": 45.2
}
```

## Configuration

The service can be configured via environment variables in the `.env` file:

- `QDRANT_HOST`: Qdrant server hostname (default: localhost)
- `QDRANT_PORT`: Qdrant server port (default: 6333)
- `QDRANT_API_KEY`: Qdrant API key (optional)
- `QDRANT_COLLECTION_NAME`: Name of the collection containing Physical AI Book content (default: physical_ai_book_content)
- `COHERE_API_KEY`: Cohere API key (required)
- `EMBEDDING_MODEL_NAME`: Name of the Cohere embedding model (default: embed-multilingual-v2.0)
- `SIMILARITY_THRESHOLD`: Default similarity threshold (default: 0.7)
- `MAX_RESULTS`: Maximum number of results to return (default: 10)

## Error Handling

The service implements comprehensive error handling:

- Query validation errors return 400 Bad Request
- Qdrant connection errors return 500 Internal Server Error
- General exceptions are caught and return appropriate error responses

## Testing

Run the tests to verify the implementation:

```bash
# Run basic functionality tests
python test_basic_functionality.py

# Run final implementation tests
python test_final_implementation.py
```

## Compliance with Constitutional Principles

This implementation adheres to the Physical AI Book constitution:

- **Data Retrieval as First-Class Capability**: The service is focused solely on semantic search functionality
- **Separation from LLM Generation**: No prompt construction or LLM generation is performed
- **Separation from UI Logic**: The service is a pure API without UI concerns
- **Structured Accuracy**: Results include complete metadata for RAG system compatibility
- **Embodied AI Focus**: Content retrieval is specific to Physical AI topics