# Quickstart Guide: AI Answering Agent for Physical AI Book

**Feature**: AI Answering Agent for Physical AI Book
**Date**: 2025-12-17
**Branch**: 001-ai-answering-agent

## Overview

This quickstart guide provides the essential information needed to set up, run, and test the AI Answering Agent for the Physical AI Book chatbot.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to OpenAI-compatible API with Gemini-compatible key
- Access to Content Extraction tool (from Qdrant retrieval system)
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
OPENAI_API_KEY=your_openai_gemini_compatible_api_key
CONTENT_EXTRACTION_ENDPOINT=http://localhost:8000/api/v1/search  # Endpoint for content extraction tool
CONTENT_EXTRACTION_API_KEY=your_content_extraction_api_key
MODEL_NAME=gpt-4-turbo  # Or compatible model
TEMPERATURE=0.3  # Low temperature for deterministic responses
MAX_TOKENS=2000
TIMEOUT_SECONDS=30
SYSTEM_PROMPT="You are an AI assistant for the Physical AI Book. Answer questions based only on the provided context. If the information is not in the context, explicitly state that the information is not available in the book."
```

### 3. Install Dependencies

```bash
cd backend
pip install openai fastapi uvicorn pydantic python-dotenv
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
docker build -t physical-ai-agent .

# Run the container
docker run -p 8000:8000 -e OPENAI_API_KEY=... -e CONTENT_EXTRACTION_API_KEY=... physical-ai-agent
```

## API Usage Examples

### Submit a Query

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "query": "How does a PID controller work in robotics?",
    "user_context": {
      "conversation_history": [],
      "user_preferences": {}
    }
  }'
```

### Example Response

```json
{
  "query": "How does a PID controller work in robotics?",
  "response": "A PID (Proportional-Integral-Derivative) controller in robotics is a control loop mechanism that calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error by adjusting the process control inputs. The PID controller uses three terms: Proportional (P) which reduces the rise time and reduces, but never eliminates, the steady-state error; Integral (I) which eliminates the steady-state error but may worsen the transient response; and Derivative (D) which increases the stability of the system, reduces overshoot, and improves the transient response.",
  "source_references": [
    {
      "chunk_id": "chunk-001",
      "module": "Robot Control Systems",
      "lesson": "Motor Control Algorithms",
      "section": "PID Controllers",
      "relevance_score": 0.92
    }
  ],
  "confidence_level": "high",
  "is_information_available": true,
  "response_time_ms": 1250.5,
  "query_id": "query-12345"
}
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

- `POST /chat` - Process user query and return AI-generated response
  - Request: JSON with query and user context
  - Response: AI response grounded in book content with source references
- `GET /chat/health` - Health check for the AI Answering Agent
  - Response: Health status of the service

## Configuration Options

- `OPENAI_API_KEY`: API key for OpenAI-compatible service with Gemini compatibility
- `CONTENT_EXTRACTION_ENDPOINT`: URL for the Content Extraction tool API
- `CONTENT_EXTRACTION_API_KEY`: API key for the Content Extraction tool
- `MODEL_NAME`: Name of the LLM to use (default: gpt-4-turbo)
- `TEMPERATURE`: Temperature setting for response variability (default: 0.3 for deterministic responses)
- `MAX_TOKENS`: Maximum tokens in the response (default: 2000)
- `TIMEOUT_SECONDS`: Request timeout (default: 30)
- `SYSTEM_PROMPT`: System prompt that guides the AI behavior

## Troubleshooting

### Common Issues

1. **API Key Authentication Failure**: Verify OPENAI_API_KEY and CONTENT_EXTRACTION_API_KEY are correctly set
2. **Content Extraction Tool Unreachable**: Check CONTENT_EXTRACTION_ENDPOINT is accessible
3. **No Relevant Content Found**: Verify the Content Extraction tool has relevant Physical AI Book content
4. **High Latency**: Check network connectivity to external APIs

### Health Check

```bash
curl http://localhost:8000/chat/health
```

Expected response: `{"status": "healthy", "service": "ai-answering-agent", "timestamp": "2025-12-17T10:00:00Z"}`