from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # OpenAI configuration
    openai_api_key: Optional[str] = None
    openai_base_url: Optional[str] = None  # For Gemini compatibility
    model_name: str = "gpt-4-turbo"
    temperature: float = 0.3
    max_tokens: int = 2000

    # Cohere configuration (for compatibility with existing setup)
    cohere_api_key: Optional[str] = None
    embedding_model_name: str = "embed-english-v3.0"

    # Qdrant configuration (for compatibility with existing setup)
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_host: str = "localhost"
    qdrant_port: str = "6333"
    qdrant_collection_name: str = "physical_ai_book_content"

    # Content Extraction Tool configuration
    content_extraction_endpoint: str = "http://localhost:8000/api/v1/search"
    content_extraction_api_key: Optional[str] = None

    # Timeout configuration
    timeout_seconds: int = 30

    # System prompt for AI agent
    system_prompt: str = (
        "You are an AI assistant for the Physical AI Book. "
        "Answer questions based only on the provided context. "
        "If the information is not in the context, explicitly state that the information is not available in the book."
    )

    class Config:
        env_file = ".env"


settings = Settings()