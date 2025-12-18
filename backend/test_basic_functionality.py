"""
Basic functionality test to verify the search service implementation.
This is a simple test to ensure all components work together.
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.services.embedding_service import EmbeddingService
from src.services.qdrant_search_service import QdrantSearchService
from src.services.retrieval_service import RetrievalService
from src.models.query import QueryRequest
from src.lib.config import settings


def test_services_initialization():
    """Test that all services can be initialized without errors."""
    print("Testing service initialization...")

    try:
        # Test embedding service initialization
        embedding_service = EmbeddingService()
        print("+ Embedding service initialized")

        # Test Qdrant search service initialization
        qdrant_service = QdrantSearchService()
        print("+ Qdrant search service initialized")

        # Test retrieval service initialization
        retrieval_service = RetrievalService()
        print("+ Retrieval service initialized")

        return True
    except Exception as e:
        print(f"- Service initialization failed: {e}")
        return False


def test_query_model():
    """Test that the QueryRequest model works correctly."""
    print("\nTesting QueryRequest model...")

    try:
        # Test creating a valid query request
        query_request = QueryRequest(
            query="How do I implement a PID controller?",
            similarity_threshold=0.7,
            top_k=5
        )

        print(f"+ QueryRequest created: {query_request.query}")
        print(f"+ Similarity threshold: {query_request.similarity_threshold}")
        print(f"+ Top K: {query_request.top_k}")

        return True
    except Exception as e:
        print(f"- QueryRequest model test failed: {e}")
        return False


def test_config_loading():
    """Test that configuration is loaded properly."""
    print("\nTesting configuration loading...")

    try:
        # Check that required settings are available
        assert settings.cohere_api_key, "Cohere API key should be set"
        print(f"+ Cohere API key loaded (first 5 chars): {settings.cohere_api_key[:5]}...")

        print(f"+ Qdrant host: {settings.qdrant_host}")
        print(f"+ Qdrant port: {settings.qdrant_port}")
        print(f"+ Collection name: {settings.qdrant_collection_name}")

        return True
    except Exception as e:
        print(f"- Configuration loading test failed: {e}")
        return False


if __name__ == "__main__":
    print("Running basic functionality tests for Qdrant retrieval service...\n")

    all_tests_passed = True

    all_tests_passed &= test_config_loading()
    all_tests_passed &= test_query_model()
    all_tests_passed &= test_services_initialization()

    print(f"\n{'='*50}")
    if all_tests_passed:
        print("+ All basic functionality tests PASSED!")
        print("\nThe implementation is structurally correct and all components can be initialized.")
        print("Note: Actual search functionality requires a running Qdrant instance and valid Cohere API key.")
    else:
        print("- Some tests FAILED!")
        print("\nThere are issues with the basic implementation that need to be addressed.")
    print(f"{'='*50}")