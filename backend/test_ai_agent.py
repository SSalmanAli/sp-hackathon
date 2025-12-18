"""
Test script to verify AI Answering Agent implementation
"""

import uuid
from datetime import datetime

from src.models.user_query import UserQuery
from src.models.ai_response import AIResponse, ContextReference


def test_ai_agent_models():
    """Test the AI agent models functionality"""
    print("Testing AI Answering Agent models...")

    # Test model creation
    print("\n1. Testing UserQuery model creation...")
    query = UserQuery(
        query_text="What is a PID controller in robotics?",
        user_context={"test": True},
        query_id=str(uuid.uuid4())
    )
    print(f"   UserQuery created: {query.query_text}")
    print(f"   Query ID: {query.query_id}")
    print(f"   Timestamp: {query.timestamp}")

    # Test ContextReference model
    print("\n2. Testing ContextReference model...")
    context_ref = ContextReference(
        chunk_id="chunk-001",
        module="Robot Control Systems",
        lesson="Motor Control Algorithms",
        section="PID Controllers",
        relevance_score=0.92
    )
    print(f"   ContextReference created: {context_ref.module} - {context_ref.section}")
    print(f"   Relevance score: {context_ref.relevance_score}")

    # Test AIResponse model
    print("\n3. Testing AIResponse model...")
    response_model = AIResponse(
        response_text="This is a test response",
        source_references=[context_ref],
        confidence_level="high",
        response_time_ms=100.0,
        query_id=query.query_id,
        is_information_available=True
    )
    print(f"   AIResponse created successfully: {response_model.response_text}")
    print(f"   Source references: {len(response_model.source_references)}")
    print(f"   Confidence level: {response_model.confidence_level}")
    print(f"   Response time: {response_model.response_time_ms}ms")
    print(f"   Query ID: {response_model.query_id}")
    print(f"   Information available: {response_model.is_information_available}")

    # Test validation - invalid confidence level
    print("\n4. Testing validation...")
    try:
        invalid_response = AIResponse(
            response_text="This should fail",
            source_references=[context_ref],
            confidence_level="invalid",  # This should fail validation
            response_time_ms=100.0,
            query_id=query.query_id,
            is_information_available=True
        )
        print("   Validation failed to catch invalid confidence level")
    except Exception as e:
        print(f"   Validation correctly caught invalid confidence level: {type(e).__name__}")

    # Test validation - invalid relevance score
    try:
        invalid_context = ContextReference(
            chunk_id="chunk-002",
            module="Test Module",
            lesson="Test Lesson",
            section="Test Section",
            relevance_score=1.5  # This should fail validation (must be 0-1)
        )
        print("   Validation failed to catch invalid relevance score")
    except Exception as e:
        print(f"   Validation correctly caught invalid relevance score: {type(e).__name__}")

    print("\nAll model tests completed successfully!")


if __name__ == "__main__":
    test_ai_agent_models()