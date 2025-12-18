"""
Test script to verify AI Agent Service can be instantiated
"""

from src.services.ai_agent_service import AIAgentService


def test_ai_agent_service_instantiation():
    """Test that the AI agent service can be instantiated"""
    print("Testing AI Agent Service instantiation...")

    try:
        # Create an instance of the AI agent service
        agent_service = AIAgentService()
        print("+ AIAgentService instantiated successfully")
        print(f"  - Model name: {agent_service.model_name}")
        print(f"  - Temperature: {agent_service.temperature}")
        print(f"  - Max tokens: {agent_service.max_tokens}")
        print(f"  - Timeout: {agent_service.timeout}")

        # Check if API key is available (it might be None)
        if agent_service.client and agent_service.client.api_key:
            print("  - API key: Available")
        else:
            print("  - API key: Not configured (expected in test environment)")

        print("\n+ AI Agent Service instantiation test passed!")

    except Exception as e:
        print(f"X Failed to instantiate AIAgentService: {str(e)}")
        return False

    return True


if __name__ == "__main__":
    test_ai_agent_service_instantiation()