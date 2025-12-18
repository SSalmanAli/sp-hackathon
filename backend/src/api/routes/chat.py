"""
Chat API Routes for AI Answering Agent
Implements the chat endpoint for the Physical AI Book AI Answering Agent
"""

from typing import Dict, Any
import uuid
from datetime import datetime

from fastapi import APIRouter, HTTPException, Depends, Request
from pydantic import ValidationError

from ...models.user_query import UserQuery
from ...models.ai_response import AIResponse
from ...services.ai_agent_service import AIAgentService
from ...lib.logging_config import ai_agent_logger
from ...lib.exceptions import AIAgentException

router = APIRouter(prefix="/chat", tags=["chat"])


def get_ai_agent_service() -> AIAgentService:
    """Dependency to get AI agent service instance"""
    return AIAgentService()


@router.post("/", response_model=AIResponse)
async def process_chat_query(
    request: Request,
    user_query: UserQuery,
    ai_agent_service: AIAgentService = Depends(get_ai_agent_service)
):
    """
    Process user query and return AI-generated response
    Accepts a user query and returns an AI-generated response grounded in Physical AI Book content
    """
    query_id = user_query.query_id
    ai_agent_logger.info(f"Processing chat query {query_id}: {user_query.query_text[:100]}...")

    try:
        # Process the query using the AI agent service
        response = await ai_agent_service.process_query(user_query)

        ai_agent_logger.info(f"Successfully processed query {query_id}, response time: {response.response_time_ms}ms")

        return response

    except ValidationError as ve:
        ai_agent_logger.error(f"Validation error for query {query_id}: {str(ve)}")
        raise HTTPException(status_code=422, detail=f"Validation error: {str(ve)}")

    except AIAgentException as ae:
        ai_agent_logger.error(f"AI agent error for query {query_id}: {ae.error_code} - {ae.message}")
        raise HTTPException(status_code=500, detail={
            "error": ae.error_code,
            "message": ae.message
        })

    except Exception as e:
        ai_agent_logger.error(f"Unexpected error processing query {query_id}: {str(e)}")
        raise HTTPException(status_code=500, detail={
            "error": "INTERNAL_ERROR",
            "message": "An unexpected error occurred while processing your query"
        })


@router.get("/health")
async def chat_health_check():
    """
    Health check for the AI Answering Agent
    Check if the AI Answering Agent is operational
    """
    try:
        # Basic health check - just return status
        return {
            "status": "healthy",
            "service": "ai-answering-agent",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        ai_agent_logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail="Health check failed")