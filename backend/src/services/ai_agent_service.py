"""
AI Agent Service for Physical AI Book
Implements an AI Answering Agent that operates within the Physical AI Book chatbot.
Uses OpenAI SDK with Gemini-compatible API key and follows tool-augmented reasoning.
"""

import asyncio
import logging
from typing import Optional, List, Dict, Any
from datetime import datetime

from openai import AsyncOpenAI
from ..models.user_query import UserQuery
from ..models.ai_response import AIResponse, ContextReference
from .retrieval_service import RetrievalService
from ..models.query import QueryRequest
from ..lib.config import settings
from ..lib.exceptions import (
    AIAgentException,
    ContentExtractionException,
    AIResponseGenerationException,
    ContextGroundingException
)
from ..lib.logging_config import ai_agent_logger


class AIAgentService:
    """Main AI Agent Service for Physical AI Book"""

    def __init__(self):
        # Only initialize the OpenAI client if API key is available
        if settings.openai_api_key:
            self.client = AsyncOpenAI(
                api_key=settings.openai_api_key,
                base_url=settings.openai_base_url,  # For Gemini compatibility
            )
        else:
            self.client = None  # Will be handled in methods that require it
        self.retrieval_service = RetrievalService()
        self.model_name = settings.model_name
        self.temperature = settings.temperature
        self.max_tokens = settings.max_tokens
        self.timeout = settings.timeout_seconds

    async def process_query(self, user_query: UserQuery) -> AIResponse:
        """
        Process a user query by directly calling the OpenAI SDK (Simple Chat Mode)
        """
        start_time = datetime.now()
        query_id = user_query.query_id

        ai_agent_logger.info(f"Processing query {query_id}: {user_query.query_text[:100]}...")

        try:
            # Check if OpenAI client is available
            if not self.client:
                raise AIAgentException("OpenAI client not configured", "CONFIG_ERROR")

            # Create system message for general chat
            system_message = (
                "You are an AI assistant for the Physical AI Book. "
                "You can answer questions about Physical AI, robotics, sensors, and control systems. "
                "Be helpful, educational, and concise."
            )

            # Call OpenAI API directly
            ai_agent_logger.debug("Calling OpenAI API to generate response")
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_query.query_text}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                timeout=self.timeout
            )

            response_text = response.choices[0].message.content.strip()
            
            # Calculate response time
            response_time = (datetime.now() - start_time).total_seconds() * 1000

            ai_agent_logger.info(f"Successfully processed query {query_id}, response time: {response_time:.2f}ms")

            return AIResponse(
                is_information_available=True,
                response_text=response_text,
                source_references=[],  # No sources in simple mode
                confidence_level="high", # Assume high for direct chat
                response_time_ms=response_time,
                query_id=query_id
            )

        except Exception as e:
            ai_agent_logger.error(f"Unexpected error processing query {query_id}: {str(e)}")
            response_time = (datetime.now() - start_time).total_seconds() * 1000

            # Return error response with explicit information unavailable message
            return AIResponse(
                response_text="An unexpected error occurred while processing your query. Please try again later.",
                source_references=[],
                confidence_level="low",
                response_time_ms=response_time,
                query_id=query_id,
                is_information_available=False
            )

    def _construct_response_from_content(self, query: str, content_chunks: List) -> str:
        """
        Construct a proper response from the retrieved content chunks
        """
        if not content_chunks:
            return "The information requested is not available in the Physical AI Book."

        # Format the content chunks into a coherent response
        response_parts = []

        # Start with acknowledging the query
        response_parts.append(f"Based on the Physical AI Book, here's information about '{query}':")
        response_parts.append("")  # Empty line for formatting

        # Add content from each chunk
        for i, chunk in enumerate(content_chunks):
            response_parts.append(f"Module: {chunk.module_id}")
            response_parts.append(f"Lesson: {chunk.lesson_id}")
            response_parts.append(f"Section: {chunk.section_id}")
            response_parts.append(f"Content: {chunk.content}")
            if i < len(content_chunks) - 1:
                response_parts.append("")  # Empty line between chunks

        # Add a note about the sources
        response_parts.append("")
        response_parts.append("This information was retrieved from the Physical AI Book content.")

        return "\n".join(response_parts)

    def _determine_confidence_level(self, source_references: List[Dict], response_text: str) -> str:
        """
        Determine confidence level based on source references and response quality
        """
        if not source_references or not response_text.strip():
            return "low"

        # Calculate average relevance score
        avg_relevance = sum(ref["relevance_score"] for ref in source_references) / len(source_references) if source_references else 0

        if avg_relevance >= 0.8:
            return "high"
        elif avg_relevance >= 0.6:
            return "medium"
        else:
            return "low"