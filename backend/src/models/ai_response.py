"""
AIResponse Model for AI Answering Agent
Represents the AI assistant's answer to the user, grounded in extracted content and formatted for chatbot UI
"""

from typing import List, Dict, Any
from datetime import datetime

from pydantic import BaseModel, Field, validator


class ContextReference(BaseModel):
    """
    Specific references to book content (module, lesson, section, chunk identifiers) that support the AI response
    """
    chunk_id: str = Field(
        ...,
        description="Reference to the specific content chunk"
    )
    module: str = Field(
        ...,
        description="Module name where content is found"
    )
    lesson: str = Field(
        ...,
        description="Lesson name where content is found"
    )
    section: str = Field(
        ...,
        description="Section name where content is found"
    )
    relevance_score: float = Field(
        ...,
        description="How relevant this content is to the query",
        ge=0.0,
        le=1.0
    )

    @validator('chunk_id', 'module', 'lesson', 'section')
    def validate_ids(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('All ID fields must be non-empty strings')
        return v

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('relevance_score must be between 0.0 and 1.0')
        return v


class AIResponse(BaseModel):
    """
    Represents the AI assistant's answer to the user, grounded in extracted content and formatted for chatbot UI
    """
    is_information_available: bool = Field(
        ...,
        description="Whether the requested information was found in the book"
    )
    response_text: str = Field(
        ...,
        description="The AI's response to the user's query"
    )
    source_references: List[ContextReference] = Field(
        default_factory=list,
        description="References to book content that support the response"
    )
    confidence_level: str = Field(
        ...,
        description="Confidence level of the response",
        pattern=r"^(high|medium|low)$"
    )
    response_time_ms: float = Field(
        ...,
        description="Time taken to generate response in milliseconds",
        ge=0
    )
    query_id: str = Field(
        ...,
        description="Reference to the original query ID"
    )

    @validator('response_text')
    def validate_response_text(cls, v, values):
        # Response text must be provided and non-empty when information is available
        is_available = values.get('is_information_available', True)
        if is_available and (not v or len(v.strip()) == 0):
            raise ValueError('response_text must be provided and non-empty when is_information_available is true')
        return v

    @validator('source_references')
    def validate_source_references(cls, v, values):
        # We now allow empty source_references for general chat
        return v


    @validator('response_time_ms')
    def validate_response_time_ms(cls, v):
        if v < 0:
            raise ValueError('response_time_ms must be >= 0')
        return v

    @validator('query_id')
    def validate_query_id(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query_id cannot be empty')
        return v