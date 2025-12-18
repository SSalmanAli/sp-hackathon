"""
ContextReference Model for AI Answering Agent
Specific references to book content (module, lesson, section, chunk identifiers) that support the AI response
"""

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