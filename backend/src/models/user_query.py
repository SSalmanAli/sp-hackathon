"""
UserQuery Model for AI Answering Agent
Represents a user's natural language question submitted to the AI Answering Agent
"""

import uuid
from datetime import datetime
from typing import Optional, Dict, Any

from pydantic import BaseModel, Field, validator


class UserQuery(BaseModel):
    """
    Represents a user's natural language question submitted to the AI Answering Agent
    """
    query_text: str = Field(
        ...,
        description="The user's natural language question",
        min_length=1,
        max_length=1000
    )
    user_context: Optional[Dict[str, Any]] = Field(
        default_factory=dict,
        description="Additional context about the user or conversation history"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the query was submitted"
    )
    query_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the query"
    )

    @validator('query_text')
    def validate_query_text(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query_text cannot be empty')
        if len(v) > 1000:
            raise ValueError('query_text must be 1000 characters or less')
        return v