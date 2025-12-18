from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from enum import Enum


class ContentType(str, Enum):
    module = "module"
    lesson = "lesson"
    section = "section"
    exercise = "exercise"


class QueryFilters(BaseModel):
    content_types: Optional[List[ContentType]] = Field(default=None, description="Content types to include in search")


class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500, description="The natural language search query")
    filters: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Optional filters for the search")
    content_types: Optional[List[ContentType]] = Field(default=None, description="Content types to filter search results")
    similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0, description="Minimum similarity threshold for results")
    top_k: int = Field(default=10, ge=1, le=50, description="Number of top results to return")