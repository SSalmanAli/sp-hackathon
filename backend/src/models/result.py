from pydantic import BaseModel, Field
from typing import List
from .content_chunk import ContentChunk
from typing import Optional, Dict, Any


class SearchResult(BaseModel):
    content_chunk: ContentChunk = Field(..., description="The matched content chunk")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score between query and result")
    metadata: Dict[str, str] = Field(..., description="Complete metadata including module, lesson, section, and chunk identifiers")
    relevance_rank: int = Field(..., ge=1, description="Rank position in the result set (1-indexed)")


class SearchResults(BaseModel):
    query: str = Field(..., description="The original query text")
    results: List[SearchResult] = Field(..., description="Array of ranked search results")
    total_results: int = Field(..., ge=0, description="Total number of results found before ranking")
    execution_time_ms: float = Field(..., ge=0, description="Time taken to execute the search in milliseconds")