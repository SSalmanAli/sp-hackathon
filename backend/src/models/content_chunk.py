from pydantic import BaseModel, Field
from typing import Dict, Any, Optional


class ContentChunk(BaseModel):
    chunk_id: str = Field(..., description="Unique identifier for the content chunk")
    module_id: str = Field(..., description="Module identifier where this content belongs")
    lesson_id: str = Field(..., description="Lesson identifier within the module")
    section_id: str = Field(..., description="Section identifier within the lesson")
    content: str = Field(..., min_length=10, max_length=10000, description="The actual text content of the chunk")
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Additional metadata about the content")