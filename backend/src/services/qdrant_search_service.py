from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from src.lib.config import settings


class QdrantSearchService:
    def __init__(self):
        # Use URL if provided, otherwise use host/port
        if settings.qdrant_url:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        else:
            # For local Qdrant, don't pass api_key if it's None or empty
            kwargs = {
                "host": settings.qdrant_host,
                "port": settings.qdrant_port
            }
            if settings.qdrant_api_key:
                kwargs["api_key"] = settings.qdrant_api_key
            
            self.client = QdrantClient(**kwargs)
        self.collection_name = settings.qdrant_collection_name

    def search_content_chunks(
        self,
        query_vector: List[float],
        top_k: int = 10,
        filters: Optional[Dict[str, Any]] = None,
        content_types: Optional[List[str]] = None,
        similarity_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search for content chunks in Qdrant based on the query vector.

        Args:
            query_vector: The vector representation of the query
            top_k: Number of top results to return
            filters: Optional filters to apply to the search
            content_types: Optional content types to filter search results
            similarity_threshold: Minimum similarity threshold for results

        Returns:
            List of search results with content and metadata
        """
        # Build filter conditions
        filter_conditions = []

        # Add custom filters if provided
        if filters:
            for key, value in filters.items():
                if isinstance(value, list):
                    # Handle array-like filters (e.g., content_types)
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchAny(any=value)
                        )
                    )
                else:
                    # Handle single value filters
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

        # Add content type filters if provided
        if content_types:
            filter_conditions.append(
                models.FieldCondition(
                    key="content_type",  # Assuming content type is stored as a field in Qdrant
                    match=models.MatchAny(any=content_types)
                )
            )

        # Create search filter if any conditions exist
        search_filter = None
        if filter_conditions:
            search_filter = models.Filter(
                must=filter_conditions
            )

        # Perform the search
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=similarity_threshold,
            query_filter=search_filter
        )

        # Format results
        formatted_results = []
        for result in search_results:
            formatted_result = {
                "chunk_id": result.id,
                "content": result.payload.get("content", ""),
                "module_id": result.payload.get("module_id", ""),
                "lesson_id": result.payload.get("lesson_id", ""),
                "section_id": result.payload.get("section_id", ""),
                "similarity_score": result.score,
                "metadata": result.payload.get("metadata", {})
            }
            formatted_results.append(formatted_result)

        return formatted_results

    def get_content_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific content chunk by its ID.

        Args:
            chunk_id: The ID of the content chunk to retrieve

        Returns:
            The content chunk with metadata, or None if not found
        """
        records = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[chunk_id]
        )

        if records:
            record = records[0]
            return {
                "chunk_id": record.id,
                "content": record.payload.get("content", ""),
                "module_id": record.payload.get("module_id", ""),
                "lesson_id": record.payload.get("lesson_id", ""),
                "section_id": record.payload.get("section_id", ""),
                "metadata": record.payload.get("metadata", {})
            }

        return None