from typing import List, Dict, Any, Optional
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_search_service import QdrantSearchService
from src.models.query import QueryRequest
from src.models.result import SearchResult, SearchResults
from src.models.content_chunk import ContentChunk
from src.lib.config import settings
from src.lib.logging_config import search_logger
from src.lib.exceptions import QueryValidationException, NoResultsException
import time


class RetrievalService:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantSearchService()

    def search(self, query_request: QueryRequest) -> SearchResults:
        """
        Perform semantic search on Physical AI Book content.

        Args:
            query_request: The query request with text and parameters

        Returns:
            SearchResults containing ranked results with metadata
        """
        start_time = time.time()

        # Validate query length
        if len(query_request.query.strip()) == 0:
            raise QueryValidationException("Query cannot be empty")

        if len(query_request.query) > 500:
            raise QueryValidationException("Query exceeds maximum length of 500 characters")

        # Generate embedding for the query
        search_logger.info(f"Generating embedding for query: {query_request.query[:50]}...")
        query_vector = self.embedding_service.embed_query(query_request.query)

        # Prepare filters
        filters = query_request.filters or {}

        # Prepare content types filter
        content_types = query_request.content_types

        # Search in Qdrant
        search_logger.info(f"Searching Qdrant with vector for query: {query_request.query[:50]}...")
        raw_results = self.qdrant_service.search_content_chunks(
            query_vector=query_vector,
            top_k=query_request.top_k,
            filters=filters,
            content_types=content_types,
            similarity_threshold=query_request.similarity_threshold
        )

        # Transform raw results to SearchResults model
        search_results = []
        for idx, raw_result in enumerate(raw_results):
            # Create ContentChunk from raw result
            content_chunk = ContentChunk(
                chunk_id=raw_result["chunk_id"],
                module_id=raw_result["module_id"],
                lesson_id=raw_result["lesson_id"],
                section_id=raw_result["section_id"],
                content=raw_result["content"],
                metadata=raw_result["metadata"]
            )

            # Create metadata for the search result
            result_metadata = {
                "module": raw_result["module_id"],
                "lesson": raw_result["lesson_id"],
                "section": raw_result["section_id"],
                "chunk": raw_result["chunk_id"]
            }

            # Add any additional metadata from the payload
            if "metadata" in raw_result and raw_result["metadata"]:
                result_metadata.update(raw_result["metadata"])

            # Create SearchResult
            search_result = SearchResult(
                content_chunk=content_chunk,
                similarity_score=raw_result["similarity_score"],
                metadata=result_metadata,
                relevance_rank=idx + 1
            )

            search_results.append(search_result)

        # Calculate execution time
        execution_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Create final SearchResults object
        final_results = SearchResults(
            query=query_request.query,
            results=search_results,
            total_results=len(search_results),
            execution_time_ms=execution_time
        )

        search_logger.info(f"Search completed in {execution_time:.2f}ms, found {len(search_results)} results")

        return final_results

    def deduplicate_results(self, results: List[SearchResult], threshold: float = 0.9) -> List[SearchResult]:
        """
        Remove duplicate or highly similar results based on content similarity.

        Args:
            results: List of search results to deduplicate
            threshold: Similarity threshold above which results are considered duplicates

        Returns:
            List of deduplicated search results
        """
        if not results:
            return []

        # For now, we'll implement a simple deduplication based on content similarity
        # In a real implementation, we might use more sophisticated techniques
        unique_results = []
        seen_content = set()

        for result in results:
            # Use a hash of the content as a simple deduplication mechanism
            content_hash = hash(result.content_chunk.content.strip().lower())

            if content_hash not in seen_content:
                seen_content.add(content_hash)
                unique_results.append(result)

        return unique_results

    def sort_results_by_relevance(self, results: List[SearchResult]) -> List[SearchResult]:
        """
        Sort results by similarity score in descending order.

        Args:
            results: List of search results to sort

        Returns:
            List of search results sorted by relevance
        """
        sorted_results = sorted(results, key=lambda x: x.similarity_score, reverse=True)

        # Update relevance ranks after sorting
        for idx, result in enumerate(sorted_results):
            result.relevance_rank = idx + 1

        return sorted_results