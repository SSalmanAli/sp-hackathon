from fastapi import APIRouter, Depends, HTTPException, status
from typing import Dict, Any
from src.models.query import QueryRequest
from src.services.retrieval_service import RetrievalService
from src.api.dependencies import get_api_key
from src.lib.exceptions import SearchException, QueryValidationException, NoResultsException
from src.lib.logging_config import search_logger


router = APIRouter()


@router.post("/search", summary="Perform semantic search on Physical AI Book content")
async def search_content(
    query_request: QueryRequest,
    api_key: str = Depends(get_api_key)
):
    """
    Accepts a natural language query and returns semantically similar content chunks from the Physical AI Book.
    """
    try:
        # Log the incoming request
        search_logger.info(f"Received search request for query: {query_request.query[:50]}...")

        # Log content type filters if provided
        if query_request.content_types:
            search_logger.info(f"Content type filters applied: {query_request.content_types}")

        # Initialize retrieval service
        retrieval_service = RetrievalService()

        # Perform the search
        results = retrieval_service.search(query_request)

        # Log successful completion
        search_logger.info(f"Search completed successfully, returning {len(results.results)} results")

        # Return the results
        return results

    except QueryValidationException as e:
        search_logger.error(f"Query validation error: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"error": e.error_code, "message": e.message}
        )

    except NoResultsException as e:
        search_logger.info(f"No results found: {e.message}")
        # Return empty results instead of error for no results
        return {
            "query": query_request.query,
            "results": [],
            "total_results": 0,
            "execution_time_ms": 0.0
        }

    except SearchException as e:
        search_logger.error(f"Search service error: {e.message}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": e.error_code, "message": e.message}
        )

    except Exception as e:
        search_logger.error(f"Unexpected error during search: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={"error": "INTERNAL_ERROR", "message": "An unexpected error occurred during search"}
        )


@router.get("/health", summary="Health check for the search service")
async def search_health():
    """
    Health check endpoint for the search service.
    """
    return {"status": "search service healthy", "service": "qdrant-retrieval"}