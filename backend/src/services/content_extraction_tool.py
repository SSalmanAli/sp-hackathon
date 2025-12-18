import httpx
from typing import Dict, Any, List, Optional
from src.lib.config import settings


class ContentExtractionTool:
    """
    Tool that interfaces with the Content Extraction service to retrieve
    relevant book content based on user queries.
    """

    def __init__(self):
        self.endpoint = settings.content_extraction_endpoint
        self.api_key = settings.content_extraction_api_key

    async def extract_content(self, query_text: str, context_requirements: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Extract relevant content from the Physical AI Book based on the query.

        Args:
            query_text: The user's query text
            context_requirements: Additional requirements for context extraction

        Returns:
            Dictionary containing content chunks and metadata
        """
        headers = {
            "Content-Type": "application/json"
        }

        if self.api_key:
            headers["X-API-Key"] = self.api_key

        # Prepare the request payload
        payload = {
            "query": query_text,
            "similarity_threshold": 0.5,  # Minimum relevance threshold
            "top_k": 5  # Number of results to return
        }

        # Add any context requirements to the payload
        if context_requirements:
            payload.update(context_requirements)

        try:
            async with httpx.AsyncClient(timeout=settings.timeout_seconds) as client:
                response = await client.post(
                    self.endpoint,
                    json=payload,
                    headers=headers
                )

                response.raise_for_status()
                result = response.json()

                # Process and return the extracted content
                return {
                    "content_chunks": result.get("results", []),
                    "total_chunks_found": result.get("total_results", 0),
                    "extraction_time_ms": result.get("execution_time_ms", 0),
                    "query_id": result.get("query_id", "")
                }

        except httpx.RequestError as e:
            # Handle request errors (connection issues, timeouts, etc.)
            return {
                "content_chunks": [],
                "total_chunks_found": 0,
                "extraction_time_ms": 0,
                "query_id": "",
                "error": f"Request error: {str(e)}"
            }
        except httpx.HTTPStatusError as e:
            # Handle HTTP errors (4xx, 5xx status codes)
            return {
                "content_chunks": [],
                "total_chunks_found": 0,
                "extraction_time_ms": 0,
                "query_id": "",
                "error": f"HTTP error {e.response.status_code}: {str(e)}"
            }
        except Exception as e:
            # Handle any other errors
            return {
                "content_chunks": [],
                "total_chunks_found": 0,
                "extraction_time_ms": 0,
                "query_id": "",
                "error": f"Extraction error: {str(e)}"
            }

    def extract_content_sync(self, query_text: str, context_requirements: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Synchronous version of content extraction.
        """
        import requests

        headers = {
            "Content-Type": "application/json"
        }

        if self.api_key:
            headers["X-API-Key"] = self.api_key

        # Prepare the request payload
        payload = {
            "query": query_text,
            "similarity_threshold": 0.5,  # Minimum relevance threshold
            "top_k": 5  # Number of results to return
        }

        # Add any context requirements to the payload
        if context_requirements:
            payload.update(context_requirements)

        try:
            response = requests.post(
                self.endpoint,
                json=payload,
                headers=headers,
                timeout=settings.timeout_seconds
            )

            response.raise_for_status()
            result = response.json()

            # Process and return the extracted content
            return {
                "content_chunks": result.get("results", []),
                "total_chunks_found": result.get("total_results", 0),
                "extraction_time_ms": result.get("execution_time_ms", 0),
                "query_id": result.get("query_id", "")
            }

        except requests.RequestException as e:
            # Handle request errors (connection issues, timeouts, etc.)
            return {
                "content_chunks": [],
                "total_chunks_found": 0,
                "extraction_time_ms": 0,
                "query_id": "",
                "error": f"Request error: {str(e)}"
            }