class SearchException(Exception):
    """Base exception for search-related errors."""
    def __init__(self, message: str, error_code: str = "SEARCH_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class QueryValidationException(SearchException):
    """Exception raised for query validation errors."""
    def __init__(self, message: str):
        super().__init__(message, "QUERY_VALIDATION_ERROR")


class QdrantConnectionException(SearchException):
    """Exception raised for Qdrant connection errors."""
    def __init__(self, message: str):
        super().__init__(message, "QDRANT_CONNECTION_ERROR")


class EmbeddingGenerationException(SearchException):
    """Exception raised for embedding generation errors."""
    def __init__(self, message: str):
        super().__init__(message, "EMBEDDING_GENERATION_ERROR")


class NoResultsException(SearchException):
    """Exception raised when no relevant results are found."""
    def __init__(self, message: str = "No relevant results found for the given query"):
        super().__init__(message, "NO_RESULTS_ERROR")


class AIAgentException(Exception):
    """Base exception for AI agent-related errors."""
    def __init__(self, message: str, error_code: str = "AI_AGENT_ERROR"):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class ContentExtractionException(AIAgentException):
    """Exception raised for content extraction errors."""
    def __init__(self, message: str):
        super().__init__(message, "CONTENT_EXTRACTION_ERROR")


class AIResponseGenerationException(AIAgentException):
    """Exception raised for AI response generation errors."""
    def __init__(self, message: str):
        super().__init__(message, "AI_RESPONSE_GENERATION_ERROR")


class ContextGroundingException(AIAgentException):
    """Exception raised when AI response is not properly grounded in context."""
    def __init__(self, message: str):
        super().__init__(message, "CONTEXT_GROUNDING_ERROR")