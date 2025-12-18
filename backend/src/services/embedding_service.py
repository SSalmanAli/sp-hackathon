import cohere
from typing import List
from src.lib.config import settings


class EmbeddingService:
    def __init__(self):
        self.client = cohere.Client(settings.cohere_api_key)
        self.model_name = settings.embedding_model_name

    def embed_query(self, query_text: str) -> List[float]:
        """
        Convert a query text to vector embedding using Cohere.

        Args:
            query_text: The natural language query to embed

        Returns:
            List of floats representing the vector embedding
        """
        response = self.client.embed(
            texts=[query_text],
            model=self.model_name,
            input_type="search_query"
        )
        return response.embeddings[0]

    def embed_content_chunks(self, content_chunks: List[str]) -> List[List[float]]:
        """
        Convert multiple content chunks to vector embeddings using Cohere.

        Args:
            content_chunks: List of content chunks to embed

        Returns:
            List of lists of floats representing the vector embeddings
        """
        response = self.client.embed(
            texts=content_chunks,
            model=self.model_name,
            input_type="search_document"
        )
        return response.embeddings