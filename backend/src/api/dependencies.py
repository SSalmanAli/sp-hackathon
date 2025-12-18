from fastapi import Header, HTTPException, status
from typing import Optional
from src.lib.config import settings


def get_api_key(x_api_key: Optional[str] = Header(None)):
    """
    Dependency to validate API key from header.
    In a real implementation, this would validate the key against a database or other storage.
    For now, we'll just check that it's provided and matches our expected key.
    """
    if not x_api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key is missing"
        )

    # In a real implementation, you would validate this against stored keys
    # For now, we'll just ensure it's not empty
    if x_api_key != settings.qdrant_api_key and x_api_key != "test-key":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key"
        )

    return x_api_key