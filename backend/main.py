"""
Embedding Pipeline Implementation
This script implements a complete pipeline for:
1. Crawling Docusaurus URLs
2. Extracting and cleaning text content
3. Generating embeddings using Cohere
4. Storing embeddings in Qdrant vector database
"""

import os
import logging
from typing import List, Dict, Optional
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import re
from urllib.parse import urljoin, urlparse
import argparse


# Load environment variables
load_dotenv()


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def load_config() -> Dict[str, any]:
    """
    Create configuration loading function to read environment variables

    Returns:
        Dict[str, any]: Configuration dictionary with all required settings
    """
    config = {
        'cohere_api_key': os.getenv('COHERE_API_KEY'),
        'qdrant_url': os.getenv('QDRANT_URL'),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY'),
        'qdrant_host': os.getenv('QDRANT_HOST', 'localhost'),
        'qdrant_port': int(os.getenv('QDRANT_PORT', 6333)),
        'chunk_size': int(os.getenv('CHUNK_SIZE', 1000)),
        'chunk_overlap': int(os.getenv('CHUNK_OVERLAP', 100)),
        'default_url': os.getenv('DOCUSSAURUS_URL', 'https://ssalmanali.github.io/sp-hackathon/')
    }
    return config


def setup_logging():
    """Implement logging setup"""
    # Already configured above, but this function can be expanded if needed
    pass


def get_all_urls(base_url: str) -> List[str]:
    """
    Implement get_all_urls function to crawl Docusaurus site

    Args:
        base_url (str): The base URL of the Docusaurus site to crawl

    Returns:
        List[str]: A list of all URLs found on the site
    """
    logger.info(f"Starting to crawl: {base_url}")
    urls = set()
    visited = set()

    # Add the base URL to start with
    urls.add(base_url)

    # Create a session for connection pooling
    session = requests.Session()
    session.headers.update({
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
    })

    while urls:
        current_url = urls.pop()

        if current_url in visited or not current_url.startswith(base_url):
            continue

        visited.add(current_url)
        logger.info(f"Crawling: {current_url}")

        try:
            response = session.get(current_url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Find all links that are relative to the base URL
            for link in soup.find_all('a', href=True):
                href = link['href']

                # Convert relative URLs to absolute
                absolute_url = urljoin(current_url, href)

                # Only add URLs that are under the same domain/base URL
                if absolute_url.startswith(base_url) and absolute_url not in visited:
                    urls.add(absolute_url)

        except requests.RequestException as e:
            logger.error(f"Error crawling {current_url}: {str(e)}")
            continue
        except Exception as e:
            logger.error(f"Unexpected error crawling {current_url}: {str(e)}")
            continue

    logger.info(f"Crawling complete. Found {len(visited)} URLs.")
    return list(visited)


def extract_text_from_url(url: str) -> str:
    """
    Implement extract_text_from_url function to extract clean text

    Args:
        url (str): The URL to extract text from

    Returns:
        str: The extracted clean text content
    """
    logger.info(f"Extracting text from: {url}")

    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to find main content areas in Docusaurus sites
        # Docusaurus typically has content in main tags or specific class names
        main_content = None

        # Look for main content area
        main_content = soup.find('main') or soup.find('article') or soup.find('div', class_=re.compile(r'container|main|content|doc'))

        if not main_content:
            # If no specific main content found, use the body
            main_content = soup.find('body')

        if main_content:
            text = main_content.get_text()
        else:
            text = soup.get_text()

        # Clean up the text
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        logger.info(f"Extracted {len(text)} characters from {url}")
        return text

    except requests.RequestException as e:
        logger.error(f"Error extracting text from {url}: {str(e)}")
        return ""
    except Exception as e:
        logger.error(f"Unexpected error extracting text from {url}: {str(e)}")
        return ""


def identify_docusaurus_elements() -> Dict[str, List[str]]:
    """
    Implement helper function to identify Docusaurus-specific HTML elements

    Returns:
        Dict[str, List[str]]: Dictionary containing patterns for Docusaurus elements
    """
    # This function provides patterns commonly found in Docusaurus sites
    patterns = {
        'nav_elements': ['nav', 'header', 'footer', 'aside'],
        'content_containers': ['main', 'article', 'div.container', 'div.main-wrapper', 'div.doc-page'],
        'sidebar_elements': ['div.sidebar', 'nav.sidebar', 'div.menu'],
        'doc_elements': ['div.doc-content', 'div.markdown', 'div.theme-doc-markdown'],
        'skip_classes': ['nav', 'navigation', 'sidebar', 'footer', 'header', 'menu', 'toc']
    }
    return patterns


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict]:
    """
    Implement chunk_text function to split text into manageable pieces

    Args:
        text (str): The text to chunk
        chunk_size (int): Size of each chunk in characters (default: 1000)
        overlap (int): Overlap between chunks in characters (default: 100)

    Returns:
        List[Dict]: List of dictionaries containing chunk content and position information
    """
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If we're at the end, make sure to include the remaining text
        if end > len(text):
            end = len(text)
        else:
            # Try to break at sentence boundary if possible
            # Look for sentence endings near the end
            for i in range(end, start, -1):
                if text[i] in '.!?':
                    end = i + 1
                    break

        chunk_text = text[start:end].strip()

        if chunk_text:  # Only add non-empty chunks
            chunks.append({
                'content': chunk_text,
                'start_pos': start,
                'end_pos': end
            })

        # Move start position with overlap
        start = end - overlap if end < len(text) else end

        # If overlap brings us back to same position, advance by chunk_size
        if start <= start:
            start += chunk_size

    logger.info(f"Text chunked into {len(chunks)} pieces")
    return chunks


def setup_cohere_client(config: Dict) -> cohere.Client:
    """
    Set up Cohere client configuration

    Args:
        config (Dict): Configuration dictionary containing COHERE_API_KEY

    Returns:
        cohere.Client: Initialized Cohere client

    Raises:
        ValueError: If COHERE_API_KEY is not provided in config
    """
    if not config.get('cohere_api_key'):
        raise ValueError("COHERE_API_KEY environment variable is required")

    client = cohere.Client(config['cohere_api_key'])
    logger.info("Cohere client initialized")
    return client


def embed(text_chunks: List[str], cohere_client: cohere.Client) -> List[List[float]]:
    """
    Implement embed function to generate embeddings using Cohere

    Args:
        text_chunks (List[str]): List of text chunks to embed
        cohere_client (cohere.Client): Initialized Cohere client

    Returns:
        List[List[float]]: List of embedding vectors

    Raises:
        Exception: If there's an error generating embeddings
    """
    if not text_chunks:
        logger.warning("No text chunks to embed")
        return []

    logger.info(f"Generating embeddings for {len(text_chunks)} text chunks")

    try:
        # Cohere's embed function can handle multiple texts at once
        response = cohere_client.embed(
            texts=text_chunks,
            model='embed-english-v3.0',  # Using the latest English embedding model
            input_type="search_document"  # Appropriate for document search
        )

        embeddings = response.embeddings
        logger.info(f"Successfully generated {len(embeddings)} embeddings")
        return embeddings

    except Exception as e:
        logger.error(f"Error generating embeddings: {str(e)}")
        raise


def setup_qdrant_client(config: Dict):
    """
    Set up Qdrant client configuration
    """
    if config.get('qdrant_url'):
        # Using remote Qdrant instance
        client = QdrantClient(
            url=config['qdrant_url'],
            api_key=config.get('qdrant_api_key')
        )
        logger.info(f"Qdrant client initialized with remote URL: {config['qdrant_url']}")
    else:
        # Using local Qdrant instance
        client = QdrantClient(
            host=config['qdrant_host'],
            port=config['qdrant_port']
        )
        logger.info(f"Qdrant client initialized with host: {config['qdrant_host']}, port: {config['qdrant_port']}")

    return client


def create_collection(client, collection_name: str = "rag-embedding"):
    """
    Implement create_collection function to create 'rag-embedding' collection
    """
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create the collection with appropriate vector size
        # Cohere's embed-english-v3.0 produces 1024-dimensional vectors
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
        )

        logger.info(f"Collection '{collection_name}' created successfully")
        return True

    except Exception as e:
        logger.error(f"Error creating collection '{collection_name}': {str(e)}")
        return False


def save_chunk_to_qdrant(client, collection_name: str, chunk_data: Dict, embedding: List[float], url: str, chunk_index: int):
    """
    Implement save_chunk_to_qdrant function to store embeddings with metadata
    """
    try:
        import uuid
        # Create a unique ID - Qdrant accepts UUIDs
        point_id = str(uuid.uuid4())

        # Prepare the point to be inserted
        point = models.PointStruct(
            id=point_id,  # Use UUID instead of hash-based string
            vector=embedding,
            payload={
                "source_url": url,
                "content": chunk_data['content'],
                "chunk_index": chunk_index,
                "start_pos": chunk_data['start_pos'],
                "end_pos": chunk_data['end_pos'],
                "created_at": time.time()
            }
        )

        # Insert the point into the collection
        client.upsert(
            collection_name=collection_name,
            points=[point]
        )

        logger.info(f"Chunk {chunk_index} from {url} saved to Qdrant with ID {point_id}")
        return True

    except Exception as e:
        logger.error(f"Error saving chunk to Qdrant: {str(e)}")
        return False


def main_pipeline(urls: List[str], config: Dict):
    """
    Create main execution function to orchestrate the full pipeline
    """
    logger.info("Starting embedding pipeline")

    # Initialize clients
    cohere_client = setup_cohere_client(config)
    qdrant_client = setup_qdrant_client(config)

    # Create the collection if it doesn't exist
    if not create_collection(qdrant_client, "rag-embedding"):
        logger.error("Failed to create Qdrant collection")
        return False

    total_chunks_processed = 0

    for i, url in enumerate(urls):
        logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

        # Extract text from URL
        text = extract_text_from_url(url)
        if not text.strip():
            logger.warning(f"No text extracted from {url}, skipping")
            continue

        # Chunk the text
        text_chunks = chunk_text(text, config['chunk_size'], config['chunk_overlap'])
        if not text_chunks:
            logger.warning(f"No chunks created from {url}, skipping")
            continue

        # Extract just the content for embedding
        chunk_contents = [chunk['content'] for chunk in text_chunks]

        try:
            # Generate embeddings
            embeddings = embed(chunk_contents, cohere_client)

            # Save each chunk with its embedding to Qdrant
            for idx, (chunk_data, embedding_vector) in enumerate(zip(text_chunks, embeddings)):
                success = save_chunk_to_qdrant(
                    qdrant_client,
                    "rag-embedding",
                    chunk_data,
                    embedding_vector,
                    url,
                    idx
                )

                if success:
                    total_chunks_processed += 1
                else:
                    logger.error(f"Failed to save chunk {idx} from {url}")

        except Exception as e:
            logger.error(f"Error processing {url}: {str(e)}")
            continue

    logger.info(f"Pipeline completed. Processed {total_chunks_processed} chunks from {len(urls)} URLs")
    return True


def test_crawling_functionality(target_url: str):
    """
    Test crawling functionality with the target site
    """
    logger.info(f"Testing crawling functionality with: {target_url}")
    urls = get_all_urls(target_url)
    logger.info(f"Found {len(urls)} URLs during test crawl")

    if urls:
        # Test extracting from the first URL
        sample_text = extract_text_from_url(urls[0])
        logger.info(f"Sample text length from first URL: {len(sample_text)} characters")

    return urls


def test_embedding_functionality():
    """
    Test embedding functionality with sample text
    """
    config = load_config()

    if not config.get('cohere_api_key'):
        logger.warning("COHERE_API_KEY not set, skipping embedding test")
        return False

    sample_text = ["This is a sample text for testing embedding functionality.",
                   "The embedding pipeline should be able to process this text successfully."]

    try:
        cohere_client = setup_cohere_client(config)
        embeddings = embed(sample_text, cohere_client)

        logger.info(f"Successfully generated {len(embeddings)} embeddings for test")
        logger.info(f"First embedding has {len(embeddings[0])} dimensions")

        return True
    except Exception as e:
        logger.error(f"Error in embedding test: {str(e)}")
        return False


def test_storage_functionality():
    """
    Test storage functionality with sample embeddings
    """
    config = load_config()

    sample_embedding = [0.1] * 1024  # Mock embedding vector

    try:
        qdrant_client = setup_qdrant_client(config)

        # Create collection
        if not create_collection(qdrant_client, "rag-embedding"):
            logger.error("Failed to create collection for storage test")
            return False

        # Create sample chunk data
        sample_chunk = {
            'content': 'This is a test chunk for storage functionality.',
            'start_pos': 0,
            'end_pos': 50
        }

        # Save to Qdrant
        success = save_chunk_to_qdrant(
            qdrant_client,
            "rag-embedding",
            sample_chunk,
            sample_embedding,
            "https://test.example.com",
            0
        )

        if success:
            logger.info("Storage test completed successfully")
            return True
        else:
            logger.error("Storage test failed")
            return False

    except Exception as e:
        logger.error(f"Error in storage test: {str(e)}")
        return False


def test_complete_pipeline(target_url: str):
    """
    Test complete pipeline with the target site
    """
    config = load_config()

    if not config.get('cohere_api_key'):
        logger.warning("COHERE_API_KEY not set, skipping complete pipeline test")
        return False

    # Get a few sample URLs for testing
    urls = get_all_urls(target_url)[:3]  # Limit to first 3 URLs for testing

    if not urls:
        logger.error("No URLs found for pipeline test")
        return False

    logger.info(f"Testing complete pipeline with {len(urls)} URLs")
    return main_pipeline(urls, config)


def main():
    """
    Main function with command-line argument parsing
    """
    parser = argparse.ArgumentParser(description='Embedding Pipeline for Docusaurus Sites')
    parser.add_argument('--url', type=str, default=None,
                        help='Target Docusaurus URL to process (default: from .env)')
    parser.add_argument('--test', type=str, choices=['crawl', 'embed', 'store', 'full'],
                        help='Run specific test instead of full pipeline')

    args = parser.parse_args()

    # Load configuration
    config = load_config()

    # Use provided URL or default from config
    target_url = args.url or config['default_url']

    if args.test:
        if args.test == 'crawl':
            test_crawling_functionality(target_url)
        elif args.test == 'embed':
            test_embedding_functionality()
        elif args.test == 'store':
            test_storage_functionality()
        elif args.test == 'full':
            test_complete_pipeline(target_url)
    else:
        # Run the full pipeline
        urls = get_all_urls(target_url)
        if urls:
            main_pipeline(urls, config)
        else:
            logger.error(f"No URLs found at {target_url}")


if __name__ == "__main__":
    main()