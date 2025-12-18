# Interface Contract: Embedding Pipeline Functions

## Function Interfaces

### get_all_urls(base_url: str) -> List[str]
**Purpose**: Crawl the Docusaurus site and return all discoverable URLs
- **Input**: Base URL of the Docusaurus site
- **Output**: List of all page URLs found on the site
- **Errors**: Returns empty list if site cannot be crawled

### extract_text_from_url(url: str) -> str
**Purpose**: Extract clean text content from a single URL
- **Input**: URL to extract text from
- **Output**: Clean text content without HTML tags
- **Errors**: Returns empty string if URL is inaccessible

### chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict]
**Purpose**: Split text into smaller chunks for embedding
- **Input**: Text to chunk, chunk size, and overlap
- **Output**: List of dictionaries containing chunk content and metadata
- **Errors**: Returns empty list if text is empty

### embed(text_chunks: List[str]) -> List[List[float]]
**Purpose**: Generate embeddings for text chunks using Cohere
- **Input**: List of text chunks to embed
- **Output**: List of embedding vectors (lists of floats)
- **Errors**: Returns empty list if API call fails

### create_collection(collection_name: str) -> bool
**Purpose**: Create a Qdrant collection for storing embeddings
- **Input**: Name of the collection to create
- **Output**: True if successful, False otherwise
- **Errors**: Returns False if collection creation fails

### save_chunk_to_qdrant(collection_name: str, chunk_data: Dict, embedding: List[float]) -> bool
**Purpose**: Save a text chunk with its embedding to Qdrant
- **Input**: Collection name, chunk data dictionary, and embedding vector
- **Output**: True if successful, False otherwise
- **Errors**: Returns False if saving fails