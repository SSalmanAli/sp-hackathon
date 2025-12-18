# Data Model: Embedding Pipeline

## Entities

### DocusaurusDocument
**Description**: Represents a crawled page from a Docusaurus site
- **url**: string - The source URL of the document
- **raw_html**: string - The raw HTML content extracted from the page
- **clean_text**: string - The cleaned text content with HTML tags removed
- **title**: string - The title of the page
- **created_at**: datetime - When the document was crawled

### TextChunk
**Description**: Represents a chunk of text that will be embedded
- **id**: string - Unique identifier for the chunk (UUID)
- **document_url**: string - Reference to the source document
- **content**: string - The text content of the chunk
- **chunk_index**: integer - The position of this chunk in the original document
- **metadata**: object - Additional metadata about the chunk

### EmbeddingVector
**Description**: Represents the vector representation of text content
- **id**: string - Unique identifier for the embedding (matches TextChunk.id)
- **vector**: array[float] - The embedding vector from Cohere
- **document_url**: string - Reference to the source document
- **chunk_index**: integer - The position of this chunk in the original document
- **created_at**: datetime - When the embedding was generated

### QdrantPoint
**Description**: Represents a point stored in Qdrant vector database
- **id**: string - Unique identifier for the point
- **vector**: array[float] - The embedding vector
- **payload**: object - Metadata including source URL, content, and other attributes
  - **source_url**: string - The original document URL
  - **content**: string - The text content of the chunk
  - **chunk_index**: integer - Position in original document
  - **title**: string - Title of the source document
  - **created_at**: datetime - When the embedding was stored

## Relationships

- **DocusaurusDocument** 1 → * **TextChunk**: One document can be split into multiple text chunks
- **TextChunk** 1 → 1 **EmbeddingVector**: Each text chunk generates one embedding vector
- **EmbeddingVector** 1 → 1 **QdrantPoint**: Each embedding vector is stored as one point in Qdrant

## Validation Rules

### DocusaurusDocument
- url must be a valid URL format
- clean_text must not be empty after cleaning
- created_at must be a valid timestamp

### TextChunk
- content must not exceed Cohere's token limit
- chunk_index must be a non-negative integer
- id must be unique within the system

### EmbeddingVector
- vector must have consistent dimensions across all embeddings
- vector values must be valid floats
- document_url must reference an existing DocusaurusDocument

## State Transitions

1. **Crawling State**: DocusaurusDocument is created with raw_html and basic metadata
2. **Processing State**: DocusaurusDocument is cleaned to create clean_text
3. **Chunking State**: DocusaurusDocument is split into multiple TextChunk objects
4. **Embedding State**: TextChunk objects are converted to EmbeddingVector objects
5. **Storage State**: EmbeddingVector objects are stored as QdrantPoint objects in the vector database