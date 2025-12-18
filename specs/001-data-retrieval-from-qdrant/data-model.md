# Data Model: Data Retrieval from Qdrant Vector Database

**Feature**: Data Retrieval from Qdrant Vector Database
**Date**: 2025-12-17
**Branch**: 001-data-retrieval-from-qdrant

## Overview

This document defines the data models for the Qdrant-based retrieval system that will enable semantic search of Physical AI Book content. The models are designed to support the functional requirements while maintaining the separation of concerns required by the constitutional principles.

## Core Entities

### Query Model
**Purpose**: Represents a user's search query input

**Fields**:
- `query_text` (string, required): The natural language search query from the user
- `filters` (object, optional): Content type filters (module, lesson, section, etc.)
- `similarity_threshold` (float, optional): Minimum similarity threshold for results (default: 0.7)
- `top_k` (integer, optional): Number of top results to return (default: 10, max: 50)

**Validation Rules**:
- `query_text` must be 1-500 characters
- `similarity_threshold` must be between 0.0 and 1.0
- `top_k` must be between 1 and 50

### Content Chunk Model
**Purpose**: Represents a chunk of Physical AI Book content stored in Qdrant

**Fields**:
- `chunk_id` (string, required): Unique identifier for the content chunk
- `module_id` (string, required): Module identifier where this content belongs
- `lesson_id` (string, required): Lesson identifier within the module
- `section_id` (string, required): Section identifier within the lesson
- `content` (string, required): The actual text content of the chunk
- `embedding` (array of floats): Vector embedding of the content
- `metadata` (object): Additional metadata about the content

**Validation Rules**:
- All ID fields must be non-empty strings
- `content` must be 10-10000 characters
- `embedding` must match the dimensionality of the Cohere model used

### Search Result Model
**Purpose**: Represents a single search result returned to the user

**Fields**:
- `content_chunk` (ContentChunk, required): The matched content chunk
- `similarity_score` (float, required): Similarity score between query and result (0.0-1.0)
- `metadata` (object, required): Complete metadata including module, lesson, section, and chunk identifiers
- `relevance_rank` (integer, required): Rank position in the result set (1-indexed)

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `relevance_rank` must be >= 1
- All metadata fields from the content chunk must be preserved

### Search Results Model
**Purpose**: Represents the complete set of search results

**Fields**:
- `query` (string, required): The original query text
- `results` (array of SearchResult, required): Array of ranked search results
- `total_results` (integer, required): Total number of results found before ranking
- `execution_time_ms` (float, required): Time taken to execute the search in milliseconds

**Validation Rules**:
- `results` array length must match the actual number of results
- `total_results` must be >= length of `results` array
- `execution_time_ms` must be >= 0

## Relationships

```
Query --[1-to-many]--> Search Results --[1-to-many]--> Search Result --[1-to-1]--> Content Chunk
```

## State Transitions

### Query Processing Flow
1. **Input State**: Query received with text and optional parameters
2. **Embedding State**: Query converted to vector embedding using Cohere
3. **Search State**: Qdrant queried for similar content chunks
4. **Filtering State**: Results filtered by similarity threshold and deduplicated
5. **Ranking State**: Results sorted by similarity score in descending order
6. **Output State**: Results returned with complete metadata

## Data Validation

### Input Validation
- Query text length: 1-500 characters
- Filters: Valid content type identifiers only
- Threshold: 0.0-1.0 range
- Top-k: 1-50 range

### Processing Validation
- Embedding dimensionality: Must match Cohere model output
- Similarity scores: Must be in 0.0-1.0 range after filtering
- Metadata completeness: All required identifiers must be present

### Output Validation
- Result count: Must not exceed requested top-k
- Ranking order: Results must be in descending similarity order
- Metadata preservation: All original metadata must be included