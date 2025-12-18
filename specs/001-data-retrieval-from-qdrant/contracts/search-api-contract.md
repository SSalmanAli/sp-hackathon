# OpenAPI Contract: Search API for Qdrant Retrieval

**Feature**: Data Retrieval from Qdrant Vector Database
**Date**: 2025-12-17
**Branch**: 001-data-retrieval-from-qdrant

## Overview

This document specifies the OpenAPI contract for the semantic search API that retrieves Physical AI Book content from the Qdrant vector database.

## OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: Physical AI Book Search API
  description: API for semantic search of Physical AI Book content using Qdrant vector database
  version: 1.0.0
  contact:
    name: Physical AI Book Team

servers:
  - url: https://api.physical-ai-book.com/v1
    description: Production server
  - url: https://staging-api.physical-ai-book.com/v1
    description: Staging server

paths:
  /search:
    post:
      summary: Perform semantic search on Physical AI Book content
      description: Accepts a natural language query and returns semantically similar content chunks from the Physical AI Book
      operationId: searchContent
      tags:
        - search
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/SearchRequest'
            example:
              query: "How do I implement a PID controller for robot motor control?"
              filters:
                content_types: ["lesson", "exercise"]
              similarity_threshold: 0.7
              top_k: 10
      responses:
        '200':
          description: Successful search response
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/SearchResponse'
              example:
                query: "How do I implement a PID controller for robot motor control?"
                results:
                  - content_chunk:
                      chunk_id: "chunk-001"
                      module_id: "module-001"
                      lesson_id: "lesson-003"
                      section_id: "section-002"
                      content: "PID controller implementation for robot motor control involves three components: Proportional, Integral, and Derivative terms..."
                      metadata:
                        title: "PID Controller Implementation"
                        author: "Physical AI Team"
                        created_at: "2024-12-01"
                    similarity_score: 0.92
                    metadata:
                      module: "Robot Control Systems"
                      lesson: "Motor Control Algorithms"
                      section: "PID Controllers"
                      chunk: "chunk-001"
                    relevance_rank: 1
                  - content_chunk:
                      chunk_id: "chunk-002"
                      module_id: "module-001"
                      lesson_id: "lesson-005"
                      section_id: "section-001"
                      content: "When implementing PID controllers for robotics, consider the trade-offs between stability and response time..."
                      metadata:
                        title: "PID Controller Considerations"
                        author: "Physical AI Team"
                        created_at: "2024-12-02"
                    similarity_score: 0.87
                    metadata:
                      module: "Robot Control Systems"
                      lesson: "Advanced Control Techniques"
                      section: "PID Controllers"
                      chunk: "chunk-002"
                    relevance_rank: 2
                total_results: 2
                execution_time_ms: 45.2
        '400':
          description: Bad request - invalid query parameters
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Internal server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
      security:
        - api_key: []

components:
  schemas:
    SearchRequest:
      type: object
      required:
        - query
      properties:
        query:
          type: string
          description: The natural language search query
          example: "How do I implement a PID controller for robot motor control?"
          minLength: 1
          maxLength: 500
        filters:
          type: object
          description: Optional filters for the search
          properties:
            content_types:
              type: array
              items:
                type: string
                enum: [module, lesson, section, exercise]
              description: Content types to include in search
              example: ["lesson", "exercise"]
        similarity_threshold:
          type: number
          description: Minimum similarity threshold for results (0.0 to 1.0)
          minimum: 0.0
          maximum: 1.0
          default: 0.7
          example: 0.7
        top_k:
          type: integer
          description: Number of top results to return
          minimum: 1
          maximum: 50
          default: 10
          example: 10

    SearchResponse:
      type: object
      required:
        - query
        - results
        - total_results
        - execution_time_ms
      properties:
        query:
          type: string
          description: The original search query
          example: "How do I implement a PID controller for robot motor control?"
        results:
          type: array
          items:
            $ref: '#/components/schemas/SearchResult'
        total_results:
          type: integer
          description: Total number of results found before ranking
          example: 2
        execution_time_ms:
          type: number
          description: Time taken to execute the search in milliseconds
          example: 45.2

    SearchResult:
      type: object
      required:
        - content_chunk
        - similarity_score
        - metadata
        - relevance_rank
      properties:
        content_chunk:
          $ref: '#/components/schemas/ContentChunk'
        similarity_score:
          type: number
          description: Similarity score between query and result (0.0-1.0)
          minimum: 0.0
          maximum: 1.0
          example: 0.92
        metadata:
          type: object
          description: Complete metadata with module, lesson, section, and chunk identifiers
          properties:
            module:
              type: string
              description: Module name
              example: "Robot Control Systems"
            lesson:
              type: string
              description: Lesson name
              example: "Motor Control Algorithms"
            section:
              type: string
              description: Section name
              example: "PID Controllers"
            chunk:
              type: string
              description: Chunk identifier
              example: "chunk-001"
        relevance_rank:
          type: integer
          description: Rank position in the result set (1-indexed)
          minimum: 1
          example: 1

    ContentChunk:
      type: object
      required:
        - chunk_id
        - module_id
        - lesson_id
        - section_id
        - content
      properties:
        chunk_id:
          type: string
          description: Unique identifier for the content chunk
          example: "chunk-001"
        module_id:
          type: string
          description: Module identifier where this content belongs
          example: "module-001"
        lesson_id:
          type: string
          description: Lesson identifier within the module
          example: "lesson-003"
        section_id:
          type: string
          description: Section identifier within the lesson
          example: "section-002"
        content:
          type: string
          description: The actual text content of the chunk
          example: "PID controller implementation for robot motor control involves three components: Proportional, Integral, and Derivative terms..."
        metadata:
          type: object
          description: Additional metadata about the content
          properties:
            title:
              type: string
              description: Title of the content
              example: "PID Controller Implementation"
            author:
              type: string
              description: Author of the content
              example: "Physical AI Team"
            created_at:
              type: string
              format: date
              description: Creation date of the content
              example: "2024-12-01"

    ErrorResponse:
      type: object
      required:
        - error
        - message
      properties:
        error:
          type: string
          description: Error code
          example: "INVALID_QUERY"
        message:
          type: string
          description: Human-readable error message
          example: "Query must be between 1 and 500 characters"

  securitySchemes:
    api_key:
      type: apiKey
      name: X-API-Key
      in: header
      description: API key for authentication
```

## Contract Validation Requirements

1. All request parameters must be validated according to schema definitions
2. Response bodies must conform to the specified schemas
3. HTTP status codes must match the documented responses
4. Error responses must follow the ErrorResponse schema
5. API must authenticate using the specified API key mechanism